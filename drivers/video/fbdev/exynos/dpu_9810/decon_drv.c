// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos9810 DECON-F fbdev driver (Galaxy Note9 "crownlte"), adopt mode.
 *
 * SBoot leaves DECON-F, DSIM and the S6E3HA8 command-mode panel running and
 * scanning a 1440x2960 buffer at 0xcc000000. This driver inherits that
 * state: it never touches power, clocks, DSIM, the MIPI PHY, the panel or
 * video timing. It registers /dev/graphics/fb0 with the 4.9 dpu_9810 ioctl
 * ABI the vendor HWC (libexynosdisplay.so) uses, the 16030000.decon_f
 * "vsync" and "psr_info" sysfs files, TE-EINT vsync events, and applies
 * S3CFB_WIN_CONFIG(_OLD) window configurations through the DPP channels for
 * physically contiguous dma-bufs (no SysMMU yet).
 *
 * Register sequences follow 4.9 __decon_update_regs(): wait for the
 * previous shadow update, mask the TE trigger, program the DPPs, then the
 * DECON windows, request a global shadow update, unmask the trigger, and
 * wait for the update to latch at the next TE-triggered frame. Unlike 4.9
 * the trigger is then left unmasked (continuous refresh), so fbcon drawing
 * into fb0 memory stays visible.
 *
 * Exception, opt-in by DT (panel node samsung,fault-display-brightness):
 * once after adopt, panel_bright.c sends the owner's 4.9 brightness-44700
 * panel sequence through DSIM0 (dsim_cmd.c), inside a command window with
 * the TE trigger masked and no frame in flight.
 */
#include <linux/aperture.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-direct.h>
#include <linux/dma-mapping.h>
#include <linux/file.h>
#include <linux/font.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sync_file.h>
#include <linux/uaccess.h>

#include "dpu9810.h"

#define DPU_SHADOW_PRE_US	50000
#define DPU_LATCH_US		100000
#define DPU_ACQ_FENCE_MS	900
#define DPU_VSYNC_TIMEOUT_MS	200
#define DPU_HWC_INIT		0

/* ---- fences --------------------------------------------------------- */

static const char *dpu_fence_driver(struct dma_fence *f)
{
	return "exynos9810-decon";
}

static const char *dpu_fence_timeline(struct dma_fence *f)
{
	return "decon0";
}

static const struct dma_fence_ops dpu_fence_ops = {
	.get_driver_name = dpu_fence_driver,
	.get_timeline_name = dpu_fence_timeline,
};

static struct dma_fence *dpu_fence_create(struct dpu_dev *d)
{
	struct dma_fence *f = kzalloc(sizeof(*f), GFP_KERNEL);

	if (f)
		dma_fence_init(f, &dpu_fence_ops, &d->fence_lock, d->fence_ctx,
			       atomic64_inc_return(&d->fence_seqno));
	return f;
}

/* ---- buffers -------------------------------------------------------- */

static void dpu_buf_release(struct dpu_buf *b)
{
	if (b->sgt)
		dma_buf_unmap_attachment_unlocked(b->att, b->sgt, DMA_TO_DEVICE);
	if (b->att)
		dma_buf_detach(b->dmabuf, b->att);
	if (b->dmabuf)
		dma_buf_put(b->dmabuf);
	memset(b, 0, sizeof(*b));
}

/*
 * Import one plane. Without a SysMMU the DPP takes a physical address, so
 * the mapped buffer must be one contiguous run below 4 GiB.
 */
static int dpu_buf_import(struct dpu_dev *d, struct dma_buf *db, struct dpu_buf *b,
			  u64 need)
{
	struct scatterlist *sg;
	dma_addr_t addr;
	u64 len = 0;
	int i, ret;

	b->dmabuf = db;
	b->att = dma_buf_attach(db, d->dev);
	if (IS_ERR(b->att)) {
		ret = PTR_ERR(b->att);
		b->att = NULL;
		return ret;
	}
	b->sgt = dma_buf_map_attachment_unlocked(b->att, DMA_TO_DEVICE);
	if (IS_ERR(b->sgt)) {
		ret = PTR_ERR(b->sgt);
		b->sgt = NULL;
		return ret;
	}
	addr = sg_dma_address(b->sgt->sgl);
	for_each_sgtable_dma_sg(b->sgt, sg, i) {
		if (sg_dma_address(sg) != addr + len) {
			dev_warn_ratelimited(d->dev, "buffer not contiguous (segment %d); needs the DPU SysMMU\n",
					     i);
			return -EINVAL;
		}
		len += sg_dma_len(sg);
	}
	if (len < need || addr + need > SZ_4G) {
		dev_warn_ratelimited(d->dev, "buffer %pad+%#llx unusable (need %#llx below 4 GiB)\n",
				     &addr, len, need);
		return -EINVAL;
	}
	b->addr = addr;
	b->len = len;
	return 0;
}

static void dpu_commit_free(struct dpu_commit *c)
{
	int i;

	if (!c)
		return;
	for (i = 0; i < DECON_MAX_WIN; i++) {
		dpu_buf_release(&c->buf[i]);
		if (c->acq[i])
			dma_fence_put(c->acq[i]);
	}
	if (c->fence)
		dma_fence_put(c->fence);
	kfree(c);
}

/* ---- validation (4.9 decon_prepare_win_config / check_limitation) --- */

static int dpu_check_buffer_win(struct dpu_dev *d, int i, const struct decon_win_config *w,
				u32 dpp_fmt)
{
	const struct decon_frame *s = &w->src, *t = &w->dst;
	int bpp = dpu_fmt_bpp(dpp_fmt);

	if (w->idma_type >= DPU_NR_DPP || !bpp)
		return -EINVAL;
	if (s->w < 16 || s->h < 16 || s->w > 4096 || s->h > 4096 ||
	    s->f_w < 16 || s->f_h < 16 || s->f_w > 65534 || s->f_h > 8190 ||
	    s->x < 0 || s->y < 0 || s->x + s->w > s->f_w || s->y + s->h > s->f_h)
		return -EINVAL;
	if (t->x < 0 || t->y < 0 || t->x + t->w > d->lcd.xres ||
	    t->y + t->h > d->lcd.yres)
		return -EINVAL;
	/* no scaler, rotation, AFBC, protected or block-area support yet */
	if (t->w != s->w || t->h != s->h || w->dpp_parm.rot || w->compression ||
	    w->protection || w->blending > DECON_BLENDING_COVERAGE)
		return -EINVAL;
	if (bpp == 16 && (s->x & 1))
		return -EINVAL;
	return 0;
}

static struct dpu_commit *dpu_build_commit(struct dpu_dev *d,
					   struct decon_win_config_data *wd,
					   struct dma_buf **kbufs, int *err)
{
	struct dpu_commit *c = kzalloc(sizeof(*c), GFP_KERNEL);
	unsigned long used = 0;
	int i, ret = 0;

	if (!c) {
		*err = -ENOMEM;
		return NULL;
	}
	c->internal = !!kbufs;

	for (i = 0; i < DECON_MAX_WIN && !ret; i++) {
		struct decon_win_config *w = &wd->config[i];
		struct dma_buf *db;
		u64 need;

		c->cfg[i] = *w;
		switch (w->state) {
		case DECON_WIN_STATE_DISABLED:
			break;
		case DECON_WIN_STATE_COLOR:
			if (w->dst.x < 0 || w->dst.y < 0 || !w->dst.w || !w->dst.h ||
			    w->dst.x + w->dst.w > d->lcd.xres ||
			    w->dst.y + w->dst.h > d->lcd.yres)
				ret = -EINVAL;
			else
				c->nr_win++;
			break;
		case DECON_WIN_STATE_BUFFER:
		case DECON_WIN_STATE_CURSOR:
			c->cfg[i].state = DECON_WIN_STATE_BUFFER;
			c->dpp_fmt[i] = dpu_translate_fmt_to_dpp(w->format);
			ret = dpu_check_buffer_win(d, i, w, c->dpp_fmt[i]);
			if (ret)
				break;
			if (test_and_set_bit(w->idma_type, &used)) {
				ret = -EINVAL;	/* 4.9: channel mapped to multiple windows */
				break;
			}
			if (kbufs) {
				db = kbufs[i];
				if (!db) {
					ret = -EINVAL;
					break;
				}
				get_dma_buf(db);
			} else {
				db = dma_buf_get(w->fd_idma[0]);
				if (IS_ERR(db)) {
					ret = PTR_ERR(db);
					break;
				}
			}
			need = (u64)w->src.f_w * w->src.f_h * dpu_fmt_bpp(c->dpp_fmt[i]) / 8;
			ret = dpu_buf_import(d, db, &c->buf[i], need);
			if (ret)
				break;
			if (!kbufs && w->acq_fence >= 0) {
				c->acq[i] = sync_file_get_fence(w->acq_fence);
				if (!c->acq[i])
					ret = -EINVAL;
			}
			c->nr_win++;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		if (ret)
			dev_warn_ratelimited(d->dev, "win%d rejected (state %u fmt %u idma %u src %ux%u@%d,%d f %ux%u dst %ux%u@%d,%d): %d\n",
					     i, w->state, w->format, w->idma_type,
					     w->src.w, w->src.h, w->src.x, w->src.y,
					     w->src.f_w, w->src.f_h, w->dst.w, w->dst.h,
					     w->dst.x, w->dst.y, ret);
	}
	if (ret) {
		dpu_commit_free(c);
		*err = ret;
		return NULL;
	}
	return c;
}

/* ---- update worker -------------------------------------------------- */

static int dpu_apply(struct dpu_dev *d, struct dpu_commit *c)
{
	ktime_t t0;
	int i, ret;

	mutex_lock(&d->hw_lock);
	if (!dpu_hw_powered(d)) {
		ret = -EIO;
		goto out;
	}
	/* 4.9: the previous shadow update must have latched */
	if (dpu_hw_wait_shadow(d, DPU_SHADOW_PRE_US)) {
		dev_warn_ratelimited(d->dev, "previous shadow update still pending, frame skipped\n");
		ret = -EBUSY;		/* nothing programmed */
		goto out;
	}
	c->fc_before = dpu_hw_frame_count(d);
	dpu_hw_trigger(d, false);

	/* 4.9 (exynos9810): program the DPPs before enabling DECON windows */
	for (i = 0; i < DECON_MAX_WIN; i++) {
		if (c->cfg[i].state != DECON_WIN_STATE_BUFFER)
			continue;
		ret = dpu_hw_dpp_config(d, c->cfg[i].idma_type, &c->cfg[i],
					c->dpp_fmt[i], c->buf[i].addr);
		if (ret) {
			dev_warn(d->dev, "dpp%u config failed (%d), win%d disabled\n",
				 c->cfg[i].idma_type, ret, i);
			c->cfg[i].state = DECON_WIN_STATE_DISABLED;
		}
	}
	for (i = 0; i < DECON_MAX_WIN; i++)
		dpu_hw_win_config(d, i, &c->cfg[i],
				  c->cfg[i].state != DECON_WIN_STATE_DISABLED);

	dpu_hw_update_req(d);
	dpu_hw_trigger(d, true);
	t0 = ktime_get();
	ret = dpu_hw_wait_shadow(d, DPU_LATCH_US);
	c->latch_us = ktime_us_delta(ktime_get(), t0);
	c->fc_after = dpu_hw_frame_count(d);
	if (ret) {
		d->nr_latch_timeouts++;
		dev_warn_ratelimited(d->dev, "shadow update did not latch in %u us (FRAME_COUNT %u -> %u)\n",
				     DPU_LATCH_US, c->fc_before, c->fc_after);
	} else {
		/* "latched" must mean the IDMA now scans the new buffer */
		for (i = 0; i < DECON_MAX_WIN; i++) {
			u32 act;

			if (c->cfg[i].state != DECON_WIN_STATE_BUFFER)
				continue;
			act = dpu_hw_dma_active_rd(d, c->cfg[i].idma_type, IDMA_IN_BASE_ADDR_Y);
			if (act == lower_32_bits(c->buf[i].addr))
				continue;
			d->nr_base_mismatch++;
			dev_warn_ratelimited(d->dev, "win%d dpp%u: active IDMA base %#x, programmed %pad\n",
					     i, c->cfg[i].idma_type, act, &c->buf[i].addr);
		}
	}
out:
	mutex_unlock(&d->hw_lock);
	return ret;
}

static void dpu_process(struct dpu_dev *d, struct dpu_commit *c)
{
	struct dpu_commit *old = NULL;
	int i;

	for (i = 0; i < DECON_MAX_WIN; i++) {
		if (!c->acq[i])
			continue;
		if (dma_fence_wait_timeout(c->acq[i], false,
					   msecs_to_jiffies(DPU_ACQ_FENCE_MS)) <= 0)
			dev_warn_ratelimited(d->dev, "win%d acquire fence timeout\n", i);
		dma_fence_put(c->acq[i]);
		c->acq[i] = NULL;
	}

	c->result = dpu_apply(d, c);
	d->nr_commits++;

	if (c->fence) {
		if (c->result)
			dma_fence_set_error(c->fence, c->result);
		dma_fence_signal(c->fence);
	}

	/*
	 * Once the new configuration latched, the previous buffers are no
	 * longer read (4.9 releases them after the update; HWC2 release fence
	 * semantics). If nothing was programmed (-EBUSY, -EIO) the new
	 * buffers were never used. If the update was requested but has not
	 * latched yet (-ETIMEDOUT), DECON may read either set: keep both
	 * until a later update latches.
	 */
	if (c->result == 0) {
		struct dpu_commit *p, *n;

		list_for_each_entry_safe(p, n, &d->stale, node) {
			list_del(&p->node);
			dpu_commit_free(p);
		}
		old = d->active;
		d->active = c;
	} else if (c->result == -ETIMEDOUT) {
		if (d->active)
			list_add_tail(&d->active->node, &d->stale);
		d->active = c;
	} else {
		old = c;
	}
	dpu_commit_free(old);
}

static void dpu_update_work(struct kthread_work *work)
{
	struct dpu_dev *d = container_of(work, struct dpu_dev, work);
	struct dpu_commit *c;

	for (;;) {
		spin_lock(&d->queue_lock);
		c = list_first_entry_or_null(&d->queue, struct dpu_commit, node);
		if (c)
			list_del(&c->node);
		spin_unlock(&d->queue_lock);
		if (!c)
			break;
		dpu_process(d, c);
	}
}

/*
 * S3CFB_WIN_CONFIG(_OLD) common path. User requests pass kbufs = NULL and
 * get retire/release fence fds back in @wd; kernel requests (fb takeover,
 * self-test) pass dma-bufs per window and get the dma_fence in @kfence.
 */
static int dpu_win_config(struct dpu_dev *d, struct decon_win_config_data *wd,
			  struct dma_buf **kbufs, struct dma_fence **kfence)
{
	struct sync_file *sf = NULL;
	int rel[DECON_MAX_WIN], retire = -1;
	struct dpu_commit *c;
	int i, ret = 0, queued;

	for (i = 0; i < DECON_MAX_WIN; i++)
		rel[i] = -1;

	mutex_lock(&d->lock);
	c = dpu_build_commit(d, wd, kbufs, &ret);
	if (!c) {
		d->nr_rejects++;
		goto out;
	}

	if (c->nr_win) {
		c->fence = dpu_fence_create(d);
		if (!c->fence) {
			ret = -ENOMEM;
			goto err_free;
		}
	}

	if (!kbufs && c->fence) {
		sf = sync_file_create(c->fence);
		if (!sf) {
			ret = -ENOMEM;
			goto err_free;
		}
		retire = get_unused_fd_flags(O_CLOEXEC);
		if (retire < 0) {
			ret = retire;
			goto err_sf;
		}
		for (i = 0; i < DECON_MAX_WIN; i++) {
			if (c->cfg[i].state != DECON_WIN_STATE_BUFFER)
				continue;
			rel[i] = get_unused_fd_flags(O_CLOEXEC);
			if (rel[i] < 0) {
				ret = rel[i];
				goto err_fds;
			}
		}
	}
	if (kfence)
		*kfence = c->fence ? dma_fence_get(c->fence) : NULL;

	c->queued = ktime_get();
	spin_lock(&d->queue_lock);
	list_add_tail(&c->node, &d->queue);
	queued = list_count_nodes(&d->queue);
	spin_unlock(&d->queue_lock);
	kthread_queue_work(d->worker, &d->work);

	/* fds become visible only after the commit is queued (4.9 order) */
	if (sf) {
		for (i = 0; i < DECON_MAX_WIN; i++)
			if (rel[i] >= 0)
				fd_install(rel[i], get_file(sf->file));
		fd_install(retire, sf->file);
	}
	wd->retire_fence = retire;
	for (i = 0; i < DECON_MAX_WIN; i++)
		wd->config[i].rel_fence = rel[i];
	wd->extra.remained_frames = queued;
	mutex_unlock(&d->lock);
	return 0;

err_fds:
	for (i = 0; i < DECON_MAX_WIN; i++)
		if (rel[i] >= 0)
			put_unused_fd(rel[i]);
	put_unused_fd(retire);
err_sf:
	fput(sf->file);
err_free:
	if (c->fence) {
		dma_fence_set_error(c->fence, ret);
		dma_fence_signal(c->fence);
	}
	dpu_commit_free(c);
out:
	wd->retire_fence = -1;
	for (i = 0; i < DECON_MAX_WIN; i++)
		wd->config[i].rel_fence = -1;
	mutex_unlock(&d->lock);
	return ret;
}

/* ---- vsync (4.9 decon_ext_irq_handler + decon_vsync_thread) --------- */

static irqreturn_t dpu_te_irq(int irq, void *data)
{
	struct dpu_dev *d = data;
	ktime_t now = ktime_get();

	/* the panel keeps sending TE at 60 Hz: a longer gap = IRQs held off */
	if (READ_ONCE(d->te_gap_track)) {
		u32 us = ktime_us_delta(now, d->vsync_ts);

		if (us > d->te_gap_max_us)
			d->te_gap_max_us = us;
	}
	d->vsync_ts = now;
	atomic_inc(&d->te_count);
	wake_up_interruptible_all(&d->vsync_wait);
	return IRQ_HANDLED;
}

static int dpu_vsync_thread(void *data)
{
	struct dpu_dev *d = data;

	while (!kthread_should_stop()) {
		ktime_t ts = READ_ONCE(d->vsync_ts);
		int ret = wait_event_interruptible(d->vsync_wait,
				kthread_should_stop() ||
				(READ_ONCE(d->vsync_active) &&
				 ts != READ_ONCE(d->vsync_ts)));

		if (!ret && !kthread_should_stop())
			sysfs_notify(&d->dev->kobj, NULL, "vsync");
	}
	return 0;
}

static int dpu_wait_for_vsync(struct dpu_dev *d, u32 timeout_ms)
{
	ktime_t ts = READ_ONCE(d->vsync_ts);
	long ret;

	if (d->te_irq < 0) {
		usleep_range(16000, 17000);
		return 0;
	}
	ret = wait_event_interruptible_timeout(d->vsync_wait,
			ts != READ_ONCE(d->vsync_ts), msecs_to_jiffies(timeout_ms));
	if (ret < 0)
		return ret;
	return ret ? 0 : -ETIMEDOUT;
}

static ssize_t vsync_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dpu_dev *d = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%llu\n", ktime_to_ns(READ_ONCE(d->vsync_ts)));
}
static DEVICE_ATTR_RO(vsync);

/* 4.9 decon_show_psr_info(); adopt mode exposes the boot mode only */
static ssize_t psr_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dpu_dev *d = dev_get_drvdata(dev);
	const struct dpu_lcd *l = &d->lcd;

	return sysfs_emit(buf, "%d\n%d\n%u\n%u\n%u\n%u\n%u\n", DPU_PSR_MIPI_COMMAND, 1,
			  l->xres, l->yres, l->dsc_en ? l->dsc_w : 8,
			  l->dsc_en ? l->dsc_h : 1, l->dsc_en);
}
static DEVICE_ATTR_RO(psr_info);

static struct attribute *dpu_attrs[] = {
	&dev_attr_vsync.attr,
	&dev_attr_psr_info.attr,
	NULL
};
ATTRIBUTE_GROUPS(dpu);

/* ---- ioctls ---------------------------------------------------------- */

static void dpu_fill_mres(struct dpu_dev *d, struct decon_lcd_mres_info *m)
{
	memset(m, 0, sizeof(*m));
	m->mres_en = 0;			/* no DSU/multi-resolution in adopt mode */
	m->mres_number = 1;
	m->res_info[0].width = d->lcd.xres;
	m->res_info[0].height = d->lcd.yres;
	m->res_info[0].dsc_en = d->lcd.dsc_en;
	m->res_info[0].dsc_width = d->lcd.dsc_w;
	m->res_info[0].dsc_height = d->lcd.dsc_h;
}

/* 4.9 decon_get_edid() for DSI: a fixed "samsung lcd" EDID block */
static void dpu_fill_edid(struct decon_edid_data *e)
{
	static const u8 hdr[8] = { 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00 };
	static const u8 name[13] = {	/* "samsung lcd  " */
		0x73, 0x61, 0x6d, 0x73, 0x75, 0x6e, 0x67, 0x20, 0x6c, 0x63, 0x64, 0x20, 0x20
	};
	u8 *b = e->edid_data, sum = 0;
	int i;

	memset(e, 0, sizeof(*e));
	memcpy(b, hdr, sizeof(hdr));
	b[8] = 0x4c;			/* mfg_id */
	b[9] = 0x2d;
	b[16] = 0x10;			/* week 16 */
	b[17] = 0x1e;			/* 1990 + 30 */
	b[54 + 3] = 0xfc;		/* detailed_timings[0]: display name */
	memcpy(&b[54 + 5], name, sizeof(name));
	for (i = 0; i < 127; i++)
		sum += b[i];
	b[127] = 0x100 - sum;
	e->size = 128;
}

static int dpu_ioctl_win_config(struct dpu_dev *d, unsigned int cmd, void __user *argp)
{
	struct decon_win_config_data *wd;
	size_t sz = _IOC_SIZE(cmd);
	int ret;

	wd = kzalloc(sizeof(*wd), GFP_KERNEL);
	if (!wd)
		return -ENOMEM;
	/* the old 1416-byte layout is a prefix of the new one */
	if (copy_from_user(wd, argp, sz)) {
		ret = -EFAULT;
		goto out;
	}
	ret = dpu_win_config(d, wd, NULL, NULL);
	if (!ret && copy_to_user(argp, wd, sz))
		ret = -EFAULT;
out:
	kfree(wd);
	return ret;
}

static int dpu_fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct dpu_dev *d = info->par;
	void __user *argp = (void __user *)arg;
	union {
		struct decon_disp_info di;
		struct decon_hdr_capabilities hc;
		struct decon_hdr_capabilities_info hci;
		struct decon_display_mode dm;
		struct decon_color_mode_info cm;
		struct decon_edid_data edid;
		struct vsync_applied_time_data vt;
	} u;
	u32 v;
	bool active;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		if (get_user(v, (u32 __user *)argp))
			return -EFAULT;
		return v ? -ENODEV : dpu_wait_for_vsync(d, DPU_VSYNC_TIMEOUT_MS);

	case S3CFB_SET_VSYNC_INT:
		if (get_user(active, (bool __user *)argp))
			return -EFAULT;
		WRITE_ONCE(d->vsync_active, active);
		wake_up_interruptible_all(&d->vsync_wait);
		return 0;

	case S3CFB_WIN_CONFIG_OLD:
	case S3CFB_WIN_CONFIG:
		return dpu_ioctl_win_config(d, cmd, argp);

	case S3CFB_GET_HDR_CAPABILITIES:
		/* the DPP HDR path is not ported: advertise none */
		memset(&u.hc, 0, sizeof(u.hc));
		return copy_to_user(argp, &u.hc, sizeof(u.hc)) ? -EFAULT : 0;

	case S3CFB_GET_HDR_CAPABILITIES_NUM:
		memset(&u.hci, 0, sizeof(u.hci));
		return copy_to_user(argp, &u.hci, sizeof(u.hci)) ? -EFAULT : 0;

	case EXYNOS_DISP_INFO:
		if (copy_from_user(&u.di, argp, sizeof(u.di)))
			return -EFAULT;
		if (d->hwc_ver != u.di.ver) {
			d->hwc_ver = u.di.ver;
			dev_info(d->dev, "decon is being set up by HWC%u.0\n", u.di.ver);
		}
		u.di.psr_mode = DPU_PSR_MIPI_COMMAND;
		u.di.chip_ver = 9810;
		dpu_fill_mres(d, &u.di.mres_info);
		return copy_to_user(argp, &u.di, sizeof(u.di)) ? -EFAULT : 0;

	case EXYNOS_GET_DISPLAYPORT_CONFIG:
	case EXYNOS_SET_DISPLAYPORT_CONFIG:
		return -ENODEV;		/* no DisplayPort on decon_f */

	case EXYNOS_DPU_DUMP:
		dev_info(d->dev, "dump: FRAME_COUNT=%u TRIG=%#x DP0=%#x DP1=%#x commits=%u timeouts=%u rejects=%u te=%d\n",
			 dpu_hw_frame_count(d), decon_rd(d, HW_SW_TRIG_CONTROL),
			 decon_rd(d, DATA_PATH_CONTROL_0), decon_rd(d, DATA_PATH_CONTROL_1),
			 d->nr_commits, d->nr_latch_timeouts, d->nr_rejects,
			 atomic_read(&d->te_count));
		return 0;

	case S3CFB_POWER_MODE:
		if (get_user(v, (u32 __user *)argp))
			return -EFAULT;
		/* doze needs the panel driver; 4.9 ignores other modes too */
		dev_dbg(d->dev, "power mode %u ignored in adopt mode\n", v);
		return 0;

	case DECON_WIN_CURSOR_POS:
		return -EPERM;		/* 4.9: decon_f does not support cursor */

	case EXYNOS_GET_COLOR_MODE_NUM:
		v = 1;			/* HAL_COLOR_MODE_NUM_MAX */
		return put_user(v, (u32 __user *)argp);

	case EXYNOS_GET_COLOR_MODE:
		if (copy_from_user(&u.cm, argp, sizeof(u.cm)))
			return -EFAULT;
		if (u.cm.index != 0)
			return -EINVAL;
		u.cm.color_mode = 0;	/* HAL_COLOR_MODE_NATIVE */
		return copy_to_user(argp, &u.cm, sizeof(u.cm)) ? -EFAULT : 0;

	case EXYNOS_SET_COLOR_MODE:
		if (get_user(v, (u32 __user *)argp))
			return -EFAULT;
		return v ? -EINVAL : 0;

	case EXYNOS_GET_EDID:
		dpu_fill_edid(&u.edid);
		return copy_to_user(argp, &u.edid, sizeof(u.edid)) ? -EFAULT : 0;

	case EXYNOS_GET_DISPLAY_MODE_NUM:
		v = 1;
		return put_user(v, (u32 __user *)argp);

	case EXYNOS_GET_DISPLAY_MODE:
		if (copy_from_user(&u.dm, argp, sizeof(u.dm)))
			return -EFAULT;
		if (u.dm.index != 0)
			return -EINVAL;
		u.dm.width = d->lcd.xres;
		u.dm.height = d->lcd.yres;
		u.dm.mm_width = d->lcd.mm_w;
		u.dm.mm_height = d->lcd.mm_h;
		u.dm.fps = d->lcd.fps;
		u.dm.group = 0;
		return copy_to_user(argp, &u.dm, sizeof(u.dm)) ? -EFAULT : 0;

	case EXYNOS_SET_DISPLAY_MODE:
		if (copy_from_user(&u.dm, argp, sizeof(u.dm)))
			return -EFAULT;
		return u.dm.index ? -EINVAL : 0;

	case EXYNOS_GET_DISPLAY_CURRENT_MODE:
		v = 0;
		return put_user(v, (u32 __user *)argp);

	case EXYNOS_GET_VSYNC_CHANGE_TIMELINE:
		return -EPERM;		/* 4.9: "MRES is not enabled" */

	default:
		return -ENOTTY;
	}
}

/* ---- fbdev ----------------------------------------------------------- */

static int dpu_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	if (var->xres != info->var.xres || var->yres != info->var.yres ||
	    var->bits_per_pixel != 32 || var->xoffset || var->yoffset)
		return -EINVAL;
	*var = info->var;
	return 0;
}

static int dpu_fb_set_par(struct fb_info *info)
{
	return 0;
}

static int dpu_fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			    u_int transp, struct fb_info *info)
{
	u32 *pal = info->pseudo_palette;

	if (regno >= 16)
		return -EINVAL;
	pal[regno] = 0xff000000 | ((red >> 8) << 16) | ((green >> 8) << 8) | (blue >> 8);
	return 0;
}

static int dpu_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	return (var->xoffset || var->yoffset) ? -EINVAL : 0;
}

static int dpu_fb_blank(int mode, struct fb_info *info)
{
	struct dpu_dev *d = info->par;

	/* panel power is not ours yet: blanking is accepted and ignored */
	dev_info_once(d->dev, "blank %d ignored (adopt mode, no panel driver)\n", mode);
	return 0;
}

static const struct fb_ops dpu_fb_ops = {
	.owner		= THIS_MODULE,
	FB_DEFAULT_IOMEM_OPS,
	.fb_check_var	= dpu_fb_check_var,
	.fb_set_par	= dpu_fb_set_par,
	.fb_setcolreg	= dpu_fb_setcolreg,
	.fb_pan_display	= dpu_fb_pan_display,
	.fb_blank	= dpu_fb_blank,
	.fb_ioctl	= dpu_fb_ioctl,
};

/* fb0 window, as 4.9 decon_fb init: DPP BGRA_8888 (= user ARGB_8888) */
static void dpu_fb_window(struct dpu_dev *d, struct decon_win_config_data *wd)
{
	struct decon_win_config *w = &wd->config[d->boot_win];

	memset(wd, 0, sizeof(*wd));
	w->state = DECON_WIN_STATE_BUFFER;
	w->idma_type = d->boot_dpp;
	w->format = DECON_PIXEL_FORMAT_ARGB_8888;
	w->blending = DECON_BLENDING_NONE;
	w->plane_alpha = 0xff;
	w->src.w = w->src.f_w = w->dst.w = w->dst.f_w = d->lcd.xres;
	w->src.h = w->src.f_h = w->dst.h = w->dst.f_h = d->lcd.yres;
}

/* ---- in-kernel dma-buf for physically contiguous kernel memory -------- */

struct dpu_kbuf {
	struct device *dev;
	void *vaddr;
	dma_addr_t dma;
	size_t size;
	bool owned;		/* allocated with dma_alloc_wc */
};

static struct sg_table *dpu_kbuf_map(struct dma_buf_attachment *att,
				     enum dma_data_direction dir)
{
	struct dpu_kbuf *kb = att->dmabuf->priv;
	struct sg_table *sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	int ret;

	if (!sgt)
		return ERR_PTR(-ENOMEM);
	ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (ret)
		goto err;
	/*
	 * No IOMMU and no dma-ranges on this device: DMA address == physical
	 * address. The fb0 region is no-map, so only CPU-sync-free mappings
	 * are allowed (DMA_ATTR_SKIP_CPU_SYNC below).
	 */
	sg_set_page(sgt->sgl, phys_to_page(dma_to_phys(kb->dev, kb->dma)),
		    kb->size, 0);
	ret = dma_map_sgtable(att->dev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
	if (ret)
		goto err_free;
	return sgt;
err_free:
	sg_free_table(sgt);
err:
	kfree(sgt);
	return ERR_PTR(ret);
}

static void dpu_kbuf_unmap(struct dma_buf_attachment *att, struct sg_table *sgt,
			   enum dma_data_direction dir)
{
	dma_unmap_sgtable(att->dev, sgt, dir, DMA_ATTR_SKIP_CPU_SYNC);
	sg_free_table(sgt);
	kfree(sgt);
}

static void dpu_kbuf_release(struct dma_buf *db)
{
	struct dpu_kbuf *kb = db->priv;

	if (kb->owned)
		dma_free_wc(kb->dev, kb->size, kb->vaddr, kb->dma);
	kfree(kb);
}

static const struct dma_buf_ops dpu_kbuf_ops = {
	.map_dma_buf = dpu_kbuf_map,
	.unmap_dma_buf = dpu_kbuf_unmap,
	.release = dpu_kbuf_release,
};

/* @dma != 0: wrap existing memory (fb0), else allocate from the device CMA */
static struct dma_buf *dpu_kbuf_create(struct dpu_dev *d, size_t size, dma_addr_t dma,
				       void **vaddr)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp);
	struct dpu_kbuf *kb = kzalloc(sizeof(*kb), GFP_KERNEL);
	struct dma_buf *db;

	if (!kb)
		return ERR_PTR(-ENOMEM);
	kb->dev = d->dev;
	kb->size = PAGE_ALIGN(size);
	if (dma) {
		kb->dma = dma;
	} else {
		kb->vaddr = dma_alloc_wc(d->dev, kb->size, &kb->dma, GFP_KERNEL);
		if (!kb->vaddr) {
			kfree(kb);
			return ERR_PTR(-ENOMEM);
		}
		kb->owned = true;
	}
	exp.ops = &dpu_kbuf_ops;
	exp.size = kb->size;
	exp.flags = O_RDWR;
	exp.priv = kb;
	db = dma_buf_export(&exp);
	if (IS_ERR(db)) {
		if (kb->owned)
			dma_free_wc(d->dev, kb->size, kb->vaddr, kb->dma);
		kfree(kb);
		return db;
	}
	if (vaddr)
		*vaddr = kb->vaddr;
	return db;
}

/* synchronous in-kernel commit of one window */
static int dpu_kcommit(struct dpu_dev *d, struct decon_win_config_data *wd,
		       struct dma_buf *db, u32 *latch_us)
{
	struct dma_buf *kbufs[DECON_MAX_WIN] = { };
	struct dma_fence *f = NULL;
	ktime_t t0 = ktime_get();
	long t;
	int ret;

	kbufs[d->boot_win] = db;
	ret = dpu_win_config(d, wd, kbufs, &f);
	if (ret)
		return ret;
	if (!f)
		return -EINVAL;
	t = dma_fence_wait_timeout(f, false, msecs_to_jiffies(500));
	ret = t < 0 ? (int)t : t == 0 ? -ETIMEDOUT : f->error;
	if (latch_us)
		*latch_us = ktime_us_delta(ktime_get(), t0);
	dma_fence_put(f);
	return ret;
}

static int dpu_fb_takeover(struct dpu_dev *d)
{
	struct decon_win_config_data *wd = kzalloc(sizeof(*wd), GFP_KERNEL);
	struct dma_buf *db;
	u32 us = 0, fc0;
	int ret;

	if (!wd)
		return -ENOMEM;
	db = dpu_kbuf_create(d, d->fb_size, d->fb_phys, NULL);
	if (IS_ERR(db)) {
		kfree(wd);
		return PTR_ERR(db);
	}
	dpu_fb_window(d, wd);
	fc0 = dpu_hw_frame_count(d);
	ret = dpu_kcommit(d, wd, db, &us);
	dma_buf_put(db);
	kfree(wd);
	dev_info(d->dev, "fb0 takeover: win%d dpp%d DPP BGRA_8888 at %pa: %d, latched after %u us, FRAME_COUNT %u -> %u, active IDMA base %#x IN_CON %#x (SFR IN_CON %#x)\n",
		 d->boot_win, d->boot_dpp, &d->fb_phys, ret, us, fc0,
		 dpu_hw_frame_count(d),
		 dpu_hw_dma_active_rd(d, d->boot_dpp, IDMA_IN_BASE_ADDR_Y),
		 dpu_hw_dma_active_rd(d, d->boot_dpp, IDMA_IN_CON),
		 dpu_hw_dma_rd(d, d->boot_dpp, IDMA_IN_CON));
	if (!ret)
		d->fb_taken = true;
	return ret;
}

/* ---- self-test: flip fb0's window between two CMA buffers ----------- */

static void st_fill(u32 *p, u32 stride_px, u32 x, u32 y, u32 w, u32 h, u32 c)
{
	u32 i, j;

	for (j = y; j < y + h; j++)
		for (i = x; i < x + w; i++)
			p[j * stride_px + i] = c;
}

static void st_text(u32 *p, u32 stride_px, u32 x, u32 y, u32 s, u32 c, const char *str)
{
	const struct font_desc *f = find_font("TER16x32");
	u32 pitch, gx, gy;

	if (!f)
		f = find_font("VGA8x16");
	if (!f)
		return;
	pitch = DIV_ROUND_UP(f->width, 8);
	for (; *str; str++, x += f->width * s) {
		const u8 *g = (const u8 *)f->data + (u8)*str * pitch * f->height;

		for (gy = 0; gy < f->height; gy++)
			for (gx = 0; gx < f->width; gx++)
				if (g[gy * pitch + gx / 8] & (0x80 >> (gx % 8)))
					st_fill(p, stride_px, x + gx * s, y + gy * s, s, s, c);
	}
}

static void st_draw(struct dpu_dev *d, u32 *p, bool b)
{
	u32 w = d->lcd.xres, h = d->lcd.yres, bg = b ? 0xff400040 : 0xff004000;

	st_fill(p, w, 0, 0, w, h, bg);
	st_fill(p, w, 0, 0, w, 12, 0xffffffff);
	st_fill(p, w, 0, h - 12, w, 12, 0xffffffff);
	st_fill(p, w, 0, 0, 12, h, 0xffffffff);
	st_fill(p, w, w - 12, 0, 12, h, 0xffffffff);
	st_text(p, w, (w - 16 * 24) / 2, 500, 24, b ? 0xffff60ff : 0xff60ff60, b ? "B" : "A");
	st_text(p, w, 100, 1500, 4, 0xffffffff, "DECON FLIP TEST");
	st_text(p, w, 100, 1700, 4, 0xffffffff, b ? "CMA BUFFER B" : "CMA BUFFER A");
	/* the red bar sits at a different height in each buffer */
	st_fill(p, w, 100, b ? 2400 : 2100, w - 200, 120, 0xffff0000);
	wmb();
}

static void dpu_st_work(struct work_struct *work)
{
	struct dpu_dev *d = container_of(to_delayed_work(work), struct dpu_dev, st_work);
	size_t sz = (size_t)d->lcd.xres * d->lcd.yres * 4;
	struct decon_win_config_data *wd;
	struct dma_buf *buf[2] = { };
	void *va[2] = { };
	u32 fc0, te0, fc_prev, us, ok = 0, adv = 0, max_us = 0, i, based = 0;
	bool restored_ok = !d->fb_taken;
	int ret;
	u64 ms = ktime_to_ms(ktime_get_boottime());

	fc0 = fc_prev = dpu_hw_frame_count(d);
	te0 = atomic_read(&d->te_count);
	pr_info(DPU_ST_PFX "start at %llu.%03llu s: FRAME_COUNT=%u TE=%u commits=%u fb_taken=%d\n",
		ms / 1000, ms % 1000, fc0, te0, d->nr_commits, d->fb_taken);

	wd = kzalloc(sizeof(*wd), GFP_KERNEL);
	if (!wd)
		return;
	for (i = 0; i < 2; i++) {
		buf[i] = dpu_kbuf_create(d, sz, 0, &va[i]);
		if (IS_ERR(buf[i])) {
			pr_err(DPU_ST_PFX "FAIL: cannot allocate %zu-byte CMA buffer %c: %ld\n",
			       sz, 'A' + i, PTR_ERR(buf[i]));
			buf[i] = NULL;
			goto out;
		}
		st_draw(d, va[i], i);
		pr_info(DPU_ST_PFX "buffer %c at %pad\n", 'A' + i,
			&((struct dpu_kbuf *)buf[i]->priv)->dma);
	}

	for (i = 0; i < d->st_flips && !READ_ONCE(d->stopping); i++) {
		dma_addr_t want = ((struct dpu_kbuf *)buf[i & 1]->priv)->dma;
		u32 fc, te, act, sfr;

		dpu_fb_window(d, wd);
		ret = dpu_kcommit(d, wd, buf[i & 1], &us);
		fc = dpu_hw_frame_count(d);
		te = atomic_read(&d->te_count);
		/* what the IDMA really scans, not just "a frame went out" */
		act = dpu_hw_dma_active_rd(d, d->boot_dpp, IDMA_IN_BASE_ADDR_Y);
		sfr = dpu_hw_dma_rd(d, d->boot_dpp, IDMA_IN_BASE_ADDR_Y);
		if (!ret)
			ok++;
		if (fc != fc_prev)
			adv++;
		if (act == lower_32_bits(want))
			based++;
		max_us = max(max_us, us);
		pr_info(DPU_ST_PFX "flip %u -> %c: %d, fence after %u us, FRAME_COUNT %u (+%u) TE %u (+%u), IDMA base active %#x SFR %#x want %pad %s\n",
			i, 'A' + (i & 1), ret, us, fc, fc - fc_prev, te, te - te0, act, sfr,
			&want, act == lower_32_bits(want) ? "OK" : "WRONG");
		fc_prev = fc;
		msleep(d->st_period_ms);
	}

out:
	/* back to fb0 so the console is visible again; releases A and B */
	if (d->fb_taken) {
		struct dma_buf *fb = dpu_kbuf_create(d, d->fb_size, d->fb_phys, NULL);
		u32 act;

		if (!IS_ERR(fb)) {
			dpu_fb_window(d, wd);
			ret = dpu_kcommit(d, wd, fb, &us);
			dma_buf_put(fb);
			act = dpu_hw_dma_active_rd(d, d->boot_dpp, IDMA_IN_BASE_ADDR_Y);
			restored_ok = !ret && act == lower_32_bits(d->fb_phys);
			pr_info(DPU_ST_PFX "restored fb0: %d, active IDMA base %#x %s\n", ret, act,
				restored_ok ? "OK" : "WRONG");
		}
	}
	for (i = 0; i < 2; i++)
		if (buf[i])
			dma_buf_put(buf[i]);
	kfree(wd);

	pr_info(DPU_ST_PFX "%s: %u/%u flips latched, active IDMA base right in %u/%u flips, fb0 restore %s, FRAME_COUNT %u -> %u (advanced in %u/%u flips), TE %u -> %u, max fence wait %u us, latch timeouts %u, base mismatches %u\n",
		(d->st_flips && ok == d->st_flips && adv == d->st_flips &&
		 based == d->st_flips && restored_ok &&
		 (d->te_irq < 0 || atomic_read(&d->te_count) != te0)) ? "PASS" : "FAIL",
		ok, d->st_flips, based, d->st_flips, restored_ok ? "OK" : "WRONG",
		fc0, dpu_hw_frame_count(d), adv, d->st_flips, te0,
		atomic_read(&d->te_count), max_us, d->nr_latch_timeouts, d->nr_base_mismatch);
	pr_info(DPU_ST_PFX "a person must confirm the panel alternated green 'A' / magenta 'B' and then showed the console again\n");
}

/* ---- adopt ----------------------------------------------------------- */

static int dpu_adopt(struct dpu_dev *d)
{
	u32 gc, trig, dp0, dp1, ctrl;
	int w, i;

	if (!dpu_hw_powered(d)) {
		dev_err(d->dev, "PMU DPU domain is off: nothing to adopt\n");
		return -ENODEV;
	}
	gc = decon_rd(d, GLOBAL_CONTROL);
	trig = decon_rd(d, HW_SW_TRIG_CONTROL);
	dp0 = decon_rd(d, DATA_PATH_CONTROL_0);
	dp1 = decon_rd(d, DATA_PATH_CONTROL_1);
	dev_info(d->dev, "SBoot state: GLOBAL_CONTROL=%#x TRIG=%#x DP0=%#x DP1=%#x FRAME_COUNT=%u\n",
		 gc, trig, dp0, dp1, dpu_hw_frame_count(d));

	for (i = 0; i < DPU_NR_SYSMMU; i++) {
		if (!d->sysmmu[i])
			continue;
		ctrl = readl(d->sysmmu[i] + DPU_SYSMMU_CTRL);
		dev_info(d->dev, "sysmmu_dpu%d CTRL=%#x VERSION=%#x\n", i, ctrl,
			 readl(d->sysmmu[i] + DPU_SYSMMU_VERSION));
		if (ctrl & 1) {
			dev_err(d->dev, "DPU SysMMU enabled by SBoot: addresses are IOVAs, not adopting\n");
			return -ENODEV;
		}
	}

	if (!(gc & GLOBAL_CONTROL_DECON_EN) || !(gc & GLOBAL_CONTROL_DECON_EN_F) ||
	    !(gc & GLOBAL_CONTROL_OPERATION_MODE_CMD_F)) {
		dev_err(d->dev, "DECON-F not running in command mode\n");
		return -ENODEV;
	}

	d->boot_win = -1;
	for (w = 0; w < DECON_MAX_WIN; w++) {
		int dpp;
		u32 src, img, base, in, off, dimg;

		if (!(dp0 & WIN_EN_F(w)))
			continue;
		dpp = dpu_hw_channel_to_dpp((dp1 >> (4 * w)) & 0x7);
		if (dpp < 0)
			continue;
		/*
		 * IDMA block (0x16071000 + n*0x1000) holds format, sizes and
		 * the base address; the DPP block (0x16021000 + ...) only has
		 * IMG_SIZE at the same offset. Trial q26 had the two swapped
		 * in DT and read IMG right but IN_CON/SRC/BASE = 0. The DPP
		 * IMG_SIZE cross-check refuses adoption if that ever recurs.
		 */
		in = dpu_hw_dma_rd(d, dpp, IDMA_IN_CON);
		src = dpu_hw_dma_rd(d, dpp, IDMA_SRC_SIZE);
		img = dpu_hw_dma_rd(d, dpp, IDMA_IMG_SIZE);
		off = dpu_hw_dma_rd(d, dpp, IDMA_SRC_OFFSET);
		base = dpu_hw_dma_rd(d, dpp, IDMA_IN_BASE_ADDR_Y);
		dimg = readl(d->dpp_regs + dpp * 0x1000 + DPP_IMG_SIZE);
		dev_info(d->dev, "win%d -> dpp%d: IN_CON=%#x SRC=%#x IMG=%#x OFF=%#x BASE=%#x (DPP IMG=%#x; active IN_CON=%#x BASE=%#x)\n",
			 w, dpp, in, src, img, off, base, dimg,
			 dpu_hw_dma_active_rd(d, dpp, IDMA_IN_CON),
			 dpu_hw_dma_active_rd(d, dpp, IDMA_IN_BASE_ADDR_Y));
		if (base == d->fb_phys && (src & 0xffff) * 4 == d->fb_stride &&
		    ((src >> 16) & 0x3fff) >= d->lcd.yres &&
		    (img & 0x1fff) == d->lcd.xres && ((img >> 16) & 0x1fff) == d->lcd.yres &&
		    (dimg & 0x1fff1fff) == (img & 0x1fff1fff) &&
		    ((in >> 11) & 0x1f) <= IDMA_IMG_FORMAT_BGRX8888 &&
		    !(in & (IDMA_ROTATION_MASK | IDMA_AFBC_EN | IDMA_BLOCK_EN)) &&
		    !off && d->boot_win < 0) {
			d->boot_win = w;
			d->boot_dpp = dpp;
			d->boot_fmt = (in >> 11) & 0x1f;
		}
	}
	if (d->boot_win < 0) {
		dev_err(d->dev, "no enabled window scans %pa %ux%u: not adopting\n",
			&d->fb_phys, d->lcd.xres, d->lcd.yres);
		return -ENODEV;
	}
	dev_info(d->dev, "adopted: win%d dpp%d IDMA format %u at %pa\n",
		 d->boot_win, d->boot_dpp, d->boot_fmt, &d->fb_phys);

	/* Stage 1 / 4.9 decon_reg_set_trigger(HW, ENABLE): continuous TE refresh */
	if ((trig & (HW_TRIG_EN | HW_TRIG_MASK_DECON)) != HW_TRIG_EN) {
		d->trig_orig = trig;
		d->trig_written = true;
		dpu_hw_trigger(d, true);
		dev_info(d->dev, "WRITE HW_SW_TRIG_CONTROL %#x -> %#x\n", trig,
			 decon_rd(d, HW_SW_TRIG_CONTROL));
	}
	d->continuous = true;
	return 0;
}

/* ---- panel command window --------------------------------------------- */

#define DPU_CMD_IDLE_US		20000	/* > one 60 Hz frame */
#define DPU_CMD_IDLE_TRIES	5

/*
 * Panel commands (dsim_cmd.c) go out only between frames: DECON adopted
 * and powered, no shadow update pending, TE trigger masked (as dpu_apply()
 * does for every update, 4.9 decon_reg_set_trigger(DISABLE)) and
 * FRAME_COUNT unchanged for longer than one frame, so no frame transfer is
 * in flight on the DSI link. Returns 0 with hw_lock held (no window update
 * can start); dpu_cmd_window_end() restores the trigger bits.
 */
int dpu_cmd_window_begin(struct dpu_dev *d, u32 *trig)
{
	u32 fc0, fc1 = 0, gc;
	int i, ret;

	mutex_lock(&d->hw_lock);
	if (d->boot_win < 0 || !d->fb) {
		ret = -ENODEV;			/* not adopted */
		goto err;
	}
	if (!dpu_hw_powered(d)) {
		ret = -EIO;
		goto err;
	}
	if (dpu_hw_wait_shadow(d, DPU_SHADOW_PRE_US)) {
		ret = -EBUSY;
		goto err;
	}
	*trig = decon_rd(d, HW_SW_TRIG_CONTROL);
	d->te_gap_max_us = 0;
	WRITE_ONCE(d->te_gap_track, true);
	dpu_hw_trigger(d, false);

	fc0 = dpu_hw_frame_count(d);
	for (i = 0; i < DPU_CMD_IDLE_TRIES; i++) {
		usleep_range(DPU_CMD_IDLE_US, DPU_CMD_IDLE_US + 1000);
		fc1 = dpu_hw_frame_count(d);
		if (fc1 == fc0)
			break;
		fc0 = fc1;
	}
	gc = decon_rd(d, GLOBAL_CONTROL);
	pb_dbg("command window: TRIG %#x -> %#x, FRAME_COUNT %u %s, GLOBAL_CONTROL=%#x (idle %d, run %d)\n",
		*trig, decon_rd(d, HW_SW_TRIG_CONTROL), fc1,
		i < DPU_CMD_IDLE_TRIES ? "stable" : "still advancing", gc,
		!!(gc & GLOBAL_CONTROL_IDLE_STATUS), !!(gc & GLOBAL_CONTROL_RUN_STATUS));
	if (i == DPU_CMD_IDLE_TRIES) {
		decon_wr_mask(d, HW_SW_TRIG_CONTROL, *trig, HW_TRIG_EN | HW_TRIG_MASK_DECON);
		WRITE_ONCE(d->te_gap_track, false);
		ret = -EBUSY;
		goto err;
	}
	d->cmd_window = true;
	return 0;
err:
	mutex_unlock(&d->hw_lock);
	return ret;
}

void dpu_cmd_window_end(struct dpu_dev *d, u32 trig)
{
	d->cmd_window = false;
	decon_wr_mask(d, HW_SW_TRIG_CONTROL, trig, HW_TRIG_EN | HW_TRIG_MASK_DECON);
	WRITE_ONCE(d->te_gap_track, false);
	pb_dbg("command window closed: TRIG=%#x FRAME_COUNT=%u, longest TE irq interval %u us\n",
	       decon_rd(d, HW_SW_TRIG_CONTROL), dpu_hw_frame_count(d), d->te_gap_max_us);
	mutex_unlock(&d->hw_lock);
}

/* ---- probe ----------------------------------------------------------- */

static int dpu_parse_dt(struct dpu_dev *d)
{
	struct device_node *np = d->dev->of_node, *fb;
	struct dpu_lcd *l = &d->lcd;
	struct resource res;
	int ret;

	fb = of_parse_phandle(np, "samsung,boot-fb", 0);
	if (!fb)
		return dev_err_probe(d->dev, -EINVAL, "no samsung,boot-fb\n");
	ret = of_address_to_resource(fb, 0, &res);
	if (!ret && (of_property_read_u32(fb, "width", &l->xres) ||
		     of_property_read_u32(fb, "height", &l->yres) ||
		     of_property_read_u32(fb, "stride", &d->fb_stride)))
		ret = -EINVAL;
	of_node_put(fb);
	if (ret)
		return dev_err_probe(d->dev, ret, "bad samsung,boot-fb\n");
	d->fb_phys = res.start;
	d->fb_size = (size_t)d->fb_stride * l->yres;
	if (resource_size(&res) < d->fb_size || d->fb_stride != l->xres * 4)
		return dev_err_probe(d->dev, -EINVAL, "boot-fb geometry\n");

	l->fps = 60;
	of_property_read_u32(np, "samsung,lcd-fps", &l->fps);
	of_property_read_u32(np, "samsung,lcd-width-mm", &l->mm_w);
	of_property_read_u32(np, "samsung,lcd-height-mm", &l->mm_h);
	l->dsc_en = of_property_read_bool(np, "samsung,dsc-enabled");
	of_property_read_u32(np, "samsung,dsc-slice-width", &l->dsc_w);
	of_property_read_u32(np, "samsung,dsc-slice-height", &l->dsc_h);
	return 0;
}

static int dpu_register_fb(struct dpu_dev *d)
{
	struct fb_info *info;
	void __iomem *base;
	int ret;

	/* simplefb (stage 1) owns the aperture until now */
	ret = aperture_remove_conflicting_devices(d->fb_phys, d->fb_size, "exynos9810-decon");
	if (ret)
		return ret;
	base = devm_ioremap_wc(d->dev, d->fb_phys, d->fb_size);
	if (!base)
		return -ENOMEM;

	info = framebuffer_alloc(0, d->dev);
	if (!info)
		return -ENOMEM;
	info->par = d;
	info->fbops = &dpu_fb_ops;
	info->screen_base = (char __iomem *)base;
	info->screen_size = d->fb_size;
	info->pseudo_palette = d->pseudo_palette;

	strscpy(info->fix.id, "decon_f", sizeof(info->fix.id));
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.accel = FB_ACCEL_NONE;
	info->fix.smem_start = d->fb_phys;
	info->fix.smem_len = d->fb_size;
	info->fix.line_length = d->fb_stride;

	info->var.xres = info->var.xres_virtual = d->lcd.xres;
	info->var.yres = info->var.yres_virtual = d->lcd.yres;
	info->var.bits_per_pixel = 32;
	info->var.red = (struct fb_bitfield){ 16, 8, 0 };
	info->var.green = (struct fb_bitfield){ 8, 8, 0 };
	info->var.blue = (struct fb_bitfield){ 0, 8, 0 };
	info->var.transp = (struct fb_bitfield){ 24, 8, 0 };
	info->var.width = d->lcd.mm_w;
	info->var.height = d->lcd.mm_h;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.vmode = FB_VMODE_NONINTERLACED;

	ret = register_framebuffer(info);
	if (ret) {
		framebuffer_release(info);
		return ret;
	}
	d->fb = info;
	dev_info(d->dev, "fb%d: %ux%u stride %u at %pa\n", info->node, d->lcd.xres,
		 d->lcd.yres, d->fb_stride, &d->fb_phys);
	return 0;
}

static int dpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dpu_dev *d;
	const char *trig = "none";
	int i, ret;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->dev = dev;
	d->te_irq = -1;
	mutex_init(&d->lock);
	mutex_init(&d->hw_lock);
	spin_lock_init(&d->queue_lock);
	spin_lock_init(&d->fence_lock);
	INIT_LIST_HEAD(&d->queue);
	INIT_LIST_HEAD(&d->stale);
	init_waitqueue_head(&d->vsync_wait);
	INIT_DELAYED_WORK(&d->st_work, dpu_st_work);
	INIT_DELAYED_WORK(&d->pb_work, dpu_panel_bright_work);
	d->fence_ctx = dma_fence_context_alloc(1);
	platform_set_drvdata(pdev, d);

	ret = dpu_parse_dt(d);
	if (ret)
		return ret;

	of_property_read_string(np, "samsung,decon-trigger", &trig);
	if (strcmp(trig, "hw-te"))
		return dev_err_probe(dev, -EPERM,
				     "adopt needs samsung,decon-trigger = \"hw-te\"\n");

	d->pmu = syscon_regmap_lookup_by_phandle(np, "samsung,pmu-syscon");
	if (IS_ERR(d->pmu))
		return dev_err_probe(dev, PTR_ERR(d->pmu), "pmu syscon\n");
	d->regs = devm_platform_ioremap_resource_byname(pdev, "decon");
	if (IS_ERR(d->regs))
		return PTR_ERR(d->regs);
	d->dma_regs = devm_platform_ioremap_resource_byname(pdev, "dpp-dma");
	if (IS_ERR(d->dma_regs))
		return PTR_ERR(d->dma_regs);
	d->dpp_regs = devm_platform_ioremap_resource_byname(pdev, "dpp");
	if (IS_ERR(d->dpp_regs))
		return PTR_ERR(d->dpp_regs);
	d->dma_com = devm_platform_ioremap_resource_byname(pdev, "dma-common");
	if (IS_ERR(d->dma_com))
		return PTR_ERR(d->dma_com);
	for (i = 0; i < DPU_NR_SYSMMU; i++) {
		char name[8];
		struct resource *r;

		snprintf(name, sizeof(name), "sysmmu%d", i);
		r = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
		if (!r)
			continue;
		d->sysmmu[i] = devm_ioremap_resource(dev, r);
		if (IS_ERR(d->sysmmu[i]))
			return PTR_ERR(d->sysmmu[i]);
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	if (of_property_present(np, "memory-region")) {
		ret = of_reserved_mem_device_init(dev);
		if (ret)
			return dev_err_probe(dev, ret, "memory-region\n");
	}

	/* look the TE irq up before any side effect, so a deferral is clean */
	ret = platform_get_irq_byname_optional(pdev, "te");
	if (ret == -EPROBE_DEFER)
		goto err_mem;
	d->te_irq = ret > 0 ? ret : -1;

	ret = dpu_adopt(d);
	if (ret)
		goto err_mem;

	d->worker = kthread_create_worker(0, "decon0_update");
	if (IS_ERR(d->worker)) {
		ret = PTR_ERR(d->worker);
		goto err_mem;
	}
	sched_set_fifo(d->worker->task);
	kthread_init_work(&d->work, dpu_update_work);

	ret = dpu_register_fb(d);
	if (ret)
		goto err_worker;

	if (of_property_read_bool(np, "samsung,fb-takeover"))
		dpu_fb_takeover(d);

	if (d->te_irq > 0) {
		ret = devm_request_irq(dev, d->te_irq, dpu_te_irq, 0, "decon0-te", d);
		if (ret) {
			dev_warn(dev, "TE irq: %d, vsync falls back to 60 Hz sleeps\n", ret);
			d->te_irq = -1;
		}
	} else {
		dev_warn(dev, "no TE irq, vsync falls back to 60 Hz sleeps\n");
	}
	d->vsync_thread = kthread_run(dpu_vsync_thread, d, "decon0_vsync");
	if (IS_ERR(d->vsync_thread)) {
		ret = PTR_ERR(d->vsync_thread);
		d->vsync_thread = NULL;
		goto err_fb;
	}

	if (of_property_read_bool(np, "samsung,decon-selftest")) {
		u64 now = ktime_to_ms(ktime_get_boottime());

		d->st_at_ms = 108000;
		d->st_flips = 20;
		d->st_period_ms = 400;
		of_property_read_u32(np, "samsung,decon-selftest-at-ms", &d->st_at_ms);
		of_property_read_u32(np, "samsung,decon-selftest-flips", &d->st_flips);
		d->st_flips = min(d->st_flips, 100U);
		pr_info(DPU_ST_PFX "scheduled at %u ms uptime, %u flips every %u ms\n",
			d->st_at_ms, d->st_flips, d->st_period_ms);
		queue_delayed_work(system_long_wq, &d->st_work,
				   msecs_to_jiffies(d->st_at_ms > now ? d->st_at_ms - now : 0));
	}

	/* opt-in: the 4.9 fault-display brightness, sent once (panel_bright.c) */
	ret = dpu_panel_bright_init(d);
	if (!ret) {
		u64 now = ktime_to_ms(ktime_get_boottime());

		d->pb_enabled = true;
		pr_info(DPU_PB_PFX "scheduled at %u ms uptime: brightness %u (4.9 PANEL_FAULT_DISPLAY_BRIGHTNESS), id check %s, gamma check %s\n",
			d->pb_at_ms, d->pb_target, d->pb_has_id ? "on" : "off",
			d->pb_has_gamma ? "on" : "off");
		queue_delayed_work(system_long_wq, &d->pb_work,
				   msecs_to_jiffies(d->pb_at_ms > now ? d->pb_at_ms - now : 0));
	} else if (ret != -ENODEV) {
		dev_warn(dev, "panel-bright disabled: %d\n", ret);
	}
	dev_info(dev, "adopt mode ready: TE irq %d, %s\n", d->te_irq,
		 d->fb_taken ? "fb0 window owned" : "fb0 window still SBoot's");
	return 0;

err_fb:
	unregister_framebuffer(d->fb);
	framebuffer_release(d->fb);
err_worker:
	kthread_destroy_worker(d->worker);
	dpu_commit_free(d->active);
	d->active = NULL;
err_mem:
	of_reserved_mem_device_release(dev);
	return ret;
}

static void dpu_stop(struct dpu_dev *d)
{
	WRITE_ONCE(d->stopping, true);
	cancel_delayed_work_sync(&d->pb_work);
	cancel_delayed_work_sync(&d->st_work);
	kthread_flush_worker(d->worker);
}

static void dpu_shutdown(struct platform_device *pdev)
{
	struct dpu_dev *d = platform_get_drvdata(pdev);

	dpu_stop(d);
}

static const struct of_device_id dpu_of_match[] = {
	{ .compatible = "samsung,exynos9810-decon" },
	{ }
};
MODULE_DEVICE_TABLE(of, dpu_of_match);

static struct platform_driver dpu_driver = {
	.probe = dpu_probe,
	.shutdown = dpu_shutdown,
	.driver = {
		.name = "exynos9810-decon",
		.of_match_table = dpu_of_match,
		.dev_groups = dpu_groups,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(dpu_driver);

MODULE_DESCRIPTION("Exynos9810 DECON-F fbdev driver (adopt mode)");
MODULE_LICENSE("GPL");
