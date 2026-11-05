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
 * physically contiguous dma-bufs, including AFBC on the VGF channels. By
 * default a scattered buffer (gralloc's dma-buf system heap) is copied into
 * a contiguous decon-pool buffer per frame.
 *
 * Opt-in by DT ("iommus" on decon_f, see Documentation/crownlte/
 * display-iommu.md): the three DPU SysMMUs translate for DECON, dma-bufs
 * are mapped to one IOVA run each and the mapping is cached per dma-buf
 * (dpu_iova_get()), so scattered buffers are scanned in place, without a
 * copy. SBoot's framebuffer is identity-mapped (reserved-memory
 * "iommu-addresses", IOMMU_RESV_DIRECT) before translation starts, so the
 * splash keeps reading the same memory. Without "iommus" nothing changes.
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
#include <linux/cma.h>
#include <linux/console.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/file.h>
#include <linux/font.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/kthread.h>
#include <linux/memblock.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sync_file.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>

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

/* ---- DPU SysMMU: per-dma-buf IOVA cache -------------------------------- */

/*
 * With the DPU SysMMU a dma-buf is mapped once and its IOVA is reused for
 * every frame that scans it, as 4.9 did through ion_iovmm_map(). HWC cycles
 * a few buffers per layer; mapping and unmapping the ~4200 pages of a
 * full-screen buffer every frame (page-table writes, a TLB flush per unmap)
 * would cost more than the copy it replaces.
 *
 * An entry holds its own dma-buf reference, attachment and mapping; commits
 * that scan or will scan it count as users. An entry no commit uses is
 * dropped once the cache holds the last reference to its dma-buf (every
 * other user closed it), or when it falls off the LRU beyond
 * DPU_IOVA_CACHE_MAX. A commit stays referenced until a later update has
 * latched (d->active, d->stale, dpu_process()), so an IOVA is never unmapped
 * while a window may still fetch it.
 *
 * CPU writers keep the dma-buf contract (DMA_BUF_IOCTL_SYNC): the exporter
 * syncs every mapped attachment, this one included, at end_cpu_access.
 * In-kernel buffers (fb0, self-tests) are not cached.
 */
#define DPU_IOVA_CACHE_MAX	32

static void dpu_iova_free_list(struct list_head *dead)
{
	struct dpu_iova *e, *t;

	list_for_each_entry_safe(e, t, dead, node) {
		list_del(&e->node);
		dma_buf_unmap_attachment_unlocked(e->att, e->sgt, DMA_BIDIRECTIONAL);
		dma_buf_detach(e->dmabuf, e->att);
		dma_buf_put(e->dmabuf);
		kfree(e);
	}
}

/* iova_lock held: unlink unused entries that are orphaned or beyond the cap */
static void dpu_iova_prune(struct dpu_dev *d, struct list_head *dead)
{
	struct dpu_iova *e, *t;
	unsigned int pos = 0;

	list_for_each_entry_safe(e, t, &d->iova_lru, node) {
		pos++;
		if (e->users)
			continue;
		if (pos <= DPU_IOVA_CACHE_MAX && file_count(e->dmabuf->file) > 1)
			continue;
		list_move(&e->node, dead);
		d->iova_nr--;
		d->iova_unmaps++;
		pos--;
	}
}

/* one IOVA run below 4 GiB covering @need bytes */
static int dpu_iova_usable(const struct dpu_iova *e, u64 need)
{
	return e->len >= need && e->addr + need <= SZ_4G ? 0 : -ERANGE;
}

/*
 * Map @db for DECON, or reuse its cached mapping; on success @b scans it
 * through the SysMMU (b->iova). On failure nothing is held and the caller
 * falls back to the per-commit import (and the bounce copy).
 */
static int dpu_iova_get(struct dpu_dev *d, struct dma_buf *db, struct dpu_buf *b,
			u64 need)
{
	struct scatterlist *sg;
	struct dpu_iova *e;
	LIST_HEAD(dead);
	bool hit = false;
	u64 len = 0;
	int i, ret = 0;

	mutex_lock(&d->iova_lock);
	list_for_each_entry(e, &d->iova_lru, node) {
		if (e->dmabuf == db) {
			hit = true;
			break;
		}
	}
	if (hit) {
		ret = dpu_iova_usable(e, need);
		if (ret)
			goto out;
		list_move(&e->node, &d->iova_lru);
		d->iova_hits++;
		goto found;
	}

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e) {
		ret = -ENOMEM;
		goto out;
	}
	e->d = d;
	e->att = dma_buf_attach(db, d->dev);
	if (IS_ERR(e->att)) {
		ret = PTR_ERR(e->att);
		kfree(e);
		goto out;
	}
	e->sgt = dma_buf_map_attachment_unlocked(e->att, DMA_BIDIRECTIONAL);
	if (IS_ERR(e->sgt)) {
		ret = PTR_ERR(e->sgt);
		dma_buf_detach(db, e->att);
		kfree(e);
		goto out;
	}
	e->addr = sg_dma_address(e->sgt->sgl);
	for_each_sgtable_dma_sg(e->sgt, sg, i) {
		if (sg_dma_address(sg) != e->addr + len)
			ret = -EINVAL;
		len += sg_dma_len(sg);
	}
	e->len = len;
	e->nsegs = e->sgt->orig_nents;
	if (!ret)
		ret = dpu_iova_usable(e, need);
	if (ret) {
		dev_warn_ratelimited(d->dev, DPU_IOMMU_PFX "%zu-byte dma-buf: %u DMA segments, %llu bytes at %pad, not one usable IOVA run (%d)\n",
				     db->size, e->sgt->nents, len, &e->addr, ret);
		dma_buf_unmap_attachment_unlocked(e->att, e->sgt, DMA_BIDIRECTIONAL);
		dma_buf_detach(db, e->att);
		kfree(e);
		goto out;
	}
	get_dma_buf(db);
	e->dmabuf = db;
	list_add(&e->node, &d->iova_lru);
	d->iova_nr++;
	if (++d->iova_maps <= 16 || !(d->iova_maps % 256))
		dev_info(d->dev, DPU_IOMMU_PFX "map #%u: %zu-byte dma-buf, %u physical chunks -> IOVA %pad (cached %u, hits %u, unmapped %u)\n",
			 d->iova_maps, db->size, e->nsegs, &e->addr, d->iova_nr,
			 d->iova_hits, d->iova_unmaps);
found:
	e->users++;
	b->iova = e;
	b->addr = e->addr;
	b->len = e->len;
	b->nsegs = e->nsegs;
out:
	dpu_iova_prune(d, &dead);
	mutex_unlock(&d->iova_lock);
	dpu_iova_free_list(&dead);
	return ret;
}

static void dpu_iova_put(struct dpu_iova *e)
{
	struct dpu_dev *d = e->d;
	LIST_HEAD(dead);

	mutex_lock(&d->iova_lock);
	if (!WARN_ON(!e->users))
		e->users--;
	dpu_iova_prune(d, &dead);
	mutex_unlock(&d->iova_lock);
	dpu_iova_free_list(&dead);
}

static void dpu_buf_release(struct dpu_buf *b)
{
	struct dpu_iova *e = b->iova;

	if (b->sgt)
		dma_buf_unmap_attachment_unlocked(b->att, b->sgt, DMA_BIDIRECTIONAL);
	if (b->att)
		dma_buf_detach(b->dmabuf, b->att);
	if (b->dmabuf)
		dma_buf_put(b->dmabuf);
	memset(b, 0, sizeof(*b));
	/* after the commit's own reference, so an orphaned mapping goes now */
	if (e)
		dpu_iova_put(e);
}

/*
 * Import one plane. Without a SysMMU the DPP takes a physical address, so
 * the buffer must be one contiguous run below 4 GiB. Gralloc allocates from
 * the dma-buf system heap (libion_exynos dma-heap mode), which is scattered;
 * with @may_bounce such a buffer is accepted and copied into a contiguous
 * decon-pool buffer before it is scanned (dpu_bounce()). Kernel buffers
 * (fb0, self-test) are always contiguous.
 *
 * The device has a 64-bit streaming DMA mask (dpu_probe()), so this mapping
 * is plain dma-direct: dma address = physical address, no swiotlb. With the
 * old 32-bit mask every system-heap page above 4 GiB was sent to swiotlb,
 * whose 1 MiB segments overflow it ("swiotlb buffer is full", ROM-in-RAM v10)
 * and the map failed. The 4 GiB limit of the IDMA base is checked here.
 * The mapping of a bounced buffer is kept: dpu_bounce() reads its pages.
 */
static int dpu_buf_import(struct dpu_dev *d, struct dma_buf *db, struct dpu_buf *b,
			  u64 need, bool may_bounce, const char **why)
{
	struct scatterlist *sg;
	dma_addr_t addr = 0;
	u64 len = 0;
	bool contig = true;
	int i, ret;

	b->dmabuf = db;
	if (db->size < need) {
		*why = "dma-buf smaller than the frame";
		return -EINVAL;
	}
	/* DPU SysMMU: scan the buffer in place through a cached IOVA */
	if (d->iommu && may_bounce) {
		ret = dpu_iova_get(d, db, b, need);
		if (!ret)
			return 0;
		dev_warn_ratelimited(d->dev, DPU_IOMMU_PFX "no IOVA for a %zu-byte dma-buf (%d): per-frame import and bounce copy\n",
				     db->size, ret);
	}
	b->att = dma_buf_attach(db, d->dev);
	if (IS_ERR(b->att)) {
		ret = PTR_ERR(b->att);
		b->att = NULL;
		*why = "dma_buf_attach failed";
		return ret;
	}
	b->sgt = dma_buf_map_attachment_unlocked(b->att, DMA_BIDIRECTIONAL);
	if (IS_ERR(b->sgt)) {
		ret = PTR_ERR(b->sgt);
		b->sgt = NULL;
		if (!may_bounce) {
			*why = "dma_buf_map_attachment failed";
			return ret;
		}
		contig = false;		/* dpu_bounce() falls back to dma_buf_vmap() */
	} else {
		addr = sg_dma_address(b->sgt->sgl);
		for_each_sgtable_dma_sg(b->sgt, sg, i) {
			if (sg_dma_address(sg) != addr + len)
				contig = false;
			len += sg_dma_len(sg);
		}
		b->nsegs = b->sgt->nents;
		if (contig && (len < need || addr + need > SZ_4G))
			contig = false;
	}
	if (contig) {
		b->addr = addr;
		b->len = len;
		return 0;
	}
	if (!may_bounce) {
		*why = "buffer not contiguous below 4 GiB (needs the DPU SysMMU)";
		return -EINVAL;
	}
	b->bounce = true;
	b->len = db->size;
	return 0;
}

/*
 * Copy a mapped buffer through the cacheable kernel linear map, segment by
 * segment, after invalidating the CPU caches for it
 * (dma_sync_sgtable_for_cpu(): DC IVAC on this non-coherent device). The
 * system heap's own vmap of an uncached buffer is write-combined, and
 * memcpy() reading from it ran at ~100 MB/s (ROM-in-RAM v10: 150-195 ms for
 * the 17.3 MB AFBC client target). The CPU never writes these pages, so the
 * invalidate cannot drop data; the GPU finished before the acquire fence.
 * Returns -ENOENT if a segment has no struct page (caller uses the vmap).
 */
static int dpu_bounce_copy_sg(struct dpu_dev *d, struct dpu_buf *b, void *dst, u32 *inval_us)
{
	struct scatterlist *sg;
	size_t off = 0;
	ktime_t t0;
	int i;

	for_each_sgtable_sg(b->sgt, sg, i)
		if (!sg_page(sg))
			return -ENOENT;
	t0 = ktime_get();
	dma_sync_sgtable_for_cpu(d->dev, b->sgt, DMA_BIDIRECTIONAL);
	*inval_us = ktime_us_delta(ktime_get(), t0);
	for_each_sgtable_sg(b->sgt, sg, i) {
		size_t n = min_t(size_t, sg->length, b->len - off);

		if (!n)
			break;
		memcpy(dst + off, page_address(sg_page(sg)) + sg->offset, n);
		off += n;
	}
	return off == b->len ? 0 : -EINVAL;
}

/* fallback: the exporter's CPU access path (slow for uncached heaps) */
static int dpu_bounce_copy_vmap(struct dpu_buf *b, void *dst)
{
	struct iosys_map map;
	int ret;

	ret = dma_buf_begin_cpu_access(b->dmabuf, DMA_FROM_DEVICE);
	if (ret)
		return ret;
	ret = dma_buf_vmap_unlocked(b->dmabuf, &map);
	if (!ret && map.is_iomem) {
		dma_buf_vunmap_unlocked(b->dmabuf, &map);
		ret = -EINVAL;
	}
	if (!ret) {
		memcpy(dst, map.vaddr, b->len);
		dma_buf_vunmap_unlocked(b->dmabuf, &map);
	}
	dma_buf_end_cpu_access(b->dmabuf, DMA_FROM_DEVICE);
	return ret;
}

/*
 * Copy the scattered buffers of a commit into the decon-pool double buffer
 * (stopgap until the DPU SysMMU, branch crownlte-dpu-iommu, is merged). An
 * AFBC stream is position independent relative to its base, so a byte copy
 * scans out identically. Runs in the update worker after the acquire fences,
 * outside hw_lock. Every window has its own pair of slots, sized to its
 * buffer (ROM-in-RAM v12: the status bar, a second scattered layer, was
 * rejected and with it every frame after boot); the slot not used by the
 * window in the last latched commit is written, so the one being scanned is
 * never touched. Budget: decon-pool is 48 MiB, the full-screen AFBC client
 * target takes 2 x 17.3 MB, bars a few hundred KiB each; a window whose slot
 * cannot be allocated fails the frame with a log line.
 */
static bool dpu_commit_uses_slot(const struct dpu_commit *c, int w, int s)
{
	return c && (c->bounced & BIT(w)) && c->slot[w] == s;
}

/*
 * Free every bounce slot that no commit references: not the frame being
 * scanned (d->active), not frames retained after a latch timeout (d->stale,
 * already emptied by dpu_process() once the pending update latched; checked
 * anyway), not the slots @cur filled so far. Runs in the update worker,
 * which alone changes those lists, after dpu_process() resolved any pending
 * update.
 * Returns the number of bytes freed. Sol review S4: slots used to be kept
 * forever, so the client target moving to another window could not get its
 * 2 x 17.3 MB from the 48 MiB pool.
 */
static size_t dpu_bounce_reclaim(struct dpu_dev *d, const struct dpu_commit *cur)
{
	const struct dpu_commit *p;
	size_t freed = 0;
	int w, s;

	for (w = 0; w < DECON_MAX_WIN; w++) {
		for (s = 0; s < 2; s++) {
			struct dpu_bounce *t = &d->bounce[w][s];
			bool used = dpu_commit_uses_slot(cur, w, s) ||
				    dpu_commit_uses_slot(d->active, w, s);

			list_for_each_entry(p, &d->stale, node)
				used |= dpu_commit_uses_slot(p, w, s);
			if (!t->vaddr || used)
				continue;
			dma_free_wc(d->dev, t->size, t->vaddr, t->dma);
			freed += t->size;
			t->vaddr = NULL;
			t->size = 0;
		}
	}
	if (freed)
		dev_info(d->dev, "bounce: reclaimed %zu bytes of unreferenced slots\n", freed);
	return freed;
}

static int dpu_bounce(struct dpu_dev *d, struct dpu_commit *c)
{
	int i, ret;

	for (i = 0; i < DECON_MAX_WIN; i++) {
		struct dpu_buf *b = &c->buf[i];
		struct dpu_bounce *t = &d->bounce[i][d->bounce_next[i]];
		ktime_t t0 = ktime_get();
		u32 us, inval_us = 0;
		const char *path = "linear map";

		if (c->cfg[i].state != DECON_WIN_STATE_BUFFER || !b->bounce)
			continue;
		if (t->size < b->len) {
			if (t->vaddr)
				dma_free_wc(d->dev, t->size, t->vaddr, t->dma);
			t->vaddr = NULL;
			t->size = PAGE_ALIGN(b->len);
			t->vaddr = dma_alloc_wc(d->dev, t->size, &t->dma, GFP_KERNEL);
			if (!t->vaddr && dpu_bounce_reclaim(d, c))
				t->vaddr = dma_alloc_wc(d->dev, t->size, &t->dma, GFP_KERNEL);
			if (!t->vaddr || upper_32_bits(t->dma + t->size - 1)) {
				dev_err_ratelimited(d->dev, "win%d bounce buffer %d: cannot allocate %zu bytes below 4 GiB from decon-pool\n",
						    i, d->bounce_next[i], t->size);
				if (t->vaddr)
					dma_free_wc(d->dev, t->size, t->vaddr, t->dma);
				t->vaddr = NULL;
				t->size = 0;
				return -ENOMEM;
			}
			dev_info(d->dev, "win%d bounce buffer %d: %zu bytes at %pad\n",
				 i, d->bounce_next[i], t->size, &t->dma);
		}
		ret = b->sgt ? dpu_bounce_copy_sg(d, b, t->vaddr, &inval_us) : -ENOENT;
		if (ret == -ENOENT) {
			path = "dma_buf_vmap";
			ret = dpu_bounce_copy_vmap(b, t->vaddr);
		}
		if (ret) {
			dev_err_ratelimited(d->dev, "win%d: bounce copy (%s) failed: %d\n",
					    i, path, ret);
			return ret;
		}
		wmb();
		b->addr = t->dma;
		c->slot[i] = d->bounce_next[i];
		c->bounced |= BIT(i);	/* dpu_process() flips bounce_next[i] once latched */
		us = ktime_us_delta(ktime_get(), t0);
		d->bounce_max_us = max(d->bounce_max_us, us);
		if (!d->nr_bounces++ || us > 40000)
			dev_info_ratelimited(d->dev, "win%d: scattered buffer (%u segments, %zu bytes%s) copied to %pad via %s in %u us (invalidate %u us; bounce #%u, max %u us)\n",
					     i, b->nsegs, b->len, c->cfg[i].compression ? ", AFBC" : "",
					     &b->addr, path, us, inval_us, d->nr_bounces,
					     d->bounce_max_us);
	}
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

/* NULL if the buffer window can be scanned, else the reason it cannot */
static const char *dpu_check_buffer_win(struct dpu_dev *d, const struct decon_win_config *w,
					u32 dpp_fmt)
{
	const struct decon_frame *s = &w->src, *t = &w->dst;
	int bpp = dpu_fmt_bpp(dpp_fmt);

	if (w->idma_type >= DPU_NR_DPP)
		return "idma_type out of range";
	if (!bpp)
		return "format not supported (RGB 32-bit and RGB565 only)";
	if (s->w < 16 || s->h < 16 || s->w > 4096 || s->h > 4096 ||
	    s->f_w < 16 || s->f_h < 16 || s->f_w > 65534 || s->f_h > 8190 ||
	    s->x < 0 || s->y < 0 || s->x + s->w > s->f_w || s->y + s->h > s->f_h)
		return "source size/crop out of range";
	if (t->x < 0 || t->y < 0 || t->x + t->w > d->lcd.xres ||
	    t->y + t->h > d->lcd.yres)
		return "destination outside the panel";
	if (t->w != s->w || t->h != s->h)
		return "scaling not supported";
	if (w->dpp_parm.rot)
		return "rotation not supported";
	if (w->protection)
		return "protected content not supported";
	if (w->blending > DECON_BLENDING_COVERAGE)
		return "unknown blending mode";
	if (w->block_area.w || w->block_area.h)
		return "block area not supported";
	if (w->compression) {
		/* 4.9 dpp_check_format() / dpp_check_limitation() */
		if (w->idma_type != DECON_IDMA_VGF0 && w->idma_type != DECON_IDMA_VGF1)
			return "AFBC on a non-VGF channel";
		if (w->dpp_parm.hdr_std)
			return "AFBC with HDR";
	}
	if (bpp == 16 && (s->x & 1))
		return "odd x offset for RGB565";
	return NULL;
}

static struct dpu_commit *dpu_build_commit(struct dpu_dev *d,
					   struct decon_win_config_data *wd,
					   struct dma_buf **kbufs, int *err)
{
	struct dpu_commit *c = kzalloc(sizeof(*c), GFP_KERNEL);
	unsigned long used = 0;
	int i, ret = 0;
	const char *why = NULL;

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
		why = NULL;
		switch (w->state) {
		case DECON_WIN_STATE_DISABLED:
			break;
		case DECON_WIN_STATE_COLOR:
			if (w->dst.x < 0 || w->dst.y < 0 || !w->dst.w || !w->dst.h ||
			    w->dst.x + w->dst.w > d->lcd.xres ||
			    w->dst.y + w->dst.h > d->lcd.yres) {
				why = "color window outside the panel";
				ret = -EINVAL;
			} else {
				c->nr_win++;
			}
			break;
		case DECON_WIN_STATE_BUFFER:
		case DECON_WIN_STATE_CURSOR:
			c->cfg[i].state = DECON_WIN_STATE_BUFFER;
			c->dpp_fmt[i] = dpu_translate_fmt_to_dpp(w->format);
			why = dpu_check_buffer_win(d, w, c->dpp_fmt[i]);
			if (why) {
				ret = -EINVAL;
				break;
			}
			if (test_and_set_bit(w->idma_type, &used)) {
				why = "channel used by two windows";	/* 4.9 chmap check */
				ret = -EINVAL;
				break;
			}
			if (kbufs) {
				db = kbufs[i];
				if (!db) {
					why = "no kernel buffer";
					ret = -EINVAL;
					break;
				}
				get_dma_buf(db);
			} else {
				db = dma_buf_get(w->fd_idma[0]);
				if (IS_ERR(db)) {
					why = "fd_idma[0] is not a dma-buf";
					ret = PTR_ERR(db);
					break;
				}
			}
			/* AFBC: header + compressed payload, never smaller than this */
			need = (u64)w->src.f_w * w->src.f_h * dpu_fmt_bpp(c->dpp_fmt[i]) / 8;
			ret = dpu_buf_import(d, db, &c->buf[i], need, !kbufs, &why);
			if (ret)
				break;
			if (w->compression)
				d->nr_afbc++;
			if (!kbufs && w->acq_fence >= 0) {
				c->acq[i] = sync_file_get_fence(w->acq_fence);
				if (!c->acq[i]) {
					why = "acq_fence is not a sync_file";
					ret = -EINVAL;
				}
			}
			c->nr_win++;
			break;
		default:
			why = "unknown window state";
			ret = -EINVAL;
			break;
		}
		if (ret)
			dev_warn_ratelimited(d->dev, "win%d rejected: %s (%d) [state %u fmt %u idma %u comp %u rot %u prot %u blend %u pa %d hdr %u fd %d/%d/%d acq %d block %ux%u src %ux%u@%d,%d f %ux%u dst %ux%u@%d,%d]\n",
					     i, why ? why : "?", ret, w->state, w->format,
					     w->idma_type, w->compression, w->dpp_parm.rot,
					     w->protection, w->blending, w->plane_alpha,
					     w->dpp_parm.hdr_std, w->fd_idma[0], w->fd_idma[1],
					     w->fd_idma[2], w->acq_fence, w->block_area.w,
					     w->block_area.h, w->src.w, w->src.h, w->src.x,
					     w->src.y, w->src.f_w, w->src.f_h, w->dst.w,
					     w->dst.h, w->dst.x, w->dst.y);
	}
	if (ret) {
		dpu_commit_free(c);
		*err = ret;
		return NULL;
	}
	return c;
}

/* ---- update worker -------------------------------------------------- */

#define DPU_FAULT_TRIES		3	/* controlled unmasks per held update */

/*
 * hw_lock held. 0 once no shadow update is pending, else -EBUSY.
 *
 * SysMMU only: a fault masked the TE trigger while an update waited for it,
 * so that update cannot latch and every later commit or panel command
 * window would stop here. Such an update gets at most DPU_FAULT_TRIES
 * controlled unmasks (commits and command windows together), each followed
 * by a wait of up to DPU_LATCH_US. A fault after the latch masks the
 * trigger again; the next commit replaces the frame. If the update still
 * does not latch, DECON is quiesced: the trigger stays masked (logged once)
 * until a commit with a different buffer set replaces the held update
 * (dpu_commit_fresh()). Nothing is freed here: the held frame is
 * d->active and the frame before it stays on d->stale until a latch.
 */
static int dpu_pending_resolve(struct dpu_dev *d, const char *who)
{
	int ret = dpu_hw_wait_shadow(d, DPU_SHADOW_PRE_US);

	if (!ret) {
		if (d->fault_quiesced)
			dev_info(d->dev, DPU_IOMMU_PFX "%s: no update pending any more, quiesce over\n",
				 who);
		d->fault_quiesced = false;
		return 0;
	}
	if (d->fault_quiesced || !dpu_hw_fault_inhibited(d))
		return -EBUSY;
	if (d->fault_try_seq != d->update_seq) {
		d->fault_try_seq = d->update_seq;
		d->fault_tries = 0;
	}
	if (d->fault_tries >= DPU_FAULT_TRIES) {
		d->fault_quiesced = true;
		d->fault_quiesces++;
		dev_err(d->dev, DPU_IOMMU_PFX "%s: update #%u still held after %u unmasks: quiesced, TE trigger masked until a commit with other buffers\n",
			who, d->update_seq, DPU_FAULT_TRIES);
		return -EBUSY;
	}
	if (!dpu_hw_trigger_fault_unmask(d))
		return -EBUSY;
	d->fault_tries++;
	ret = dpu_hw_wait_shadow(d, DPU_LATCH_US);
	dev_warn(d->dev, DPU_IOMMU_PFX "%s: update #%u held by a SysMMU fault mask: unmask %u/%u, %s\n",
		 who, d->update_seq, d->fault_tries, DPU_FAULT_TRIES,
		 ret ? "still pending" : "latched");
	return ret ? -EBUSY : 0;
}

/*
 * Quiesced (hw_lock held): @c may replace the held update if it scans a
 * different buffer set and needs no bounce copy (both slots of a bounced
 * window may still be referenced while an update is held).
 */
static bool dpu_commit_fresh(struct dpu_dev *d, const struct dpu_commit *c)
{
	const struct dpu_commit *h = d->active;
	bool same = !!h;
	int i;

	for (i = 0; i < DECON_MAX_WIN; i++) {
		bool on = c->cfg[i].state == DECON_WIN_STATE_BUFFER;

		if (on && c->buf[i].bounce)
			return false;
		if (!h)
			continue;
		if (on != (h->cfg[i].state == DECON_WIN_STATE_BUFFER) ||
		    (on && c->buf[i].addr != h->buf[i].addr))
			same = false;
	}
	return !same;
}

/*
 * @fresh: quiesced, @c replaces a held update (dpu_commit_fresh()): the
 * trigger is masked, so no frame starts and the shadow registers can be
 * rewritten although the old request is still pending.
 */
static int dpu_apply(struct dpu_dev *d, struct dpu_commit *c, bool fresh)
{
	ktime_t t0;
	int i, ret;

	mutex_lock(&d->hw_lock);
	if (!dpu_hw_powered(d)) {
		ret = -EIO;
		goto out;
	}
	/* 4.9: the previous shadow update must have latched */
	if (!fresh && dpu_hw_wait_shadow(d, DPU_SHADOW_PRE_US)) {
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
	d->update_seq++;
	dpu_hw_trigger_commit(d);	/* also ends a SysMMU fault inhibit */
	t0 = ktime_get();
	ret = dpu_hw_wait_shadow(d, DPU_LATCH_US);
	c->latch_us = ktime_us_delta(ktime_get(), t0);
	c->fc_after = dpu_hw_frame_count(d);
	if (ret) {
		d->nr_latch_timeouts++;
		dev_warn_ratelimited(d->dev, "shadow update did not latch in %u us (FRAME_COUNT %u -> %u)\n",
				     DPU_LATCH_US, c->fc_before, c->fc_after);
	} else {
		d->fault_quiesced = false;
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

/*
 * Pixel layout of the 32-bit DPP-level formats. The IDMA format name lists
 * the components of a little-endian 32-bit word from bit 31 down, as DRM
 * fourccs do: ARGB8888 = 0xAARRGGBB = memory bytes B,G,R,A; XBGR8888 =
 * memory bytes R,G,B,X. Evidence:
 *  - 4.9 Android is correct on this panel, and HWC's RGBX_8888 client
 *    target (Android: memory bytes R,G,B,X) reaches IDMA as XBGR8888
 *    (4.9 dpu_translate_fmt_to_dpp());
 *  - trial q48 (camera): with the opposite table every pattern came out
 *    exactly as this layout predicts, on G0, G1 and VGF0 alike (format 0
 *    red -> cyan, fbcon black -> blue; XBGR green -> magenta, magenta ->
 *    yellow), so no DPP differs.
 * SBoot's buffer (IDMA format 0) therefore holds 0xAARRGGBB words.
 */
/* memory byte index of R, G, B, A (X) */
static const u8 dpu_fmt_pos[8][4] = {
	[DECON_PIXEL_FORMAT_ARGB_8888] = { 2, 1, 0, 3 },
	[DECON_PIXEL_FORMAT_ABGR_8888] = { 0, 1, 2, 3 },
	[DECON_PIXEL_FORMAT_RGBA_8888] = { 3, 2, 1, 0 },
	[DECON_PIXEL_FORMAT_BGRA_8888] = { 1, 2, 3, 0 },
	[DECON_PIXEL_FORMAT_XRGB_8888] = { 2, 1, 0, 3 },
	[DECON_PIXEL_FORMAT_XBGR_8888] = { 0, 1, 2, 3 },
	[DECON_PIXEL_FORMAT_RGBX_8888] = { 3, 2, 1, 0 },
	[DECON_PIXEL_FORMAT_BGRX_8888] = { 1, 2, 3, 0 },
};

/* ---- content / register diagnostics ----------------------------------- */

/* latched copy of a DECON SFR (4.9 decon_dump(): regs + SHADOW_OFFSET) */
static u32 decon_rd_shd(struct dpu_dev *d, u32 off)
{
	return readl(d->regs + SHADOW_OFFSET + off);
}

/*
 * What the DPU is fed: CRC32 of the first 64 KiB of the bounce buffer and,
 * for AFBC, the 16-byte superblock headers (one per 16x16 block, at the
 * start of the buffer). AFBC 1.x: the first 32 bits are the payload offset;
 * offset 0 marks a solid-colour superblock whose colour sits in bytes
 * 8..15 (all-zero header = solid 0, black/transparent). Reads the
 * write-combined bounce buffer, so only on sampled frames.
 */
static void dpu_diag_content(struct dpu_dev *d, struct dpu_commit *c, int win)
{
	const struct decon_win_config *w = &c->cfg[win];
	const u8 *p = d->bounce[win][d->bounce_next[win]].vaddr;
	size_t len = c->buf[win].len, n64 = min_t(size_t, len, SZ_64K), i;
	u32 crc = crc32_le(~0U, p, n64);

	if (w->compression) {
		u32 nsb = DIV_ROUND_UP(w->src.f_w, 16) * DIV_ROUND_UP(w->src.f_h, 16);
		u32 zero = 0, solid = 0, payload = 0, beyond = 0;
		u64 sum[4] = { };
		u8 h[16], h0[16] = { }, hm[16] = { };

		nsb = min_t(size_t, nsb, len / 16);
		for (i = 0; i < nsb; i++) {
			u32 ofs;

			memcpy(h, p + i * 16, 16);
			if (!i)
				memcpy(h0, h, 16);
			if (i == nsb / 2)
				memcpy(hm, h, 16);
			ofs = get_unaligned_le32(h);
			if (!memchr_inv(h, 0, 16)) {
				zero++;
			} else if (!ofs) {
				solid++;
				sum[0] += h[8];
				sum[1] += h[9];
				sum[2] += h[10];
				sum[3] += h[11];
			} else {
				payload++;
				if (ofs >= len)
					beyond++;
			}
		}
		dev_info(d->dev, "diag bounce #%u win%d: AFBC %ux%u, %u headers: %u all-zero, %u solid (mean bytes8-11 %llu %llu %llu %llu), %u payload (%u offsets beyond the buffer); hdr[0] %16ph hdr[mid] %16ph; crc32(first %zu) %08x\n",
			 d->nr_bounces, win, w->src.f_w, w->src.f_h, nsb, zero, solid,
			 solid ? sum[0] / solid : 0, solid ? sum[1] / solid : 0,
			 solid ? sum[2] / solid : 0, solid ? sum[3] / solid : 0,
			 payload, beyond, h0, hm, n64, crc);
	} else {
		size_t nz = 0;

		for (i = 0; i < n64; i++)
			nz += !!p[i];
		dev_info(d->dev, "diag bounce #%u win%d: linear, first %zu bytes: %zu non-zero, first 16 %16ph, crc32 %08x\n",
			 d->nr_bounces, win, n64, nz, p, crc);
	}
}

/* window and channel state as programmed (SFR) and as latched (active) */
static void dpu_diag_regs(struct dpu_dev *d, struct dpu_commit *c, int win, const char *tag)
{
	const struct decon_win_config *w = &c->cfg[win];
	int dpp = w->idma_type < DPU_NR_DPP ? w->idma_type : 0;
	u32 in = dpu_hw_dma_rd(d, dpp, IDMA_IN_CON);
	u32 ain = dpu_hw_dma_active_rd(d, dpp, IDMA_IN_CON);

	dev_info(d->dev, "diag %s win%d: state %u pa %d blend %u comp %u fmt %u->dpp %u | DECON SFR/active: DP0 %#x/%#x DP1 %#x/%#x WIN_CTL0 %#x/%#x WIN_CTL1 %#x/%#x START %#x/%#x END %#x/%#x CMAP %#x,%#x/%#x,%#x BG_SIZE %#x/%#x BG_COL %#x,%#x/%#x,%#x\n",
		 tag, win, w->state, w->plane_alpha, w->blending, w->compression, w->format,
		 c->dpp_fmt[win],
		 decon_rd(d, DATA_PATH_CONTROL_0), decon_rd_shd(d, DATA_PATH_CONTROL_0),
		 decon_rd(d, DATA_PATH_CONTROL_1), decon_rd_shd(d, DATA_PATH_CONTROL_1),
		 decon_rd(d, WIN_CONTROL_0(win)), decon_rd_shd(d, WIN_CONTROL_0(win)),
		 decon_rd(d, WIN_CONTROL_1(win)), decon_rd_shd(d, WIN_CONTROL_1(win)),
		 decon_rd(d, WIN_START_POSITION(win)), decon_rd_shd(d, WIN_START_POSITION(win)),
		 decon_rd(d, WIN_END_POSITION(win)), decon_rd_shd(d, WIN_END_POSITION(win)),
		 decon_rd(d, WIN_COLORMAP_0(win)), decon_rd(d, WIN_COLORMAP_1(win)),
		 decon_rd_shd(d, WIN_COLORMAP_0(win)), decon_rd_shd(d, WIN_COLORMAP_1(win)),
		 decon_rd(d, BLENDER_BG_IMAGE_SIZE_0), decon_rd_shd(d, BLENDER_BG_IMAGE_SIZE_0),
		 decon_rd(d, BLENDER_BG_IMAGE_COLOR_0), decon_rd(d, BLENDER_BG_IMAGE_COLOR_1),
		 decon_rd_shd(d, BLENDER_BG_IMAGE_COLOR_0),
		 decon_rd_shd(d, BLENDER_BG_IMAGE_COLOR_1));
	if (w->state != DECON_WIN_STATE_BUFFER)
		return;
	dev_info(d->dev, "diag %s win%d: IDMA%d SFR/active IN_CON %#x/%#x (afbc %d/%d, fmt %u/%u) BASE %#x/%#x (want %pad) SRC %#x/%#x OFFSET %#x/%#x IMG %#x/%#x RECOVERY %#x IRQ %#x | DPP IN_CON %#x IMG %#x\n",
		 tag, win, dpp, in, ain, !!(in & IDMA_AFBC_EN), !!(ain & IDMA_AFBC_EN),
		 (in >> 11) & 0x1f, (ain >> 11) & 0x1f,
		 dpu_hw_dma_rd(d, dpp, IDMA_IN_BASE_ADDR_Y),
		 dpu_hw_dma_active_rd(d, dpp, IDMA_IN_BASE_ADDR_Y), &c->buf[win].addr,
		 dpu_hw_dma_rd(d, dpp, IDMA_SRC_SIZE), dpu_hw_dma_active_rd(d, dpp, IDMA_SRC_SIZE),
		 dpu_hw_dma_rd(d, dpp, IDMA_SRC_OFFSET),
		 dpu_hw_dma_active_rd(d, dpp, IDMA_SRC_OFFSET),
		 dpu_hw_dma_rd(d, dpp, IDMA_IMG_SIZE), dpu_hw_dma_active_rd(d, dpp, IDMA_IMG_SIZE),
		 dpu_hw_dma_rd(d, dpp, IDMA_RECOVERY_CTRL), dpu_hw_dma_rd(d, dpp, IDMA_IRQ),
		 readl(d->dpp_regs + dpp * 0x1000 + DPP_IN_CON),
		 readl(d->dpp_regs + dpp * 0x1000 + DPP_IMG_SIZE));
}

/*
 * Is DECON still sending frames? FRAME_COUNT advances per frame sent, TE
 * per panel tearing-effect pulse; with the trigger unmasked both move at
 * 60 Hz. The pending bits are never cleared by this driver (no DECON
 * interrupt handler), so a set bit means "at least once since SBoot".
 */
static void dpu_diag_hdr(struct dpu_dev *d, const char *tag)
{
	dev_info(d->dev, "diag %s: FRAME_COUNT %u TE %d TRIG %#x GLOBAL_CONTROL %#x INT_PEND %#x EXTRA_INT_PEND %#x | commits %u latch timeouts %u rejects %u bounces %u\n",
		 tag, dpu_hw_frame_count(d), atomic_read(&d->te_count),
		 decon_rd(d, HW_SW_TRIG_CONTROL), decon_rd(d, GLOBAL_CONTROL),
		 decon_rd(d, INTERRUPT_PENDING), decon_rd(d, EXTRA_INTERRUPT_PENDING),
		 d->nr_commits, d->nr_latch_timeouts, d->nr_rejects, d->nr_bounces);
}

#define DPU_TH_COLS	8
#define DPU_TH_ROWS	16
#define DPU_TH_SAMPLES	16	/* per cell and axis */

/* an RGB32 picture to sample: a CPU copy in RAM or fb0 (ioremap_wc) */
struct dpu_thumb_src {
	const u8 *mem;
	const u8 __iomem *io;
	size_t len;
	u32 stride;			/* bytes */
	u32 x, y, w, h;			/* crop, pixels */
	u32 fmt;			/* DPP-level RGB32 format */
	bool alpha;			/* mark transparent cells */
};

/*
 * Sol review S3: thumbnail text is built where it is sampled (in the
 * update worker before the commit fence signals) and printed afterwards
 * by dpu_diag_emit() on system_wq, so the dump delays neither the fence
 * nor the next commit.
 */
#define DPU_TXT_SIZE	2048

struct dpu_diag_txt {
	struct list_head node;
	size_t len;
	char buf[DPU_TXT_SIZE];
};

static __printf(2, 3) void dpu_txt_add(struct dpu_diag_txt *t, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	t->len += vscnprintf(t->buf + t->len, sizeof(t->buf) - t->len, fmt, ap);
	va_end(ap);
}

static struct dpu_diag_txt *dpu_txt_new(struct list_head *list)
{
	/* Sol review S4: never reclaim in the commit worker; no memory, no dump */
	struct dpu_diag_txt *t = kmalloc(sizeof(*t), GFP_NOWAIT | __GFP_NOWARN);

	if (!t)
		return NULL;
	t->len = 0;
	t->buf[0] = '\0';
	list_add_tail(&t->node, list);
	return t;
}

static void dpu_diag_emit(struct work_struct *work)
{
	struct dpu_dev *d = container_of(work, struct dpu_dev, diag_emit_work);
	struct dpu_diag_txt *t;
	char *p, *line;

	for (;;) {
		spin_lock(&d->diag_lock);
		t = list_first_entry_or_null(&d->diag_txt, struct dpu_diag_txt, node);
		if (t)
			list_del(&t->node);
		spin_unlock(&d->diag_lock);
		if (!t)
			break;
		p = t->buf;
		while ((line = strsep(&p, "\n")))
			if (*line)
				dev_info(d->dev, "%s\n", line);
		kfree(t);
	}
}

/*
 * Print the texts on @list from system_wq; @list is left empty. Once
 * dpu_stop() began (Sol review S5) the texts are dropped instead, so
 * nothing is queued behind its final flush.
 */
static void dpu_diag_post(struct dpu_dev *d, struct list_head *list)
{
	struct dpu_diag_txt *t, *n;

	if (READ_ONCE(d->stopping)) {
		list_for_each_entry_safe(t, n, list, node) {
			list_del(&t->node);
			kfree(t);
		}
		return;
	}
	if (list_empty(list))
		return;
	spin_lock(&d->diag_lock);
	list_splice_tail_init(list, &d->diag_txt);
	spin_unlock(&d->diag_lock);
	schedule_work(&d->diag_emit_work);
}

#define DPU_RATE_MS	250

/*
 * Sol review S1: FRAME_COUNT and TE counted between two callbacks
 * DPU_RATE_MS apart (the second on system_wq), never by sleeping in the
 * update worker. One measurement at a time.
 */
static void dpu_rate_start(struct dpu_dev *d, const char *what)
{
	if (READ_ONCE(d->stopping))
		return;		/* Sol review S5: not after dpu_stop() began */
	if (test_and_set_bit_lock(0, &d->rate_busy)) {
		dev_info(d->dev, "diag %s: frame rate not measured (another measurement running)\n",
			 what);
		return;
	}
	strscpy(d->rate_tag, what, sizeof(d->rate_tag));
	d->rate_t0 = ktime_get();
	d->rate_fc0 = dpu_hw_frame_count(d);
	d->rate_te0 = atomic_read(&d->te_count);
	schedule_delayed_work(&d->rate_work, msecs_to_jiffies(DPU_RATE_MS));
}

static void dpu_rate_work(struct work_struct *work)
{
	struct dpu_dev *d = container_of(work, struct dpu_dev, rate_work.work);
	u32 fc1 = dpu_hw_frame_count(d);
	int te1 = atomic_read(&d->te_count);

	dev_info(d->dev, "diag %s: in %lld ms FRAME_COUNT %u -> %u (+%u), TE %d -> %d (+%d)\n",
		 d->rate_tag, ktime_ms_delta(ktime_get(), d->rate_t0), d->rate_fc0, fc1,
		 fc1 - d->rate_fc0, d->rate_te0, te1, te1 - d->rate_te0);
	clear_bit_unlock(0, &d->rate_busy);
}

static u32 dpu_luma_digit(u64 l1000)
{
	return min_t(u32, 9, l1000 * 10 / 256000);
}

/* sample @i of DPU_TH_SAMPLES, evenly spread over [@a, @b) */
static u32 dpu_th_pos(u32 a, u32 b, u32 i)
{
	return a + (b - a) * (2 * i + 1) / (2 * DPU_TH_SAMPLES);
}

/*
 * A coarse picture, appended to @txt: the crop split into an 8 x 16 grid,
 * one line per grid row, one character per cell: mean luma (0.299 R +
 * 0.587 G + 0.114 B) as 0-9, '.' where the mean alpha is below 16
 * (transparent, @t->alpha only), '?' outside the buffer; after '|' the
 * brightest sample of each cell on the same scale, so small bright
 * details (text, a logo) show up too. 16 x 16 samples per cell (every
 * ~11 px on 1440x2960): ~32k uncached reads.
 */
static void dpu_thumb(const struct dpu_thumb_src *t, const char *what,
		      struct dpu_diag_txt *txt)
{
	const u8 *pos = dpu_fmt_pos[t->fmt];
	char mean[DPU_TH_COLS + 1], peak[DPU_TH_COLS + 1];
	u32 cx, cy, sx, sy;

	for (cy = 0; cy < DPU_TH_ROWS; cy++) {
		u32 y0 = t->h * cy / DPU_TH_ROWS, y1 = t->h * (cy + 1) / DPU_TH_ROWS;

		for (cx = 0; cx < DPU_TH_COLS; cx++) {
			u32 x0 = t->w * cx / DPU_TH_COLS, x1 = t->w * (cx + 1) / DPU_TH_COLS;
			u64 lsum = 0, asum = 0;
			u32 n = 0, lmax = 0;

			for (sy = 0; sy < DPU_TH_SAMPLES; sy++) {
				u32 y = dpu_th_pos(y0, y1, sy);

				for (sx = 0; sx < DPU_TH_SAMPLES; sx++) {
					u32 x = dpu_th_pos(x0, x1, sx);
					size_t off = (size_t)(t->y + y) * t->stride +
						     (size_t)(t->x + x) * 4;
					u32 px, l;
					const u8 *b = (const u8 *)&px;

					if (off + 4 > t->len)
						continue;
					px = t->io ? readl_relaxed(t->io + off) :
						     READ_ONCE(*(const u32 *)(t->mem + off));
					l = 299 * b[pos[0]] + 587 * b[pos[1]] + 114 * b[pos[2]];
					lsum += l;
					lmax = max(lmax, l);
					asum += b[pos[3]];
					n++;
				}
			}
			if (!n) {
				mean[cx] = peak[cx] = '?';
			} else if (t->alpha && div_u64(asum, n) < 16) {
				mean[cx] = peak[cx] = '.';
			} else {
				mean[cx] = '0' + dpu_luma_digit(div_u64(lsum, n));
				peak[cx] = '0' + dpu_luma_digit(lmax);
			}
		}
		mean[DPU_TH_COLS] = peak[DPU_TH_COLS] = '\0';
		dpu_txt_add(txt, "diag %s thumb %02u: %s | %s\n", what, cy, mean, peak);
	}
}

/*
 * Window geometry and a thumbnail of the window's source crop, read from
 * the write-combined CPU copy the DPU scans (@vaddr), as a text on @list.
 */
static void dpu_diag_thumb(struct dpu_dev *d, const struct dpu_commit *c, int win,
			   const void *vaddr, const char *tag, struct list_head *list)
{
	const struct decon_win_config *w = &c->cfg[win];
	const struct decon_frame *s = &w->src, *t = &w->dst;
	struct dpu_diag_txt *txt = dpu_txt_new(list);
	u32 fmt = c->dpp_fmt[win];
	struct dpu_thumb_src ts;
	char what[40];

	if (!txt)
		return;
	if (w->state != DECON_WIN_STATE_BUFFER) {
		dpu_txt_add(txt, "diag %s win%d geometry: colour %#x dst %ux%u@%d,%d blend %u plane alpha %d\n",
			    tag, win, w->color, t->w, t->h, t->x, t->y, w->blending,
			    w->plane_alpha);
		return;
	}
	dpu_txt_add(txt, "diag %s win%d geometry: dst %ux%u@%d,%d src %ux%u@%d,%d of %ux%u fmt %u->dpp %u blend %u plane alpha %d idma %u -> ch %u%s\n",
		    tag, win, t->w, t->h, t->x, t->y, s->w, s->h, s->x, s->y, s->f_w, s->f_h,
		    w->format, fmt, w->blending, w->plane_alpha, w->idma_type,
		    dpu_hw_dpp_to_channel(w->idma_type < DPU_NR_DPP ? w->idma_type : 0),
		    w->compression ? " AFBC" : "");
	if (!vaddr || w->compression || !DPU_FMT_IS_RGB32(fmt) ||
	    s->w < DPU_TH_COLS || s->h < DPU_TH_ROWS) {
		dpu_txt_add(txt, "diag %s win%d thumbnail: n/a (%s)\n", tag, win,
			    !vaddr ? "no CPU copy" : w->compression ? "AFBC" :
			    !DPU_FMT_IS_RGB32(fmt) ? "not RGB32" : "too small");
		return;
	}
	ts = (struct dpu_thumb_src){
		.mem = vaddr, .len = c->buf[win].len, .stride = s->f_w * 4,
		.x = s->x, .y = s->y, .w = s->w, .h = s->h, .fmt = fmt,
		/*
		 * Sol review S2: pixel alpha only hides a pixel when the
		 * window blends with it; with DECON_BLENDING_NONE (and for
		 * the X formats) the RGB is shown whatever the alpha byte.
		 */
		.alpha = fmt <= DECON_PIXEL_FORMAT_BGRA_8888 &&
			 (w->blending == DECON_BLENDING_PREMULT ||
			  w->blending == DECON_BLENDING_COVERAGE),
	};
	snprintf(what, sizeof(what), "%s win%d", tag, win);
	dpu_thumb(&ts, what, txt);
}

/* window @i of @c scans this CPU copy, NULL if the DPU reads the dma-buf */
static const void *dpu_diag_copy(struct dpu_dev *d, const struct dpu_commit *c, int i)
{
	return (c->bounced & BIT(i)) ? d->bounce[i][c->slot[i]].vaddr : NULL;
}

#define DPU_THUMB_PERIOD_MS	5000

/*
 * Tell "HWC frames are dark" from "the DPU shows black": registers at the
 * first latched user commit, content + registers for the first 3 bounces
 * and every 100th after, each with the frame/TE counters. Window geometry
 * and thumbnails go to @list (printed after the fence signals), for the
 * first latch and then at most once per window every DPU_THUMB_PERIOD_MS.
 */
static void dpu_diag(struct dpu_dev *d, struct dpu_commit *c, struct list_head *list)
{
	bool first = !d->diag_latched;
	bool sample = c->bounced && (d->nr_bounces <= 3 || !(d->nr_bounces % 100));
	const char *tag = first ? "first HWC latch" : "sample";
	ktime_t now = ktime_get();
	int i;

	if (!first && !sample)
		return;
	d->diag_latched = true;
	dpu_diag_hdr(d, tag);
	for (i = 0; i < DECON_MAX_WIN; i++) {
		if (c->cfg[i].state == DECON_WIN_STATE_DISABLED)
			continue;
		dpu_diag_regs(d, c, i, tag);
		if (sample && c->buf[i].bounce)
			dpu_diag_content(d, c, i);
		if (!first && d->thumb_at[i] &&
		    ktime_ms_delta(now, d->thumb_at[i]) < DPU_THUMB_PERIOD_MS)
			continue;
		d->thumb_at[i] = now;
		dpu_diag_thumb(d, c, i, dpu_diag_copy(d, c, i), tag, list);
	}
}

#define DPU_IDLE_SNAP_MS	3000
#define DPU_IDLE_SNAP_MAX	32
#define DPU_FB_SNAP_MS		5000

/*
 * HWC stops committing while the screen is static, so the sampled frames
 * above can miss the one left on the panel. Once no user commit has come
 * for DPU_IDLE_SNAP_MS, log the frame being scanned (d->active): frame and
 * TE rate over DPU_RATE_MS, registers, geometry and thumbnails. Runs in
 * the update worker, so d->active and its bounce slots are stable while
 * they are read; nothing in here sleeps.
 */
static void dpu_idle_snap(struct kthread_work *work)
{
	struct dpu_dev *d = container_of(work, struct dpu_dev, idle_work.work);
	struct dpu_commit *c = d->active;
	LIST_HEAD(txt);
	char tag[16];
	int i;

	if (!c || c->internal || READ_ONCE(d->stopping) ||
	    d->nr_idle_snaps >= DPU_IDLE_SNAP_MAX)
		return;
	snprintf(tag, sizeof(tag), "idle #%u", ++d->nr_idle_snaps);
	dev_info(d->dev, "diag %s: no commit for %u ms\n", tag, DPU_IDLE_SNAP_MS);
	dpu_rate_start(d, tag);
	dpu_diag_hdr(d, tag);
	for (i = 0; i < DECON_MAX_WIN; i++) {
		if (c->cfg[i].state == DECON_WIN_STATE_DISABLED)
			continue;
		dpu_diag_regs(d, c, i, tag);
		dpu_diag_thumb(d, c, i, dpu_diag_copy(d, c, i), tag, &txt);
	}
	dpu_diag_post(d, &txt);
}

/*
 * fb0 (the buffer SBoot left at 0xcc000000, then our fb0): is it black
 * before HWC starts, and does DECON keep sending it? @rate also starts a
 * frame/TE count over DPU_RATE_MS. The thumbnail goes to @list. fb0 is
 * scanned with blending off, so its alpha byte does not matter: cells
 * show luma.
 */
static void dpu_diag_fb(struct dpu_dev *d, const char *when, bool rate,
			struct list_head *list)
{
	struct dpu_thumb_src ts = {
		.io = d->fb ? (const u8 __iomem *)d->fb->screen_base : NULL,
		.len = d->fb_size, .stride = d->fb_stride,
		.w = d->lcd.xres, .h = d->lcd.yres, .fmt = d->fb_dpp_fmt,
	};
	struct dpu_diag_txt *txt;
	char what[40];

	snprintf(what, sizeof(what), "fb0 %s", when);
	if (rate)
		dpu_rate_start(d, what);
	dpu_diag_hdr(d, what);
	if (!ts.io || !DPU_FMT_IS_RGB32(ts.fmt) || ts.stride < ts.w * 4 ||
	    (size_t)ts.stride * ts.h > ts.len) {
		dev_info(d->dev, "diag %s thumbnail: n/a (fb0 %s)\n", what,
			 !ts.io ? "not registered" : "layout unexpected");
		return;
	}
	txt = dpu_txt_new(list);
	if (!txt)
		return;
	dpu_txt_add(txt, "diag %s: %pa %ux%u stride %u DPP format %u, window %s, HWC %s; win%d IDMA base %#x active %#x\n",
		    what, &d->fb_phys, ts.w, ts.h, ts.stride, ts.fmt,
		    d->fb_taken ? "taken over" : "SBoot's",
		    d->diag_latched ? "started" : "not started",
		    d->boot_win, dpu_hw_dma_rd(d, d->boot_dpp, IDMA_IN_BASE_ADDR_Y),
		    dpu_hw_dma_active_rd(d, d->boot_dpp, IDMA_IN_BASE_ADDR_Y));
	dpu_thumb(&ts, what, txt);
}

static void dpu_fb_snap_work(struct work_struct *work)
{
	struct dpu_dev *d = container_of(work, struct dpu_dev, fb_snap_work.work);
	LIST_HEAD(txt);

	if (READ_ONCE(d->stopping))
		return;
	dpu_diag_fb(d, "at 5 s", true, &txt);
	dpu_diag_post(d, &txt);
}

/*
 * The fb0 buffer must be reserved no-map (SBoot's picture, never handed
 * to the page allocator) and no CMA area may overlap it: CMA pages are
 * lent to movable allocations, which would overwrite the picture. Logs
 * every /reserved-memory node as placed, the default and the device CMA
 * area, and whether the linear map covers fb0.
 */
static void dpu_check_fb_reservation(struct dpu_dev *d)
{
	struct device_node *rm = of_find_node_by_path("/reserved-memory"), *np;
	phys_addr_t fs = d->fb_phys, fe = d->fb_phys + d->fb_size;
	const char *verdict = "NOT covered by one non-reusable node";
	bool covered = false, covered_nomap = false, overlap = false;
	struct cma *areas[2] = { NULL, NULL };
	int i;

	for_each_available_child_of_node(rm, np) {
		struct reserved_mem *r = of_reserved_mem_lookup(np);
		bool nomap = of_property_read_bool(np, "no-map");
		bool reusable = of_property_read_bool(np, "reusable");
		bool hit;
		phys_addr_t b, e;
		struct resource res;

		if (r && r->size) {
			b = r->base;
			e = r->base + r->size;
		} else if (!of_address_to_resource(np, 0, &res)) {
			b = res.start;
			e = res.end + 1;
		} else {
			dev_info(d->dev, "reserved-memory: %pOFn not placed\n", np);
			continue;
		}
		hit = b < fe && fs < e;
		dev_info(d->dev, "reserved-memory: %pa..%pa %s%s %pOFn%s\n", &b, &e,
			 nomap ? "no-map" : "map", reusable ? " reusable" : "", np,
			 hit ? " <- overlaps fb0" : "");
		if (hit && reusable)
			overlap = true;
		if (hit && !reusable && b <= fs && e >= fe) {
			covered = true;
			covered_nomap |= nomap;
		}
	}
	of_node_put(rm);
	if (covered_nomap)
		verdict = "reserved no-map";
	else if (covered)
		verdict = "reserved (mapped, not reusable)";

#ifdef CONFIG_DMA_CMA
	areas[0] = dev_get_cma_area(NULL);
	areas[1] = d->dev->cma_area;
#endif
	for (i = 0; i < ARRAY_SIZE(areas); i++) {
		phys_addr_t b, e;

		if (!areas[i])
			continue;
		b = cma_get_base(areas[i]);
		e = b + cma_get_size(areas[i]);
		dev_info(d->dev, "%s CMA area %s: %pa..%pa%s\n", i ? "decon" : "default",
			 cma_get_name(areas[i]), &b, &e,
			 b < fe && fs < e ? " <- overlaps fb0" : "");
		if (b < fe && fs < e)
			overlap = true;
	}
	dev_info(d->dev, "fb0 %pa..%pa: %s, %s, linear map %s\n", &fs, &fe, verdict,
		 overlap ? "OVERLAPPED BY A CMA/REUSABLE AREA" : "no CMA overlap",
		 memblock_is_map_memory(fs) ? "covers it" : "excludes it");
}

static void dpu_process(struct dpu_dev *d, struct dpu_commit *c)
{
	struct dpu_commit *old = NULL;
	LIST_HEAD(txt);
	bool fresh;
	int i, ret;

	/*
	 * fb0 as the panel showed it until now, before HWC replaces it. The
	 * commit-path diagnostics stop with d->stopping (Sol review S5).
	 */
	if (!c->internal && !d->fb_snap_hwc && !READ_ONCE(d->stopping)) {
		d->fb_snap_hwc = true;
		dpu_diag_fb(d, "before the first HWC commit", true, &txt);
	}

	for (i = 0; i < DECON_MAX_WIN; i++) {
		if (!c->acq[i])
			continue;
		if (dma_fence_wait_timeout(c->acq[i], false,
					   msecs_to_jiffies(DPU_ACQ_FENCE_MS)) <= 0)
			dev_warn_ratelimited(d->dev, "win%d acquire fence timeout\n", i);
		dma_fence_put(c->acq[i]);
		c->acq[i] = NULL;
	}

	/*
	 * A bounce copy may only go into a slot nobody scans. After a latch
	 * timeout (-ETIMEDOUT below) bounce_next already points at the slot of
	 * the frame before the pending one, which DECON may still read. So
	 * resolve a pending shadow update before copying or (re)allocating
	 * any slot, and drop this frame while one is still pending (nothing
	 * copied, nothing programmed).
	 */
	mutex_lock(&d->hw_lock);
	/* SysMMU: also the bounded recovery of an update a fault mask holds */
	ret = dpu_pending_resolve(d, "commit");
	fresh = ret && d->fault_quiesced && dpu_commit_fresh(d, c);
	if (fresh) {
		d->fault_fresh++;
		dev_warn(d->dev, DPU_IOMMU_PFX "quiesced: a commit with other buffers replaces held update #%u\n",
			 d->update_seq);
		ret = 0;
	}
	mutex_unlock(&d->hw_lock);
	if (ret) {
		dev_warn_ratelimited(d->dev, "previous shadow update still pending, frame skipped before any copy\n");
		c->result = -EBUSY;
	} else {
		struct dpu_commit *p, *n;

		/*
		 * No update pending: the last requested frame (d->active), even
		 * one that timed out, has latched, so frames kept on d->stale
		 * after a latch timeout are no longer scanned. Release them and
		 * their bounce slot references before any copy, allocation or
		 * reclaim (Sol review S4: otherwise two 17.3 MB slots could stay
		 * pinned until another commit succeeded).
		 */
		/* (not when replacing a held update: nothing has latched yet) */
		if (!fresh) {
			list_for_each_entry_safe(p, n, &d->stale, node) {
				list_del(&p->node);
				dpu_commit_free(p);
			}
		}
		c->result = dpu_bounce(d, c);
		if (!c->result)
			c->result = dpu_apply(d, c, fresh);
	}
	d->nr_commits++;
	if (!c->result && !c->internal && !READ_ONCE(d->stopping))
		dpu_diag(d, c, &txt);	/* before bounce_next moves on */
	/* the bounce buffer just written is now (or may be) scanned */
	if (c->result == 0 || c->result == -ETIMEDOUT)
		for_each_set_bit(i, &c->bounced, DECON_MAX_WIN)
			d->bounce_next[i] ^= 1;
	/* (re)start the idle snapshot of the frame HWC leaves up */
	if ((c->result == 0 || c->result == -ETIMEDOUT) && !c->internal &&
	    d->nr_idle_snaps < DPU_IDLE_SNAP_MAX && !READ_ONCE(d->stopping))
		kthread_mod_delayed_work(d->worker, &d->idle_work,
					 msecs_to_jiffies(DPU_IDLE_SNAP_MS));

	if (c->fence) {
		if (c->result)
			dma_fence_set_error(c->fence, c->result);
		dma_fence_signal(c->fence);
	}
	dpu_diag_post(d, &txt);		/* thumbnails, printed from system_wq */

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

static int dpu_fb_ioctl_cmd(struct fb_info *info, unsigned int cmd, unsigned long arg)
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
		dev_info(d->dev, "dump: FRAME_COUNT=%u TRIG=%#x DP0=%#x DP1=%#x commits=%u timeouts=%u rejects=%u te=%d base_mismatch=%u afbc=%u bounces=%u (max %u us)\n",
			 dpu_hw_frame_count(d), decon_rd(d, HW_SW_TRIG_CONTROL),
			 decon_rd(d, DATA_PATH_CONTROL_0), decon_rd(d, DATA_PATH_CONTROL_1),
			 d->nr_commits, d->nr_latch_timeouts, d->nr_rejects,
			 atomic_read(&d->te_count), d->nr_base_mismatch, d->nr_afbc,
			 d->nr_bounces, d->bounce_max_us);
		return 0;

	case S3CFB_POWER_MODE:
		if (get_user(v, (u32 __user *)argp))
			return -EFAULT;
		/* doze needs the panel driver; 4.9 ignores other modes too */
		dev_info(d->dev, "power mode %u ignored in adopt mode\n", v);
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

/*
 * Which driver ioctls HWC uses and what each got: every command number is
 * logged once, at its first call, with that call's result (the fb core
 * handles FBIOGET/FBIOPUT_VSCREENINFO, FBIOBLANK etc. before this).
 */
static int dpu_fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct dpu_dev *d = info->par;
	int ret = dpu_fb_ioctl_cmd(info, cmd, arg);
	bool first = true;
	unsigned int i;

	spin_lock(&d->ioc_lock);
	for (i = 0; i < d->ioc_nseen && first; i++)
		first = d->ioc_seen[i] != cmd;
	if (first && d->ioc_nseen < ARRAY_SIZE(d->ioc_seen))
		d->ioc_seen[d->ioc_nseen++] = cmd;
	else
		first = false;
	spin_unlock(&d->ioc_lock);
	if (first)
		dev_info(d->dev, "ioctl %#x (type %#x nr %u size %u) first call by %s: %d\n",
			 cmd, _IOC_TYPE(cmd), _IOC_NR(cmd), _IOC_SIZE(cmd), current->comm, ret);
	return ret;
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
	const struct fb_var_screeninfo *v = &info->var;
	u32 *pal = info->pseudo_palette;

	if (regno >= 16)
		return -EINVAL;
	pal[regno] = ((red >> 8) << v->red.offset) | ((green >> 8) << v->green.offset) |
		     ((blue >> 8) << v->blue.offset) |
		     (v->transp.length ? 0xffU << v->transp.offset : 0);
	return 0;
}

static void dpu_fb_layout(u32 dpp_fmt, struct fb_var_screeninfo *v)
{
	const u8 *p = dpu_fmt_pos[dpp_fmt < 8 ? dpp_fmt : DECON_PIXEL_FORMAT_BGRA_8888];
	bool alpha = dpp_fmt <= DECON_PIXEL_FORMAT_BGRA_8888;

	v->red = (struct fb_bitfield){ 8 * p[0], 8, 0 };
	v->green = (struct fb_bitfield){ 8 * p[1], 8, 0 };
	v->blue = (struct fb_bitfield){ 8 * p[2], 8, 0 };
	v->transp = (struct fb_bitfield){ 8 * p[3], alpha ? 8 : 0, 0 };
}

static int dpu_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	return (var->xoffset || var->yoffset) ? -EINVAL : 0;
}

static int dpu_fb_blank(int mode, struct fb_info *info)
{
	struct dpu_dev *d = info->par;

	/* panel power is not ours yet: blanking is accepted and ignored */
	dev_info(d->dev, "blank %d ignored (adopt mode, no panel driver)\n", mode);
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

/*
 * Full-screen window on the boot DPP. @dpp_fmt is the DPP-level format; the
 * user-level format in the config is its 4.9 translation (an involution).
 * fb0 itself uses d->fb_dpp_fmt, the IDMA format SBoot left: the boot buffer
 * keeps its byte order (SBoot: IDMA ARGB8888, 0xAARRGGBB words), so taking
 * the window over does not change a pixel on the panel. 4.9 decon_fb init
 * used DPP BGRA_8888, which showed the SBoot logo with swapped channels (v4).
 */
static void dpu_fb_window(struct dpu_dev *d, struct decon_win_config_data *wd,
			  u32 dpp_fmt)
{
	struct decon_win_config *w = &wd->config[d->boot_win];

	memset(wd, 0, sizeof(*wd));
	w->state = DECON_WIN_STATE_BUFFER;
	w->idma_type = d->boot_dpp;
	w->format = dpu_translate_fmt_to_dpp(dpp_fmt);
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
	bool iova;		/* DPU SysMMU: @dma is already DECON's IOVA */
};

static struct sg_table *dpu_kbuf_map(struct dma_buf_attachment *att,
				     enum dma_data_direction dir)
{
	struct dpu_kbuf *kb = att->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	if (kb->iova && att->dev != kb->dev)
		return ERR_PTR(-EINVAL);
	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);
	ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (ret)
		goto err;
	if (kb->iova) {
		/*
		 * DPU SysMMU: the identity-mapped boot framebuffer, or the
		 * IOVA dma_alloc_wc() returned. Only DECON attaches these, so
		 * the existing address is handed out unchanged.
		 */
		sg_dma_address(sgt->sgl) = kb->dma;
		sg_dma_len(sgt->sgl) = kb->size;
		sgt->nents = 1;
		return sgt;
	}
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
	struct dpu_kbuf *kb = att->dmabuf->priv;

	if (!kb->iova)
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
	kb->iova = !!d->iommu;
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

/* synchronous in-kernel commit of one buffer window (@win) */
static int dpu_kcommit_win(struct dpu_dev *d, struct decon_win_config_data *wd, int win,
			   struct dma_buf *db, u32 *latch_us)
{
	struct dma_buf *kbufs[DECON_MAX_WIN] = { };
	struct dma_fence *f = NULL;
	ktime_t t0 = ktime_get();
	long t;
	int ret;

	kbufs[win] = db;
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

/* synchronous in-kernel commit of the boot window */
static int dpu_kcommit(struct dpu_dev *d, struct decon_win_config_data *wd,
		       struct dma_buf *db, u32 *latch_us)
{
	return dpu_kcommit_win(d, wd, d->boot_win, db, latch_us);
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
	dpu_fb_window(d, wd, d->fb_dpp_fmt);
	fc0 = dpu_hw_frame_count(d);
	ret = dpu_kcommit(d, wd, db, &us);
	dma_buf_put(db);
	kfree(wd);
	dev_info(d->dev, "fb0 takeover: win%d dpp%d DPP format %u (SBoot IDMA format %u kept) at %pa: %d, latched after %u us, FRAME_COUNT %u -> %u, active IDMA base %#x IN_CON %#x (SFR IN_CON %#x)\n",
		 d->boot_win, d->boot_dpp, d->fb_dpp_fmt, d->boot_fmt, &d->fb_phys, ret, us, fc0,
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

		/* A/B are drawn as 0xAARRGGBB words: DPP ARGB_8888 (dpu_fmt_pos) */
		dpu_fb_window(d, wd, DECON_PIXEL_FORMAT_ARGB_8888);
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
			dpu_fb_window(d, wd, d->fb_dpp_fmt);
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

/* ---- display matrix self-test (kernel buffers only) ------------------- */

/*
 * Opt-in by DT (decon_f samsung,decon-matrix): a fixed, boot-relative
 * sequence of kernel-buffer commits through the normal commit/latch path,
 * each held for DPU_MX_STEP_MS, to see on camera which windows, channels
 * and formats reach the panel. No HWC, no fences from userspace, no DSIM,
 * panel or PMIC access. Two decon-pool buffers alternate so the buffer on
 * screen is never redrawn. fb0 as SBoot left it is always restored at the
 * end, also after an error, a stop or the time box.
 */
#define DPU_MX_STEP_MS		4000
#define DPU_MX_TIMEBOX_MS	40000
#define DPU_MX_RESTORE_MS	3000	/* keep retrying the fb0 restore this long */

enum { MX_NONE, MX_BANDS_WR, MX_GREEN_CROSS, MX_MAGENTA };

struct dpu_mx_step {
	const char *name;
	int win;		/* -1: boot window */
	int dpp;		/* -1: boot DPP */
	int dpp_fmt;		/* -1: the format SBoot left */
	int pattern;
};

static const struct dpu_mx_step dpu_mx_steps[] = {
	{ "1 baseline: fb0 as is (SBoot window)", -1, -1, -1, MX_NONE },
	{ "2 fb0 window, linear, SBoot format: white top / red bottom", -1, -1, -1, MX_BANDS_WR },
	{ "3 fb0 window, linear XBGR8888 (HWC format): white top / red bottom", -1, -1,
	  DECON_PIXEL_FORMAT_XBGR_8888, MX_BANDS_WR },
	{ "4 win0 on VGF0 (ch1), linear XBGR8888: green with white cross, fb0 window off", 0,
	  DECON_IDMA_VGF0, DECON_PIXEL_FORMAT_XBGR_8888, MX_GREEN_CROSS },
	/* 5 (AFBC solid-colour on VGF0) is not implemented, see the commit */
	{ "6 win0 on G1 (ch3), linear XBGR8888: magenta with white border, fb0 window off", 0,
	  DECON_IDMA_G1, DECON_PIXEL_FORMAT_XBGR_8888, MX_MAGENTA },
};

static u32 dpu_mx_px(u32 dpp_fmt, u8 r, u8 g, u8 b)
{
	const u8 *p = dpu_fmt_pos[dpp_fmt];

	return (u32)r << (8 * p[0]) | (u32)g << (8 * p[1]) | (u32)b << (8 * p[2]) |
	       0xffU << (8 * p[3]);
}

static void dpu_mx_draw(struct dpu_dev *d, u32 *p, int pattern, u32 fmt)
{
	u32 w = d->lcd.xres, h = d->lcd.yres;
	u32 white = dpu_mx_px(fmt, 0xff, 0xff, 0xff);

	switch (pattern) {
	case MX_BANDS_WR:
		st_fill(p, w, 0, 0, w, h / 2, white);
		st_fill(p, w, 0, h / 2, w, h - h / 2, dpu_mx_px(fmt, 0xff, 0, 0));
		break;
	case MX_GREEN_CROSS:
		st_fill(p, w, 0, 0, w, h, dpu_mx_px(fmt, 0, 0xff, 0));
		st_fill(p, w, (w - 120) / 2, 0, 120, h, white);
		st_fill(p, w, 0, (h - 120) / 2, w, 120, white);
		break;
	case MX_MAGENTA:
		st_fill(p, w, 0, 0, w, h, white);
		st_fill(p, w, 48, 48, w - 96, h - 96, dpu_mx_px(fmt, 0xff, 0, 0xff));
		break;
	}
	wmb();
}

/* boot time, UTC, and the window/channel registers as latched */
static void dpu_mx_log(struct dpu_dev *d, const char *what, int win, int dpp, int ret,
		       u32 us, dma_addr_t want)
{
	u64 ms = ktime_to_ms(ktime_get_boottime());
	struct timespec64 ts;
	struct tm tm;

	ktime_get_real_ts64(&ts);
	time64_to_tm(ts.tv_sec, 0, &tm);
	pr_info(DPU_MX_PFX "%s: at boot %llu.%03llu s, UTC %02d:%02d:%02d.%03ld: commit %d, latched after %u us, FRAME_COUNT %u\n",
		what, ms / 1000, ms % 1000, tm.tm_hour, tm.tm_min, tm.tm_sec,
		ts.tv_nsec / NSEC_PER_MSEC, ret, us, dpu_hw_frame_count(d));
	pr_info(DPU_MX_PFX "  active: DP0 %#x DP1 %#x win%d CTL0 %#x START %#x END %#x | IDMA%d IN_CON %#x (afbc %d fmt %u) BASE %#x (want %pad) SRC %#x IMG %#x | DPP%d IN_CON %#x IMG %#x\n",
		decon_rd_shd(d, DATA_PATH_CONTROL_0), decon_rd_shd(d, DATA_PATH_CONTROL_1),
		win, decon_rd_shd(d, WIN_CONTROL_0(win)), decon_rd_shd(d, WIN_START_POSITION(win)),
		decon_rd_shd(d, WIN_END_POSITION(win)), dpp,
		dpu_hw_dma_active_rd(d, dpp, IDMA_IN_CON),
		!!(dpu_hw_dma_active_rd(d, dpp, IDMA_IN_CON) & IDMA_AFBC_EN),
		(dpu_hw_dma_active_rd(d, dpp, IDMA_IN_CON) >> 11) & 0x1f,
		dpu_hw_dma_active_rd(d, dpp, IDMA_IN_BASE_ADDR_Y), &want,
		dpu_hw_dma_active_rd(d, dpp, IDMA_SRC_SIZE),
		dpu_hw_dma_active_rd(d, dpp, IDMA_IMG_SIZE), dpp,
		readl(d->dpp_regs + dpp * 0x1000 + DPP_IN_CON),
		readl(d->dpp_regs + dpp * 0x1000 + DPP_IMG_SIZE));
}

static bool dpu_mx_stop(struct dpu_dev *d)
{
	return kthread_should_stop() || READ_ONCE(d->stopping);
}

/* sleep until @ms of boot time; false if asked to stop */
static bool dpu_mx_wait(struct dpu_dev *d, u64 ms)
{
	for (;;) {
		u64 now = ktime_to_ms(ktime_get_boottime());

		if (dpu_mx_stop(d))
			return false;
		if (now >= ms)
			return true;
		msleep_interruptible(min_t(u64, ms - now, 100));
	}
}

static int dpu_mx_thread(void *data)
{
	struct dpu_dev *d = data;
	size_t sz = (size_t)d->lcd.xres * d->lcd.yres * 4;
	struct decon_win_config_data *wd = kzalloc(sizeof(*wd), GFP_KERNEL);
	struct dma_buf *buf[2] = { }, *fb;
	void *va[2] = { };
	u64 t0 = d->mx_at_ms;
	u32 us = 0;
	int i, n = 0, ret = 0;

	if (!wd)
		goto out;
	pr_info(DPU_MX_PFX "scheduled at %llu ms boot time, %zu steps of %u ms, fb0 restored at %llu ms\n",
		t0, ARRAY_SIZE(dpu_mx_steps), DPU_MX_STEP_MS,
		t0 + ARRAY_SIZE(dpu_mx_steps) * DPU_MX_STEP_MS);
	if (!dpu_mx_wait(d, t0))
		goto out;
	for (i = 0; i < 2; i++) {
		buf[i] = dpu_kbuf_create(d, sz, 0, &va[i]);
		if (IS_ERR(buf[i])) {
			pr_err(DPU_MX_PFX "cannot allocate %zu-byte buffer %d: %ld\n", sz, i,
			       PTR_ERR(buf[i]));
			buf[i] = NULL;
			goto restore;
		}
	}

	for (i = 0; i < ARRAY_SIZE(dpu_mx_steps); i++) {
		const struct dpu_mx_step *s = &dpu_mx_steps[i];
		int win = s->win < 0 ? d->boot_win : s->win;
		int dpp = s->dpp < 0 ? d->boot_dpp : s->dpp;
		u32 fmt = s->dpp_fmt < 0 ? d->fb_dpp_fmt : s->dpp_fmt;
		struct decon_win_config *w = &wd->config[win];
		dma_addr_t want = d->fb_phys;

		if (!dpu_mx_wait(d, t0 + (u64)i * DPU_MX_STEP_MS) ||
		    ktime_to_ms(ktime_get_boottime()) > t0 + DPU_MX_TIMEBOX_MS)
			goto restore;
		if (i == 4)
			pr_info(DPU_MX_PFX "5 AFBC solid colour on VGF0: skipped (no validated AFBC encoder)\n");
		if (s->pattern == MX_NONE) {
			dpu_mx_log(d, s->name, win, dpp, 0, 0, want);
			continue;
		}
		dpu_mx_draw(d, va[n], s->pattern, fmt);
		want = ((struct dpu_kbuf *)buf[n]->priv)->dma;
		memset(wd, 0, sizeof(*wd));
		w->state = DECON_WIN_STATE_BUFFER;
		w->idma_type = dpp;
		w->format = dpu_translate_fmt_to_dpp(fmt);	/* user-level */
		w->blending = DECON_BLENDING_NONE;
		w->plane_alpha = 0xff;
		w->src.w = w->src.f_w = w->dst.w = w->dst.f_w = d->lcd.xres;
		w->src.h = w->src.f_h = w->dst.h = w->dst.f_h = d->lcd.yres;
		ret = dpu_kcommit_win(d, wd, win, buf[n], &us);
		dpu_mx_log(d, s->name, win, dpp, ret, us, want);
		/*
		 * Any failure, latch timeout included, may leave either buffer
		 * scanned or referenced by a pending update: never draw again,
		 * restore fb0 now.
		 */
		if (ret)
			goto restore;
		n ^= 1;
	}
	dpu_mx_wait(d, t0 + ARRAY_SIZE(dpu_mx_steps) * DPU_MX_STEP_MS);

restore:
	/* 7: fb0 exactly as SBoot left it (the takeover configuration) */
	fb = dpu_kbuf_create(d, d->fb_size, d->fb_phys, NULL);
	if (IS_ERR(fb)) {
		pr_err(DPU_MX_PFX "7 restore: cannot wrap fb0: %ld\n", PTR_ERR(fb));
	} else {
		/*
		 * Keep asking until the restore latches: after a latch timeout
		 * the update still pending makes dpu_process() drop each attempt
		 * with -EBUSY until it clears. Capped at DPU_MX_RESTORE_MS; a
		 * stop request does not end it, the restore is what a stop must
		 * leave behind.
		 */
		ktime_t t_end = ktime_add_ms(ktime_get(), DPU_MX_RESTORE_MS);

		for (i = 1; ; i++) {
			dpu_fb_window(d, wd, d->fb_dpp_fmt);
			ret = dpu_kcommit(d, wd, fb, &us);
			if (!ret)
				break;
			pr_warn(DPU_MX_PFX "7 restore attempt %d: %d, retrying\n", i, ret);
			if (ktime_after(ktime_get(), t_end))
				break;
			msleep(100);
		}
		dpu_mx_log(d, "7 restore fb0 as SBoot left it", d->boot_win, d->boot_dpp, ret, us,
			   d->fb_phys);
		if (ret)
			pr_err(DPU_MX_PFX "fb0 was NOT restored after %d attempts in %u ms: the last matrix buffer may stay on screen\n",
			       i, DPU_MX_RESTORE_MS);
		dma_buf_put(fb);
	}
	for (i = 0; i < 2; i++)
		if (buf[i])
			dma_buf_put(buf[i]);	/* commits keep their own references */
	pr_info(DPU_MX_PFX "done%s\n", dpu_mx_stop(d) ? " (stopped early)" : "");
out:
	kfree(wd);
	return 0;
}

/* ---- adopt ----------------------------------------------------------- */

/* ---- DPU SysMMUs (IOMMU API), opt-in ----------------------------------- */

#define DPU_FRAME_DRAIN_MS	34	/* two frames at 60 Hz */

/* bitmask of the read-only SysMMU views whose CTRL bit 0 (enable) is set */
static unsigned int dpu_sysmmu_enabled(struct dpu_dev *d)
{
	unsigned int m = 0;
	int i;

	for (i = 0; i < DPU_NR_SYSMMU; i++)
		if (d->sysmmu[i] && (readl(d->sysmmu[i] + DPU_SYSMMU_CTRL) & 1))
			m |= BIT(i);
	return m;
}

static unsigned int dpu_sysmmu_present(struct dpu_dev *d)
{
	unsigned int m = 0;
	int i;

	for (i = 0; i < DPU_NR_SYSMMU; i++)
		if (d->sysmmu[i])
			m |= BIT(i);
	return m;
}

static void dpu_sysmmu_dump(struct dpu_dev *d, const char *when)
{
	int i;

	for (i = 0; i < DPU_NR_SYSMMU; i++) {
		void __iomem *s = d->sysmmu[i];

		if (!s)
			continue;
		dev_info(d->dev, DPU_IOMMU_PFX "sysmmu_dpu%d %s: CTRL=%#x CFG=%#x STATUS=%#x PT_BASE=%#x INT_STATUS=%#x\n",
			 i, when, readl(s + DPU_SYSMMU_CTRL), readl(s + DPU_SYSMMU_CFG),
			 readl(s + DPU_SYSMMU_STATUS), readl(s + DPU_SYSMMU_PT_BASE),
			 readl(s + DPU_SYSMMU_INT_STATUS));
	}
}

/*
 * SysMMU fault (hard IRQ, called by exynos-iommu, which never panics on the
 * 9810): stop TE-triggered frames so that a faulting buffer is fetched at
 * most until the end of the current frame. In command mode the panel keeps
 * its last frame in GRAM. The next commit unmasks the trigger.
 */
static int dpu_iommu_fault(struct iommu_domain *dom, struct device *dev,
			   unsigned long iova, int flags, void *token)
{
	struct dpu_dev *d = token;
	unsigned int n;
	bool stopped;

	/*
	 * iommu_set_fault_handler() writes the callback and the token one
	 * after the other; dpu_iommu_set_handler() keeps the fault IRQs off
	 * meanwhile, and a NULL token is ignored anyway.
	 */
	if (!d)
		return 0;
	n = atomic_inc_return(&d->iommu_faults);
	stopped = dpu_hw_trigger_stop_atomic(d);
	WRITE_ONCE(d->last_fault_iova, iova);
	if (stopped)
		d->fault_trigger_stops++;
	if (n <= 8 || stopped)
		dev_err_ratelimited(d->dev, DPU_IOMMU_PFX "SysMMU %s fault #%u at IOVA %#lx: %s\n",
				    (flags & IOMMU_FAULT_WRITE) ? "write" : "read", n, iova,
				    stopped ? "TE trigger masked until the next commit" :
					      "trigger already masked");
	return 0;
}

/* every 1 MiB of the boot framebuffer (and its last byte) must map 1:1 */
static int dpu_iommu_check_identity(struct dpu_dev *d)
{
	unsigned int n = 0, bad = 0;
	dma_addr_t iova;
	phys_addr_t pa;
	u64 off;

	for (off = 0; off < (u64)d->fb_size + SZ_1M; off += SZ_1M) {
		iova = d->fb_phys + min_t(u64, off, d->fb_size - 1);
		pa = iommu_iova_to_phys(d->iommu, iova);
		n++;
		if (pa != iova) {
			if (!bad)
				dev_err(d->dev, DPU_IOMMU_PFX "boot fb IOVA %pad -> %pa, not identity\n",
					&iova, &pa);
			bad++;
		}
	}
	dev_info(d->dev, DPU_IOMMU_PFX "boot fb %pa+%#zx identity map: %u/%u probes 1:1\n",
		 &d->fb_phys, d->fb_size, n - bad, n);
	return bad ? -ENODEV : 0;
}

/*
 * Take the runtime PM reference that keeps the DPU SysMMUs translating
 * (exynos-iommu device links). The IOMMU core created the DMA domain with
 * the boot framebuffer identity map (IOMMU_RESV_DIRECT) before attaching
 * it, so whenever a SysMMU starts translating, SBoot's scanout of
 * 0xcc000000 keeps reading the same physical memory.
 *
 * Normally the driver core already resumed the SysMMUs for the probe
 * (supplier pre-activation), and the attach enabled them then; that is
 * logged. If they are still in bypass here, they are enabled while DECON
 * is idle, as 4.9 does for a running DECON (decon_reg_init()): mask the TE
 * trigger, let the current frame drain, resume, unmask.
 */
static int dpu_iommu_enable(struct dpu_dev *d)
{
	unsigned int want = BIT(DPU_NR_SYSMMU) - 1, before, after;
	bool masked = false;
	u32 trig, fc0;
	int ret;

	if (dpu_sysmmu_present(d) != want) {
		dev_err(d->dev, DPU_IOMMU_PFX "SysMMU views %#x of %#x: cannot verify translation, refusing\n",
			dpu_sysmmu_present(d), want);
		return -ENODEV;
	}
	before = dpu_sysmmu_enabled(d);
	trig = decon_rd(d, HW_SW_TRIG_CONTROL);
	if (before == want) {
		dev_info(d->dev, DPU_IOMMU_PFX "SysMMUs %#x already translating at probe (enabled by the domain attach, after the identity map)\n",
			 before);
	} else if ((trig & (HW_TRIG_EN | HW_TRIG_MASK_DECON)) == HW_TRIG_EN) {
		dpu_hw_trigger(d, false);
		masked = true;
		msleep(DPU_FRAME_DRAIN_MS);
	}
	fc0 = dpu_hw_frame_count(d);

	pm_runtime_enable(d->dev);
	ret = pm_runtime_resume_and_get(d->dev);
	if (ret) {
		dev_err(d->dev, DPU_IOMMU_PFX "runtime resume (SysMMUs): %d\n", ret);
		pm_runtime_disable(d->dev);
		goto out;
	}
	d->rpm_held = true;

	after = dpu_sysmmu_enabled(d);
	dpu_sysmmu_dump(d, "after runtime resume");
	if (after != want) {
		dev_err(d->dev, DPU_IOMMU_PFX "SysMMUs %#x of %#x translating: IOVAs would be fetched as physical addresses, refusing\n",
			after, want);
		pm_runtime_put_sync(d->dev);
		pm_runtime_disable(d->dev);
		d->rpm_held = false;
		ret = -EIO;
		goto out;
	}
	dev_info(d->dev, DPU_IOMMU_PFX "SysMMUs translating (%#x -> %#x)%s, FRAME_COUNT %u while enabling\n",
		 before, after, masked ? " with the TE trigger masked" : "", fc0);
out:
	if (masked)
		dpu_hw_trigger(d, true);
	return ret;
}

/*
 * Install or remove the SysMMU fault callback. iommu_set_fault_handler()
 * stores the callback and its token separately and report_iommu_fault()
 * reads them without a lock, so the three SysMMU fault IRQs (requested by
 * exynos-iommu) are disabled meanwhile; disable_irq() also waits for a
 * handler already running on another CPU.
 */
static void dpu_iommu_set_handler(struct dpu_dev *d, struct iommu_domain *dom,
				  iommu_fault_handler_t fn, void *token)
{
	struct device_node *mmu;
	int irq[DPU_NR_SYSMMU];
	int i;

	for (i = 0; i < DPU_NR_SYSMMU; i++) {
		irq[i] = 0;
		mmu = of_parse_phandle(d->dev->of_node, "iommus", i);
		if (!mmu)
			continue;
		irq[i] = of_irq_get(mmu, 0);
		of_node_put(mmu);
		if (irq[i] > 0)
			disable_irq(irq[i]);
	}
	iommu_set_fault_handler(dom, fn, token);
	for (i = 0; i < DPU_NR_SYSMMU; i++)
		if (irq[i] > 0)
			enable_irq(irq[i]);
}

/*
 * Probe failure after dpu_iommu_init() succeeded. Remove the fault handler
 * and wait for running ones, so none can mask the trigger any more; drop
 * DECON's runtime PM hold (the SysMMUs go to bypass once the driver core
 * releases the probe's supplier references, right after the failed probe).
 * Until then they translate the boot fb 1:1, so DECON can run meanwhile.
 * Then clear the fault inhibit (no commit will) and hand back the trigger
 * as it was before the fault handler, so the splash is not left frozen.
 */
static void dpu_iommu_release(struct dpu_dev *d)
{
	if (!d->iommu)
		return;
	dpu_iommu_set_handler(d, d->iommu, NULL, NULL);
	if (d->rpm_held) {
		pm_runtime_put_sync(d->dev);
		pm_runtime_disable(d->dev);
		d->rpm_held = false;
	}
	if (dpu_hw_powered(d)) {
		dpu_hw_trigger_reset_fault(d, d->trig_boot);
		dev_info(d->dev, DPU_IOMMU_PFX "probe failed: fault handler removed, TE trigger back to %#x (now %#x)\n",
			 d->trig_boot, decon_rd(d, HW_SW_TRIG_CONTROL));
	}
	d->iommu = NULL;
}

/* sysmmu_dpu0..2 non-secure SFR bases (4.9 DT) */
static const resource_size_t dpu_sysmmu_base[DPU_NR_SYSMMU] = {
	0x160a0000, 0x160b0000, 0x160c0000,
};

/*
 * The DPPs sit behind all three DPU SysMMUs (dpu0 VGRF/VGF, dpu1 G0/VG0,
 * dpu2 G1/VG1/WB). With IOVAs a channel behind a SysMMU that is not ours
 * would fetch them as physical addresses, so "iommus" must name exactly
 * these three, each once, and DECON needs a view of each to verify them.
 */
static int dpu_iommu_check_controllers(struct dpu_dev *d)
{
	struct device_node *np = d->dev->of_node;
	unsigned int seen = 0;
	int n, i, j, ret;

	n = of_count_phandle_with_args(np, "iommus", "#iommu-cells");
	if (n != DPU_NR_SYSMMU) {
		dev_err(d->dev, DPU_IOMMU_PFX "\"iommus\" lists %d SysMMUs, the DPU needs sysmmu_dpu0..2: refusing\n",
			n);
		return -ENODEV;
	}
	for (i = 0; i < n; i++) {
		struct of_phandle_args args;
		struct resource res;

		ret = of_parse_phandle_with_args(np, "iommus", "#iommu-cells", i, &args);
		if (ret)
			return -ENODEV;
		ret = of_address_to_resource(args.np, 0, &res);
		of_node_put(args.np);
		if (ret)
			return -ENODEV;
		for (j = 0; j < DPU_NR_SYSMMU; j++)
			if (res.start == dpu_sysmmu_base[j])
				break;
		if (j == DPU_NR_SYSMMU || (seen & BIT(j)) || !d->sysmmu[i]) {
			dev_err(d->dev, DPU_IOMMU_PFX "iommus[%d] at %pa: %s, refusing\n", i, &res.start,
				j == DPU_NR_SYSMMU ? "not a DPU SysMMU" :
				(seen & BIT(j)) ? "listed twice" : "no read-only view");
			return -ENODEV;
		}
		seen |= BIT(j);
	}
	return 0;
}

/*
 * "iommus" in DT gives decon_f a DMA domain whose page table already holds
 * the boot framebuffer identity map. Anything else (no "iommus", or a
 * domain that is not a DMA-API domain) leaves d->iommu NULL: the default
 * contiguous + bounce path, unchanged.
 */
static int dpu_iommu_init(struct dpu_dev *d)
{
	struct iommu_domain *dom = iommu_get_domain_for_dev(d->dev);
	int ret;

	if (!dom)
		return 0;
	if (!(dom->type & __IOMMU_DOMAIN_DMA_API)) {
		dev_warn(d->dev, DPU_IOMMU_PFX "domain type %#x is not a DMA domain: SysMMU path off\n",
			 dom->type);
		return 0;
	}
	/*
	 * Refusing means failing the probe: decon_f's DMA ops are iommu-dma
	 * now, so even the bounce path hands out IOVAs. The driver core then
	 * drops the SysMMUs (bypass) and SBoot's scanout of 0xcc000000 goes
	 * on; the DTB without "iommus" is the fallback.
	 */
	ret = dpu_iommu_check_controllers(d);
	if (ret)
		return ret;
	d->iommu = dom;
	/* one IOVA run per buffer: iommu-dma merges the whole sg_table */
	dma_set_max_seg_size(d->dev, UINT_MAX);
	ret = dpu_iommu_check_identity(d);
	if (ret) {
		d->iommu = NULL;
		return ret;
	}
	/* handed back by dpu_iommu_release() if probe fails after this */
	d->trig_boot = decon_rd(d, HW_SW_TRIG_CONTROL);
	dpu_iommu_set_handler(d, dom, dpu_iommu_fault, d);
	dev_info(d->dev, DPU_IOMMU_PFX "DMA domain attached: dma-bufs map to cached IOVAs, no bounce copies\n");
	return 0;
}

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
		/*
		 * With the IOMMU, exynos-iommu refused any SysMMU that SBoot
		 * left translating, so an enabled one runs our domain (boot fb
		 * identity-mapped). Without it, enabled = foreign IOVAs.
		 */
		if ((ctrl & 1) && !d->iommu) {
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
	{
		/* IDMA_IMG_FORMAT_* 0..7 -> DPP-level DECON format (dpp_set_format) */
		static const u32 idma_to_dpp[8] = {
			DECON_PIXEL_FORMAT_ARGB_8888, DECON_PIXEL_FORMAT_ABGR_8888,
			DECON_PIXEL_FORMAT_RGBA_8888, DECON_PIXEL_FORMAT_BGRA_8888,
			DECON_PIXEL_FORMAT_XRGB_8888, DECON_PIXEL_FORMAT_XBGR_8888,
			DECON_PIXEL_FORMAT_RGBX_8888, DECON_PIXEL_FORMAT_BGRX_8888,
		};

		/* adopt accepted only formats <= IDMA_IMG_FORMAT_BGRX8888 */
		d->fb_dpp_fmt = idma_to_dpp[d->boot_fmt & 7];
	}
	dev_info(d->dev, "adopted: win%d dpp%d IDMA format %u at %pa; fb0 keeps it (DPP format %u)\n",
		 d->boot_win, d->boot_dpp, d->boot_fmt, &d->fb_phys, d->fb_dpp_fmt);

	/* SysMMUs held on (boot fb identity-mapped) before any trigger write */
	if (d->iommu) {
		int ret = dpu_iommu_enable(d);

		if (ret)
			return ret;
	}

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
	/* SysMMU: the same bounded recovery as a commit (ALPM exit, recovery) */
	if (dpu_pending_resolve(d, "command window")) {
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
		dpu_hw_trigger_restore(d, *trig);
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
	dpu_hw_trigger_restore(d, trig);
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
	dpu_fb_layout(d->fb_dpp_fmt, &info->var);
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
	spin_lock_init(&d->trig_lock);
	mutex_init(&d->iova_lock);
	INIT_LIST_HEAD(&d->iova_lru);
	atomic_set(&d->iommu_faults, 0);
	spin_lock_init(&d->ioc_lock);
	spin_lock_init(&d->diag_lock);
	INIT_LIST_HEAD(&d->diag_txt);
	INIT_WORK(&d->diag_emit_work, dpu_diag_emit);
	INIT_DELAYED_WORK(&d->rate_work, dpu_rate_work);
	INIT_LIST_HEAD(&d->queue);
	INIT_LIST_HEAD(&d->stale);
	init_waitqueue_head(&d->vsync_wait);
	INIT_DELAYED_WORK(&d->st_work, dpu_st_work);
	INIT_DELAYED_WORK(&d->pb_work, dpu_panel_bright_work);
	INIT_DELAYED_WORK(&d->pb_status_work, dpu_panel_status_work);
	INIT_DELAYED_WORK(&d->pb_alpm_work, dpu_panel_alpm_work);
	INIT_DELAYED_WORK(&d->fb_snap_work, dpu_fb_snap_work);
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
		struct resource *r, res;
		struct device_node *mmu;

		snprintf(name, sizeof(name), "sysmmu%d", i);
		r = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
		if (r) {
			d->sysmmu[i] = devm_ioremap_resource(dev, r);
			if (IS_ERR(d->sysmmu[i]))
				return PTR_ERR(d->sysmmu[i]);
			continue;
		}
		/*
		 * SysMMU opt-in DTB: no "sysmmuN" reg entries (exynos-iommu
		 * requests the whole SFR windows and a nested decon_f resource
		 * would block that). Read-only view of the first page of the
		 * i-th "iommus" node, mapped without claiming it.
		 */
		mmu = of_parse_phandle(np, "iommus", i);
		if (!mmu)
			continue;
		ret = of_address_to_resource(mmu, 0, &res);
		of_node_put(mmu);
		if (ret)
			continue;
		d->sysmmu[i] = devm_ioremap(dev, res.start, SZ_4K);
		if (!d->sysmmu[i])
			return -ENOMEM;
	}

	/*
	 * Streaming mappings 64-bit: dma-buf imports are only inspected (and
	 * bounced) or used when they lie below 4 GiB (dpu_buf_import()), so a
	 * 32-bit streaming mask only pushed high pages into swiotlb. Coherent
	 * allocations (bounce buffers, self-test) stay below 4 GiB for the
	 * 32-bit IDMA base register.
	 */
	ret = dma_set_mask(dev, DMA_BIT_MASK(64));
	if (!ret)
		ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	if (of_property_present(np, "memory-region")) {
		ret = of_reserved_mem_device_init(dev);
		if (ret)
			return dev_err_probe(dev, ret, "memory-region\n");
	}

	/* DPU SysMMUs, opt-in ("iommus"): before adopt reads the SysMMU state */
	ret = dpu_iommu_init(d);
	if (ret)
		goto err_mem;

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
	kthread_init_delayed_work(&d->idle_work, dpu_idle_snap);

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

	/* opt-in kernel-buffer display matrix (dpu_mx_thread) */
	if (of_property_read_bool(np, "samsung,decon-matrix")) {
		d->mx_at_ms = 60000;
		of_property_read_u32(np, "samsung,decon-matrix-at-ms", &d->mx_at_ms);
		d->mx_thread = kthread_create(dpu_mx_thread, d, "decon_matrix");
		if (IS_ERR(d->mx_thread)) {
			dev_warn(dev, "display matrix thread: %ld\n", PTR_ERR(d->mx_thread));
			d->mx_thread = NULL;
		} else {
			get_task_struct(d->mx_thread);	/* for kthread_stop() after it ends */
			wake_up_process(d->mx_thread);
		}
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
		/* read-only panel status at 5.5 s and 25 s, whenever the brightness runs */
		dpu_panel_status_schedule(d);
		/* opt-in: one 4.9 fix_green() ALPM cycle, after the brightness */
		if (d->pb_alpm_at_ms) {
			pr_info(DPU_PB_PFX "alpm cycle (4.9 fix_green) scheduled at %u ms uptime\n",
				d->pb_alpm_at_ms);
			queue_delayed_work(system_long_wq, &d->pb_alpm_work,
					   msecs_to_jiffies(d->pb_alpm_at_ms > now ?
							    d->pb_alpm_at_ms - now : 0));
		}
	} else if (ret != -ENODEV) {
		dev_warn(dev, "panel-bright disabled: %d\n", ret);
	}
	/* fb0: reservation, picture and counters at adopt, again at 5 s */
	dpu_check_fb_reservation(d);
	{
		u64 now = ktime_to_ms(ktime_get_boottime());
		u64 left = now < DPU_FB_SNAP_MS ? DPU_FB_SNAP_MS - now : 0;
		LIST_HEAD(txt);

		dpu_diag_fb(d, "at adopt", false, &txt);	/* + pending-bit baseline */
		dpu_diag_post(d, &txt);
		schedule_delayed_work(&d->fb_snap_work, msecs_to_jiffies(left));
	}
	dev_info(dev, "adopt mode ready: TE irq %d, %s\n", d->te_irq,
		 d->fb_taken ? "fb0 window owned" : "fb0 window still SBoot's");
	return 0;

err_fb:
	unregister_framebuffer(d->fb);
	framebuffer_release(d->fb);
err_worker:
	kthread_cancel_delayed_work_sync(&d->idle_work);
	kthread_destroy_worker(d->worker);
	dpu_commit_free(d->active);
	d->active = NULL;
err_mem:
	dpu_iommu_release(d);
	of_reserved_mem_device_release(dev);
	return ret;
}

static void dpu_stop(struct dpu_dev *d)
{
	WRITE_ONCE(d->stopping, true);
	cancel_delayed_work_sync(&d->pb_work);
	cancel_delayed_work_sync(&d->pb_status_work);	/* re-queues itself; cancel waits */
	cancel_delayed_work_sync(&d->pb_alpm_work);	/* exit + brightness even when stopping */
	cancel_delayed_work_sync(&d->st_work);
	cancel_delayed_work_sync(&d->fb_snap_work);
	if (d->mx_thread) {
		/* the matrix restores fb0 through the worker before it returns */
		kthread_stop(d->mx_thread);
		put_task_struct(d->mx_thread);
		d->mx_thread = NULL;
	}
	/*
	 * Diagnostic producers (dpu_process(), dpu_idle_snap(),
	 * dpu_fb_snap_work()) admitted before d->stopping was set finish in
	 * the drained worker or the cancelled work above; later ones see it
	 * and neither re-arm, start a rate count nor post text. Only then
	 * the rate count and the text queue are retired.
	 */
	kthread_flush_worker(d->worker);
	kthread_cancel_delayed_work_sync(&d->idle_work);
	cancel_delayed_work_sync(&d->rate_work);
	flush_work(&d->diag_emit_work);		/* prints and frees queued texts */
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
