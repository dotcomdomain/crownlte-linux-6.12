/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/timekeeping.h>
#include <linux/kernel.h>
#include <linux/sched/clock.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/exynos-ss.h>

#include "acpm.h"
#include "acpm_ipc.h"
#include "fw_header/framework.h"

static struct acpm_ipc_info *acpm_ipc;
static bool acpm_ipc_ready;
static resource_size_t acpm_sram_size;
static bool acpm_sram_range(u32 offset, u64 bytes);
static struct workqueue_struct *debug_logging_wq;
static struct workqueue_struct *update_log_wq;
static struct acpm_debug_info *acpm_debug;
static bool is_acpm_stop_log = false;
static bool acpm_stop_log_req = false;
/* Metadata readers and supplier retirement share this mutex. */
static DEFINE_MUTEX(acpm_state_lock);
static struct device *acpm_framework_device;
static struct device_node *acpm_framework_supplier;
static bool acpm_irq_requested;
struct regulator_ss_info regulator_ss[REGULATOR_SS_MAX];
char reg_map[0x100] = {0};
bool is_set_regmap = false;

void acpm_ipc_set_waiting_mode(bool mode)
{
	if (smp_load_acquire(&acpm_ipc_ready))
		acpm_ipc->w_mode = mode;
}

void acpm_fw_log_level(unsigned int on)
{
	if (smp_load_acquire(&acpm_ipc_ready))
		acpm_debug->debug_log_level = on;
}

void acpm_ramdump(void)
{
	if (!smp_load_acquire(&acpm_ipc_ready) || !acpm_debug)
		return;
#ifdef CONFIG_EXYNOS_SNAPSHOT_ACPM
	if (acpm_debug->dump_size)
		memcpy(acpm_debug->dump_dram_base, acpm_debug->dump_base, acpm_debug->dump_size);
#endif
}

void timestamp_write(void)
{
	unsigned int tmp_index;
	if (!smp_load_acquire(&acpm_ipc_ready))
		return;
	if (spin_trylock(&acpm_debug->lock)) {
		tmp_index = __raw_readl(acpm_debug->time_index);

		tmp_index++;

		if (tmp_index >= acpm_debug->num_timestamps)
			tmp_index = 0;

		acpm_debug->timestamps[tmp_index] = sched_clock();

		__raw_writel(tmp_index, acpm_debug->time_index);
		exynos_acpm_timer_clear();

		spin_unlock(&acpm_debug->lock);
	}
}

struct regulator_ss_info *get_regulator_ss(int n)
{
	if (n < REGULATOR_SS_MAX)
		return &regulator_ss[n];
	else
		return NULL;
}

void set_reg_map(void)
{
	u32 idx;
	int i;

	for (i = 0; i < REGULATOR_SS_MAX; i++) {
		idx = regulator_ss[i].vsel_reg & 0xFF;
		if (idx == 0)
			continue;

		is_set_regmap = true;

		if (reg_map[idx] != 0)
			pr_err("duplicated set_reg_map [%d] reg_map %x\n", i, reg_map[idx]);

		reg_map[idx] = i;
	}
}

unsigned int get_reg_id(unsigned int addr)
{
	int id;

	if (addr >> 8 != 0x1)
		return NO_SS_RANGE;

	id = reg_map[addr & 0xff];
	if (id != 0)
		return id;

	return NO_SS_RANGE;
}

unsigned int get_reg_voltage(struct regulator_ss_info reg_info, unsigned int selector)
{
	if (reg_info.name[0] == 'L')
		selector = selector & 0x3F;

	return reg_info.min_uV + (reg_info.uV_step * (selector - reg_info.linear_min_sel));
}

void acpm_log_print(void)
{
	unsigned int front;
	unsigned int rear;
	unsigned int id;
	unsigned int index;
	unsigned int count;
	unsigned char str[9] = {0,};
	unsigned int val;
	unsigned int log_header;
	unsigned long long time;
	unsigned int log_level;
	unsigned int reg_id;

	if (!smp_load_acquire(&acpm_ipc_ready) || is_acpm_stop_log)
		return ;
	/* ACPM Log data dequeue & print */
	front = __raw_readl(acpm_debug->log_buff_front);
	rear = __raw_readl(acpm_debug->log_buff_rear);

	while (rear != front) {
		if (front >= acpm_debug->log_buff_len || rear >= acpm_debug->log_buff_len)
			return;
		log_header = __raw_readl(acpm_debug->log_buff_base + acpm_debug->log_buff_size * rear);

		/* log header information
		 * id: [31:28]
		 * log level : [27]
		 * index: [26:22]
		 * apm systick count: [15:0]
		 */
		id = (log_header & (0xF << LOG_ID_SHIFT)) >> LOG_ID_SHIFT;
		log_level = (log_header & (0x1 << LOG_LEVEL)) >> LOG_LEVEL;
		index = (log_header & (0x1f << LOG_TIME_INDEX)) >> LOG_TIME_INDEX;
		count = log_header & 0xffff;

		/* string length: log_buff_size - header(4) - integer_data(4) */
		memcpy_align_4(str, acpm_debug->log_buff_base + (acpm_debug->log_buff_size * rear) + 4,
				acpm_debug->log_buff_size - 8);

		val = __raw_readl(acpm_debug->log_buff_base + acpm_debug->log_buff_size * rear +
				acpm_debug->log_buff_size - 4);

		if (index >= acpm_debug->num_timestamps)
			return;
		time = acpm_debug->timestamps[index];

		/* peritimer period: (1 * 256) / 24.576MHz*/
		time += count * APM_PERITIMER_NS_PERIOD;

		/* addr : [19:8], val : [7:0]*/
		if (id == REGULATOR_INFO_ID) {
			if (is_set_regmap == false)
				set_reg_map();

			if (is_set_regmap == false)
				reg_id = NO_SET_REGMAP;
			else
				reg_id = get_reg_id(val >> 12);

			if (reg_id == NO_SS_RANGE)
				exynos_ss_regulator(time, "outSc", val >> 12, (val >> 4) & 0xFF, (val >> 4) & 0xFF, val & 0xF);
			else if (reg_id == NO_SET_REGMAP)
				exynos_ss_regulator(time, "noMap", val >> 12, (val >> 4) & 0xFF, (val >> 4) & 0xFF, val & 0xF);
			else
				exynos_ss_regulator(time, regulator_ss[reg_id].name, val >> 12,
						get_reg_voltage(regulator_ss[reg_id], (val >> 4) & 0xFF),
						(val >> 4) & 0xFF,
						val & 0xF);
		}
		exynos_ss_acpm(time, str, val);

		if (acpm_debug->debug_log_level == 1 || !log_level)
			pr_info("[ACPM_FW] : %llu id:%u, %s, %x\n", time, id, str, val);

		if (acpm_debug->log_buff_len == (rear + 1))
			rear = 0;
		else
			rear++;

		__raw_writel(rear, acpm_debug->log_buff_rear);
		front = __raw_readl(acpm_debug->log_buff_front);
	}

	if (acpm_stop_log_req) {
		is_acpm_stop_log = true;
		acpm_ramdump();
	}
}

void acpm_stop_log(void)
{
	acpm_stop_log_req = true;
}

static void acpm_update_log(struct work_struct *work)
{
	acpm_log_print();
}

static void acpm_debug_logging(struct work_struct *work)
{
	acpm_log_print();
	timestamp_write();

	if (smp_load_acquire(&acpm_ipc_ready))
		queue_delayed_work(debug_logging_wq, &acpm_debug->periodic_work,
				msecs_to_jiffies(acpm_debug->period));
}

int acpm_ipc_set_ch_mode(struct device_node *np, bool polling)
{
	int reg;
	int i, len, req_ch_id;
	const __be32 *prop;

	if (!np || !smp_load_acquire(&acpm_ipc_ready))
		return -ENODEV;

	prop = of_get_property(np, "acpm-ipc-channel", &len);
	if (!prop)
		return -ENOENT;
	req_ch_id = be32_to_cpup(prop);

	for(i = 0; i < acpm_ipc->num_channels; i++) {
		if (acpm_ipc->channel[i].id == req_ch_id) {

			reg = __raw_readl(acpm_ipc->intr + INTMR1);
			reg &= ~(1 << acpm_ipc->channel[i].id);
			reg |= polling << acpm_ipc->channel[i].id;
			__raw_writel(reg, acpm_ipc->intr + INTMR1);

			acpm_ipc->channel[i].polling = polling;

			return 0;
		}
	}

	return -ENODEV;
}

int acpm_ipc_request_channel(struct device_node *np, ipc_callback handler,
		unsigned int *id, unsigned int *size)
{
	struct callback_info *cb;
	int i, len, req_ch_id;
	const __be32 *prop;

	if (!np || !smp_load_acquire(&acpm_ipc_ready))
		return -ENODEV;

	prop = of_get_property(np, "acpm-ipc-channel", &len);
	if (!prop)
		return -ENOENT;
	req_ch_id = be32_to_cpup(prop);

	for(i = 0; i < acpm_ipc->num_channels; i++) {
		if (acpm_ipc->channel[i].id == req_ch_id) {
			*id = acpm_ipc->channel[i].id;
			*size = acpm_ipc->channel[i].tx_ch.size;

			if (handler) {
				cb = devm_kzalloc(acpm_ipc->dev, sizeof(struct callback_info),
						GFP_KERNEL);
				if (cb == NULL)
					return -ENOMEM;
				cb->ipc_callback = handler;
				cb->client = np;

				spin_lock(&acpm_ipc->channel[i].ch_lock);
				list_add(&cb->list, &acpm_ipc->channel[i].list);
				spin_unlock(&acpm_ipc->channel[i].ch_lock);
			}

			return 0;
		}
	}

	return -ENODEV;
}

int acpm_ipc_release_channel(struct device_node *np, unsigned int channel_id)
{
	struct acpm_ipc_ch *channel;
	struct list_head *cb_list;
	struct callback_info *cb;
	if (!smp_load_acquire(&acpm_ipc_ready) || channel_id >= acpm_ipc->num_channels)
		return -ENODEV;
	channel = &acpm_ipc->channel[channel_id];
	cb_list = &channel->list;

	list_for_each_entry(cb, cb_list, list) {
		if (cb->client == np) {
			spin_lock(&channel->ch_lock);
			list_del(&cb->list);
			spin_unlock(&channel->ch_lock);
			devm_kfree(acpm_ipc->dev, cb);
			break;
		}
	}

	return 0;
}

static bool check_response(struct acpm_ipc_ch *channel, struct ipc_config *cfg)
{
	unsigned int front;
	unsigned int rear;
	struct list_head *cb_list = &channel->list;
	struct callback_info *cb;
	unsigned int data;
	bool ret = true;
	unsigned int i;

	spin_lock(&channel->rx_lock);
	/* IPC command dequeue */
	front = __raw_readl(channel->rx_ch.front);
	rear = __raw_readl(channel->rx_ch.rear);

	i = rear;

	while (i != front) {
		if (front >= channel->rx_ch.len || rear >= channel->rx_ch.len ||
		    i >= channel->rx_ch.len) {
			spin_unlock(&channel->rx_lock);
			return true;
		}
		data = __raw_readl(channel->rx_ch.base + channel->rx_ch.size * i);
		data = (data >> ACPM_IPC_PROTOCOL_SEQ_NUM) & 0x3f;

		if (data == ((cfg->cmd[0] >> ACPM_IPC_PROTOCOL_SEQ_NUM) & 0x3f)) {
			memcpy_align_4(cfg->cmd, channel->rx_ch.base + channel->rx_ch.size * i,
					channel->rx_ch.size);
			memcpy_align_4(channel->cmd, channel->rx_ch.base + channel->rx_ch.size * i,
					channel->rx_ch.size);

			/* i: target command, rear: another command
			 * 1. i index command dequeue
			 * 2. rear index command copy to i index position
			 * 3. incresed rear index
			 */
			if (i != rear)
				memcpy_align_4(channel->rx_ch.base + channel->rx_ch.size * i,
						channel->rx_ch.base + channel->rx_ch.size * rear,
						channel->rx_ch.size);

			list_for_each_entry(cb, cb_list, list)
				if (cb && cb->ipc_callback)
					cb->ipc_callback(channel->cmd, channel->rx_ch.size);

			rear++;
			rear = rear % channel->rx_ch.len;

			__raw_writel(rear, channel->rx_ch.rear);
			front = __raw_readl(channel->rx_ch.front);

			if (rear == front) {
				__raw_writel((1 << channel->id), acpm_ipc->intr + INTCR1);
				if (rear != __raw_readl(channel->rx_ch.front)) {
					__raw_writel((1 << channel->id), acpm_ipc->intr + INTGR1);
				}
			}
			ret = false;
			break;
		}
		i++;
		i = i % channel->rx_ch.len;
	}

	spin_unlock(&channel->rx_lock);

	return ret;
}

static void dequeue_policy(struct acpm_ipc_ch *channel)
{
	unsigned int front;
	unsigned int rear;
	struct list_head *cb_list = &channel->list;
	struct callback_info *cb;

	spin_lock(&channel->rx_lock);

	if (channel->type == TYPE_BUFFER) {
		memcpy_align_4(channel->cmd, channel->rx_ch.base, channel->rx_ch.size);
		spin_unlock(&channel->rx_lock);
		list_for_each_entry(cb, cb_list, list)
			if (cb && cb->ipc_callback)
				cb->ipc_callback(channel->cmd, channel->rx_ch.size);

		return;
	}

	/* IPC command dequeue */
	front = __raw_readl(channel->rx_ch.front);
	rear = __raw_readl(channel->rx_ch.rear);

	while (rear != front) {
		if (front >= channel->rx_ch.len || rear >= channel->rx_ch.len) {
			spin_unlock(&channel->rx_lock);
			return;
		}
		memcpy_align_4(channel->cmd, channel->rx_ch.base + channel->rx_ch.size * rear,
				channel->rx_ch.size);

		list_for_each_entry(cb, cb_list, list)
			if (cb && cb->ipc_callback)
				cb->ipc_callback(channel->cmd, channel->rx_ch.size);

		if (channel->rx_ch.len == (rear + 1))
			rear = 0;
		else
			rear++;

		if (!channel->polling)
			complete(&channel->wait);

		__raw_writel(rear, channel->rx_ch.rear);
		front = __raw_readl(channel->rx_ch.front);
	}

	acpm_log_print();
	spin_unlock(&channel->rx_lock);
}

static irqreturn_t acpm_ipc_irq_handler(int irq, void *data)
{
	struct acpm_ipc_info *ipc = data;
	unsigned int status;
	int i;
	if (!smp_load_acquire(&acpm_ipc_ready))
		return IRQ_NONE;

	/* ACPM IPC INTERRUPT STATUS REGISTER */
	status = __raw_readl(acpm_ipc->intr + INTSR1);

	for (i = 0; i < acpm_ipc->num_channels; i++) {
		if (!ipc->channel[i].polling && (status & (0x1 << ipc->channel[i].id))) {
			/* ACPM IPC INTERRUPT PENDING CLEAR */
			__raw_writel(1 << ipc->channel[i].id, ipc->intr + INTCR1);
		}
	}

	ipc->intr_status = status;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t acpm_ipc_irq_handler_thread(int irq, void *data)
{
	struct acpm_ipc_info *ipc = data;
	int i;
	if (!smp_load_acquire(&acpm_ipc_ready))
		return IRQ_NONE;

	for (i = 0; i < acpm_ipc->num_channels; i++)
		if (!ipc->channel[i].polling && (ipc->intr_status & (1 << i)))
			dequeue_policy(&ipc->channel[i]);

	return IRQ_HANDLED;
}

static void apm_interrupt_gen(unsigned int id)
{
	/* APM NVIC INTERRUPT GENERATE */
	writel((1 << id) << 16, acpm_ipc->intr + INTGR0);
}

#if IS_ENABLED(CONFIG_EXYNOS9810_DIAGNOSTIC_RESTART)
/* Caller pins this exact supplier with a managed device link until unbind,
 * or holds its device_lock across this entire bounded call. A reference alone
 * does not protect drvdata/devres. Never call after an unowned failed probe.
 */
int acpm_ipc_prepare_restart(struct device *supplier,
			     struct device_node *framework)
{
	struct acpm_ipc_info *ipc;
	struct acpm_ipc_ch *ch;
	unsigned long flags;
	u32 id, front, rear, next, rx_front, rx_rear, seq, reply;
	u32 cmd[4] = { BIT(ACPM_IPC_PROTOCOL_STOP), 0, 0, 0 };
	u64 started;
	unsigned int polls = 0;
	int ret = -ETIMEDOUT;

	if (!supplier || !framework ||
	    !of_device_is_compatible(framework, "samsung,exynos-acpm") ||
	    of_property_read_u32(framework, "acpm-ipc-channel", &id))
		return -EINVAL;
	if (!smp_load_acquire(&acpm_ipc_ready))
		return -ENODEV;
	ipc = dev_get_drvdata(supplier);
	if (!ipc || ipc != READ_ONCE(acpm_ipc) || ipc->dev != supplier ||
	    !ipc->channel || !ipc->intr || id >= ipc->num_channels)
		return -ENODEV;
	ch = &ipc->channel[id];
	/* The original crownlte framework uses channel 4 and four-word queues. */
	if (id != 4 || ch->id != id || ch->type != TYPE_QUEUE ||
	    ch->rx_ch.size != sizeof(cmd) || ch->tx_ch.size != sizeof(cmd) ||
	    ch->rx_ch.len < 2 || ch->tx_ch.len < 2 ||
	    !ch->rx_ch.base || !ch->rx_ch.front || !ch->rx_ch.rear ||
	    !ch->tx_ch.base || !ch->tx_ch.front || !ch->tx_ch.rear)
		return -EINVAL;

	local_irq_save(flags);
	if (!spin_trylock(&ch->ch_lock)) {
		local_irq_restore(flags);
		return -EBUSY;
	}
	ret = -EBUSY;
	if (!spin_trylock(&ch->tx_lock))
		goto unlock_ch;
	if (!spin_trylock(&ch->rx_lock))
		goto unlock_tx;
	if (!ch->polling || !list_empty(&ch->list)) {
		ret = -EOPNOTSUPP;
		goto unlock_rx;
	}
	/* Refuse stale queued responses; do not consume another client's data. */
	rx_front = readl(ch->rx_ch.front);
	rx_rear = readl(ch->rx_ch.rear);
	if (rx_front >= ch->rx_ch.len || rx_rear >= ch->rx_ch.len ||
	    rx_front != rx_rear || ch->seq_num > 63) {
		ret = -EPROTO;
		goto unlock_rx;
	}
	started = ktime_get_mono_fast_ns();
	ret = -ETIMEDOUT;
	/* One total time limit and one iteration budget cover both queue waits. */
	for (;;) {
		if (++polls > 100000 ||
		    ktime_get_mono_fast_ns() - started >= IPC_TIMEOUT)
			goto unlock_rx;
		front = readl(ch->tx_ch.front);
		rear = readl(ch->tx_ch.rear);
		if (front >= ch->tx_ch.len || rear >= ch->tx_ch.len) {
			ret = -EPROTO;
			goto unlock_rx;
		}
		next = (front + 1) % ch->tx_ch.len;
		if (next != rear)
			break;
		cpu_relax();
	}
	if (readl(ch->rx_ch.front) != rx_rear ||
	    readl(ch->rx_ch.rear) != rx_rear) {
		ret = -EPROTO;
		goto unlock_rx;
	}
	seq = ch->seq_num % 63 + 1;
	ch->seq_num = seq;
	cmd[0] |= seq << ACPM_IPC_PROTOCOL_SEQ_NUM;
	memcpy_toio(ch->tx_ch.base + front * sizeof(cmd), cmd, sizeof(cmd));
	/* Ordered queue publication precedes the original mailbox notification. */
	writel(next, ch->tx_ch.front);
	writel(BIT(id) << 16, ipc->intr + INTGR0);
	for (;;) {
		if (++polls > 100000 ||
		    ktime_get_mono_fast_ns() - started >= IPC_TIMEOUT)
			goto unlock_rx;
		rx_front = readl(ch->rx_ch.front);
		rx_rear = readl(ch->rx_ch.rear);
		if (rx_front >= ch->rx_ch.len || rx_rear >= ch->rx_ch.len) {
			ret = -EPROTO;
			goto unlock_rx;
		}
		if (rx_front == rx_rear) {
			cpu_relax();
			continue;
		}
		reply = readl(ch->rx_ch.base + rx_rear * sizeof(cmd));
		if (((reply >> ACPM_IPC_PROTOCOL_SEQ_NUM) & 0x3f) != seq ||
		    !(reply & BIT(ACPM_IPC_PROTOCOL_STOP)) ||
		    (reply & (BIT(ACPM_IPC_PROTOCOL_INDIRECTION) |
			      ACPM_IPC_PROTOCOL_DP_CMD | BIT(ACPM_IPC_PROTOCOL_TEST)))) {
			ret = -EPROTO;
			goto unlock_rx;
		}
		rx_rear = (rx_rear + 1) % ch->rx_ch.len;
		writel(rx_rear, ch->rx_ch.rear);
		if (rx_rear == readl(ch->rx_ch.front)) {
			writel(BIT(id), ipc->intr + INTCR1);
			if (rx_rear != readl(ch->rx_ch.front))
				writel(BIT(id), ipc->intr + INTGR1);
		}
		ret = 0;
		break;
	}
unlock_rx:
	spin_unlock(&ch->rx_lock);
unlock_tx:
	spin_unlock(&ch->tx_lock);
unlock_ch:
	spin_unlock(&ch->ch_lock);
	local_irq_restore(flags);
	return ret;
}
#endif

static int enqueue_indirection_cmd(struct acpm_ipc_ch *channel,
		struct ipc_config *cfg)
{
	unsigned int front;
	unsigned int rear;
	unsigned int buf;
	bool timeout_flag = 0;

	if (cfg->indirection) {
		front = __raw_readl(channel->tx_ch.front);
		rear = __raw_readl(channel->tx_ch.rear);

		/* another indirection command check */
		while (rear != front) {
			if (front >= channel->tx_ch.len || rear >= channel->tx_ch.len)
				return -EPROTO;
			buf = __raw_readl(channel->tx_ch.base + channel->tx_ch.size * rear);

			if (buf & (1 << ACPM_IPC_PROTOCOL_INDIRECTION)) {

				UNTIL_EQUAL(true, rear != __raw_readl(channel->tx_ch.rear),
						timeout_flag);

				if (timeout_flag) {
					acpm_log_print();
					return -ETIMEDOUT;
				} else {
					rear = __raw_readl(channel->tx_ch.rear);
				}

			} else {
				if (channel->tx_ch.len == (rear + 1))
					rear = 0;
				else
					rear++;
			}
		}

		if (!cfg->indirection_size || cfg->indirection_size % 4 ||
		    cfg->indirection_size > channel->tx_ch.d_buff_size)
			return -EINVAL;
		if (cfg->indirection_base)
			memcpy_align_4(channel->tx_ch.direction, cfg->indirection_base,
					cfg->indirection_size);
		else
			return -EINVAL;
	}

	return 0;
}

int acpm_ipc_send_data_sync(unsigned int channel_id, struct ipc_config *cfg)
{
	int ret;
	struct acpm_ipc_ch *channel;

	ret = acpm_ipc_send_data(channel_id, cfg);

	if (!ret) {
		channel = &acpm_ipc->channel[channel_id];

		if (!channel->polling && cfg->response) {
			ret = wait_for_completion_interruptible_timeout(&channel->wait,
					msecs_to_jiffies(50));
			if (!ret) {
				pr_err("[%s] ipc_timeout!!!\n", __func__);
				ret = -ETIMEDOUT;
			} else if (ret > 0) {
				ret = 0;
			}
		}
	}

	return ret;
}

int acpm_ipc_send_data(unsigned int channel_id, struct ipc_config *cfg)
{
	unsigned int front;
	unsigned int rear;
	unsigned int tmp_index;
	struct acpm_ipc_ch *channel;
	bool timeout_flag = 0;
	int ret;
	u64 timeout, now;
	u32 retry_cnt = 0;

	if (!smp_load_acquire(&acpm_ipc_ready) || !cfg || channel_id >= acpm_ipc->num_channels)
		return -EIO;

	channel = &acpm_ipc->channel[channel_id];
	if (channel->type != TYPE_QUEUE)
		return -EOPNOTSUPP;

	spin_lock(&channel->tx_lock);

	front = __raw_readl(channel->tx_ch.front);
	rear = __raw_readl(channel->tx_ch.rear);

	if (front >= channel->tx_ch.len || rear >= channel->tx_ch.len) {
		spin_unlock(&channel->tx_lock);
		return -EPROTO;
	}
	tmp_index = front + 1;

	if (tmp_index >= channel->tx_ch.len)
		tmp_index = 0;

	/* buffer full check */
	UNTIL_EQUAL(true, tmp_index != __raw_readl(channel->tx_ch.rear), timeout_flag);
	if (timeout_flag) {
		acpm_log_print();
		acpm_debug->debug_log_level = 1;
		spin_unlock(&channel->tx_lock);
		pr_err("[%s] tx buffer full! timeout!!!\n", __func__);
		return -ETIMEDOUT;
	}

	if (!cfg->cmd) {
		spin_unlock(&channel->tx_lock);
		return -EIO;
	}

	if (++channel->seq_num == 64)
		channel->seq_num = 1;

	cfg->cmd[0] |= (channel->seq_num & 0x3f) << ACPM_IPC_PROTOCOL_SEQ_NUM;

	memcpy_align_4(channel->tx_ch.base + channel->tx_ch.size * front, cfg->cmd,
			channel->tx_ch.size);

	cfg->cmd[1] = 0;
	cfg->cmd[2] = 0;
	cfg->cmd[3] = 0;

	ret = enqueue_indirection_cmd(channel, cfg);
	if (ret) {
		pr_err("[ACPM] indirection command fail %d\n", ret);
		spin_unlock(&channel->tx_lock);
		return ret;
	}

	writel(tmp_index, channel->tx_ch.front);

	apm_interrupt_gen(channel->id);
	spin_unlock(&channel->tx_lock);

	if (channel->polling && cfg->response) {
retry:
		timeout = sched_clock() + IPC_TIMEOUT;
		timeout_flag = false;

		while (!(__raw_readl(acpm_ipc->intr + INTSR1) & (1 << channel->id)) ||
				check_response(channel, cfg)) {
			now = sched_clock();
			if (timeout < now) {
				if (retry_cnt++ < 5) {
					pr_err("acpm_ipc timeout retry %d"
						"now = %llu,"
						"timeout = %llu\n",
						retry_cnt, now, timeout);
					goto retry;
				}
				timeout_flag = true;
				break;
			} else {
				if (acpm_ipc->w_mode)
					usleep_range(50, 100);
				else
					cpu_relax();
			}
		}

		if (timeout_flag) {
			if (!check_response(channel, cfg))
				return 0;
			pr_err("%s Timeout error! now = %llu, timeout = %llu\n",
					__func__, now, timeout);
			pr_err("[ACPM] int_status:0x%x, ch_id: 0x%x\n",
					__raw_readl(acpm_ipc->intr + INTSR1),
					1 << channel->id);
			pr_err("[ACPM] queue, rx_rear:%u, rx_front:%u\n",
					__raw_readl(channel->rx_ch.rear),
					__raw_readl(channel->rx_ch.front));
			pr_err("[ACPM] queue, tx_rear:%u, tx_front:%u\n",
					__raw_readl(channel->tx_ch.rear),
					__raw_readl(channel->tx_ch.front));

			acpm_debug->debug_log_level = 1;
			acpm_log_print();
			acpm_debug->debug_log_level = 0;
			acpm_ramdump();

			return -ETIMEDOUT;
		}

		queue_work(update_log_wq, &acpm_debug->update_log_work);
	}

	return 0;
}

static int log_buffer_init(struct device *dev, struct device_node *node)
{
	const __be32 *prop;
	unsigned int num_timestamps = 0;
	int len = 0;
	unsigned int dump_base = 0;
	unsigned int dump_size = 0;

	prop = of_get_property(node, "num-timestamps", &len);
	if (prop)
		num_timestamps = be32_to_cpup(prop);

	acpm_debug = devm_kzalloc(dev, sizeof(struct acpm_debug_info), GFP_KERNEL);
	if (!acpm_debug)
		return -ENOMEM;
	if (!num_timestamps || num_timestamps > 4096 ||
	    acpm_ipc->initdata->ktime_index % sizeof(u32) ||
	    acpm_ipc->initdata->log_buf_rear % sizeof(u32) ||
	    acpm_ipc->initdata->log_buf_front % sizeof(u32) ||
	    acpm_ipc->initdata->log_data % sizeof(u32) ||
	    !acpm_sram_range(acpm_ipc->initdata->ktime_index, sizeof(u32)) ||
	    !acpm_sram_range(acpm_ipc->initdata->log_buf_rear, sizeof(u32)) ||
	    !acpm_sram_range(acpm_ipc->initdata->log_buf_front, sizeof(u32)) ||
	    !acpm_ipc->initdata->log_entry_len ||
	    acpm_ipc->initdata->log_entry_size != 16 ||
	    !acpm_sram_range(acpm_ipc->initdata->log_data,
		(u64)acpm_ipc->initdata->log_entry_len * acpm_ipc->initdata->log_entry_size))
		return -EPROTO;

	acpm_debug->time_index = acpm_ipc->sram_base + acpm_ipc->initdata->ktime_index;
	acpm_debug->num_timestamps = num_timestamps;
	acpm_debug->timestamps = devm_kzalloc(dev,
			sizeof(unsigned long long) * num_timestamps, GFP_KERNEL);
	if (!acpm_debug->timestamps)
		return -ENOMEM;
	acpm_debug->log_buff_rear = acpm_ipc->sram_base + acpm_ipc->initdata->log_buf_rear;
	acpm_debug->log_buff_front = acpm_ipc->sram_base + acpm_ipc->initdata->log_buf_front;
	acpm_debug->log_buff_base = acpm_ipc->sram_base + acpm_ipc->initdata->log_data;
	acpm_debug->log_buff_len = acpm_ipc->initdata->log_entry_len;
	acpm_debug->log_buff_size = acpm_ipc->initdata->log_entry_size;

	prop = of_get_property(node, "debug-log-level", &len);
	if (prop)
		acpm_debug->debug_log_level = be32_to_cpup(prop);

	prop = of_get_property(node, "dump-base", &len);
	if (prop)
		dump_base = be32_to_cpup(prop);

	prop = of_get_property(node, "dump-size", &len);
	if (prop)
		dump_size = be32_to_cpup(prop);

	if (dump_base && dump_size) {
		acpm_debug->dump_base = ioremap(dump_base, dump_size);
		acpm_debug->dump_size = dump_size;
	}

	prop = of_get_property(node, "logging-period", &len);
	if (prop)
		acpm_debug->period = be32_to_cpup(prop);

#ifdef CONFIG_EXYNOS_SNAPSHOT_ACPM
	acpm_debug->dump_dram_base = kzalloc(acpm_debug->dump_size, GFP_KERNEL);
	exynos_ss_printk("[ACPM] acpm framework SRAM dump to dram base: 0x%x\n",
			virt_to_phys(acpm_debug->dump_dram_base));
#endif
	pr_info("[ACPM] acpm framework SRAM dump to dram base: 0x%llx\n",
			virt_to_phys(acpm_debug->dump_dram_base));

	spin_lock_init(&acpm_debug->lock);
	return 0;
}

static bool acpm_sram_range(u32 offset, u64 bytes)
{
	return offset <= acpm_sram_size && bytes <= acpm_sram_size - offset;
}

/* ACPM's original SRAM paths use aligned 32-bit accesses. Preserve that
 * width and decode little-endian bytes in RAM, including odd name offsets.
 * The rounded backing-word extent is checked before the first read.
 */
static int acpm_sram_copy(struct acpm_ipc_info *ipc, u32 offset,
			  void *destination, size_t bytes)
{
	u8 *out = destination;
	u64 position = offset;
	u64 first = position & ~3ULL;
	u64 end = position + bytes;
	u64 rounded;

	if (!bytes)
		return 0;
	if (end < position || end > U64_MAX - 3)
		return -EPROTO;
	rounded = (end + 3) & ~3ULL;
	if (!acpm_sram_range(offset, bytes) ||
	    rounded - first > acpm_sram_size - first)
		return -EPROTO;
	while (bytes) {
		u32 word = readl(ipc->sram_base + (position & ~3ULL));
		unsigned int start = position & 3;
		unsigned int take = min_t(size_t, bytes, sizeof(word) - start);
		unsigned int i;

		for (i = 0; i < take; i++)
			out[i] = word >> ((start + i) * 8);
		out += take;
		position += take;
		bytes -= take;
	}
	return 0;
}

static int channel_init(void)
{
	unsigned int mask = 0;
	unsigned int i;
	struct ipc_channel wire, *table;
	u64 table_bytes;
	int ret;

	acpm_ipc->num_channels = acpm_ipc->initdata->ipc_ap_max;
	table_bytes = (u64)acpm_ipc->num_channels * sizeof(wire);
	if (!acpm_ipc->num_channels || acpm_ipc->num_channels > 32 ||
	    acpm_ipc->initdata->ipc_channels % sizeof(u32) ||
	    !acpm_sram_range(acpm_ipc->initdata->ipc_channels, table_bytes)) {
		dev_err(acpm_ipc->dev, "ACPM channel table rejects count=%u offset=%#x bytes=%llu\n",
			acpm_ipc->num_channels, acpm_ipc->initdata->ipc_channels,
			table_bytes);
		return -EPROTO;
	}
	acpm_ipc->channel = devm_kcalloc(acpm_ipc->dev, acpm_ipc->num_channels,
				       sizeof(*acpm_ipc->channel), GFP_KERNEL);
	if (!acpm_ipc->channel)
		return -ENOMEM;
	/* One bounded owned table supplies both evidence and validation.
	 * Never dereference unvalidated firmware buffer addresses for logging.
	 */
	table = devm_kcalloc(acpm_ipc->dev, acpm_ipc->num_channels,
			     sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;
	ret = acpm_sram_copy(acpm_ipc, acpm_ipc->initdata->ipc_channels,
			     table, table_bytes);
	if (ret)
		return ret;
	for (i = 0; i < acpm_ipc->num_channels; i++) {
		wire = table[i];
		dev_info(acpm_ipc->dev,
			 "ACPM channel snapshot index=%u id=%u type=%u polling=%u length=%u element=%u tx=%#x/%#x/%#x rx=%#x/%#x/%#x indr=%#x/%u mbox=%#x\n",
			 i, wire.id, wire.type, wire.ap_poll,
			 wire.ch.q_len, wire.ch.q_elem_size,
			 wire.ch.tx_rear, wire.ch.tx_front, wire.ch.tx_base,
			 wire.ch.rx_rear, wire.ch.rx_front, wire.ch.rx_base,
			 wire.ch.rx_indr_buf, wire.ch.rx_indr_buf_size,
			 wire.ch.mbox_addr);
	}
	for (i = 0; i < acpm_ipc->num_channels; i++) {
		const char *reason = NULL;
		struct acpm_ipc_ch *channel = &acpm_ipc->channel[i];

		/* Use exactly the owned descriptor reported above. */
		wire = table[i];
		if (wire.id != i)
			reason = "id-index";
		else if (wire.type != TYPE_QUEUE && wire.type != TYPE_BUFFER)
			reason = "type";
		else if (wire.ap_poll > 1)
			reason = "polling";
		else if (wire.type == TYPE_QUEUE && wire.ch.q_len < 2)
			reason = "queue-length";
		else if (!wire.ch.q_elem_size || wire.ch.q_elem_size % sizeof(u32))
			reason = "element-size";
		else if (wire.ch.tx_rear % sizeof(u32) ||
			 wire.ch.tx_front % sizeof(u32) ||
			 wire.ch.rx_rear % sizeof(u32) ||
			 wire.ch.rx_front % sizeof(u32) ||
			 wire.ch.tx_base % sizeof(u32) ||
			 wire.ch.rx_base % sizeof(u32) ||
			 (wire.ch.rx_indr_buf_size && wire.ch.rx_indr_buf % sizeof(u32)))
			reason = "word-alignment";
		else if (!acpm_sram_range(wire.ch.tx_rear, sizeof(u32)))
			reason = "tx-rear";
		else if (!acpm_sram_range(wire.ch.tx_front, sizeof(u32)))
			reason = "tx-front";
		else if (!acpm_sram_range(wire.ch.rx_rear, sizeof(u32)))
			reason = "rx-rear";
		else if (!acpm_sram_range(wire.ch.rx_front, sizeof(u32)))
			reason = "rx-front";
		else if (!acpm_sram_range(wire.ch.tx_base,
			 (u64)max(wire.ch.q_len, 1U) * wire.ch.q_elem_size))
			reason = "tx-queue";
		else if (!acpm_sram_range(wire.ch.rx_base,
			 (u64)max(wire.ch.q_len, 1U) * wire.ch.q_elem_size))
			reason = "rx-queue";
		else if (!acpm_sram_range(wire.ch.rx_indr_buf, wire.ch.rx_indr_buf_size))
			reason = "indirection";
		if (reason) {
			dev_err(acpm_ipc->dev,
				"ACPM channel rejects %s index=%u id=%u type=%u polling=%u length=%u element=%u tx=%#x/%#x/%#x rx=%#x/%#x/%#x indr=%#x/%u\n",
				reason, i, wire.id, wire.type, wire.ap_poll,
				wire.ch.q_len, wire.ch.q_elem_size,
				wire.ch.tx_rear, wire.ch.tx_front, wire.ch.tx_base,
				wire.ch.rx_rear, wire.ch.rx_front, wire.ch.rx_base,
				wire.ch.rx_indr_buf, wire.ch.rx_indr_buf_size);
			return -EPROTO;
		}
		channel->polling = wire.ap_poll;
		channel->id = wire.id;
		channel->type = wire.type;
		mask |= (u32)channel->polling << channel->id;
		channel->rx_ch.size = wire.ch.q_elem_size;
		channel->rx_ch.len = wire.ch.q_len;
		channel->rx_ch.rear = acpm_ipc->sram_base + wire.ch.tx_rear;
		channel->rx_ch.front = acpm_ipc->sram_base + wire.ch.tx_front;
		channel->rx_ch.base = acpm_ipc->sram_base + wire.ch.tx_base;
		channel->tx_ch.size = wire.ch.q_elem_size;
		channel->tx_ch.len = wire.ch.q_len;
		channel->tx_ch.rear = acpm_ipc->sram_base + wire.ch.rx_rear;
		channel->tx_ch.front = acpm_ipc->sram_base + wire.ch.rx_front;
		channel->tx_ch.base = acpm_ipc->sram_base + wire.ch.rx_base;
		channel->tx_ch.d_buff_size = wire.ch.rx_indr_buf_size;
		channel->tx_ch.direction = acpm_ipc->sram_base + wire.ch.rx_indr_buf;
		channel->cmd = devm_kzalloc(acpm_ipc->dev, channel->tx_ch.size, GFP_KERNEL);
		if (!channel->cmd)
			return -ENOMEM;
		init_completion(&channel->wait);
		spin_lock_init(&channel->rx_lock);
		spin_lock_init(&channel->tx_lock);
		spin_lock_init(&channel->ch_lock);
		INIT_LIST_HEAD(&channel->list);
	}
	__raw_writel(mask, acpm_ipc->intr + INTMR1);
	return 0;
}

/* Private registration binds the exact framework device to the sole enabled
 * IPC supplier. No compatible-string-only caller can read metadata.
 * Framework lifecycle mutex -> acpm_state_lock; never the reverse.
 */
int acpm_ipc_bind_framework(struct device *framework)
{
	struct device_node *node, *supplier = NULL;
	unsigned int count = 0;
	u32 channel;
	int ret = 0;

	if (!framework || !framework->of_node ||
	    !of_device_is_compatible(framework->of_node, "samsung,exynos-acpm") ||
	    !of_device_is_available(framework->of_node) ||
	    of_property_read_u32(framework->of_node, "acpm-ipc-channel", &channel) ||
	    channel != 4)
		return -EINVAL;
	for_each_compatible_node(node, NULL, "samsung,exynos-acpm-ipc") {
		if (!of_device_is_available(node))
			continue;
		count++;
		if (!supplier)
			supplier = of_node_get(node);
	}
	if (count != 1) {
		of_node_put(supplier);
		return -ENODEV;
	}
	mutex_lock(&acpm_state_lock);
	if (acpm_framework_device ||
	    (acpm_ipc && acpm_ipc->dev->of_node != supplier)) {
		ret = -EBUSY;
	} else {
		acpm_framework_device = get_device(framework);
		acpm_framework_supplier = supplier;
		supplier = NULL;
	}
	mutex_unlock(&acpm_state_lock);
	of_node_put(supplier);
	return ret;
}

void acpm_ipc_unbind_framework(struct device *framework)
{
	struct device *retired = NULL;
	struct device_node *supplier = NULL;

	mutex_lock(&acpm_state_lock);
	if (acpm_framework_device == framework) {
		retired = acpm_framework_device;
		supplier = acpm_framework_supplier;
		acpm_framework_device = NULL;
		acpm_framework_supplier = NULL;
	}
	mutex_unlock(&acpm_state_lock);
	of_node_put(supplier);
	put_device(retired);
}

/* Owned diagnostic metadata only. No SRAM pointer escapes this interface. */
void acpm_ipc_free_firmware_snapshot(struct acpm_firmware_snapshot *snapshot)
{
	if (!snapshot)
		return;
	kfree(snapshot->plugins);
	kfree(snapshot->fvmap);
	memset(snapshot, 0, sizeof(*snapshot));
}

static int acpm_snapshot_name(struct acpm_ipc_info *ipc, u32 offset,
			     char name[ACPM_PLUGIN_NAME_SIZE])
{
	unsigned int i;
	int ret;

	memset(name, 0, ACPM_PLUGIN_NAME_SIZE);
	/* Missing optional names retain the original no-match behavior. */
	if (!offset)
		return 0;
	if (!acpm_sram_range(offset, 1))
		return -EPROTO;
	for (i = 0; i < ACPM_PLUGIN_NAME_SIZE; i++) {
		if (!acpm_sram_range(offset, (u64)i + 1))
			return -EPROTO;
		if ((u64)offset + i > U32_MAX)
			return -EPROTO;
		ret = acpm_sram_copy(ipc, offset + i, &name[i], 1);
		if (ret)
			return ret;
		if (!name[i])
			return 0;
	}
	return -ENAMETOOLONG;
}

int acpm_ipc_copy_firmware_snapshot(struct device *framework,
				   struct acpm_firmware_snapshot *snapshot)
{
	struct acpm_framework header, after;
	struct acpm_ipc_info *ipc;
	struct plugin descriptor, again;
	u32 channel, fv_offset = 0;
	u64 table_bytes;
	unsigned int i;
	bool has_fv_offset;
	int ret = -ENODEV;

	if (!snapshot)
		return -EINVAL;
	memset(snapshot, 0, sizeof(*snapshot));
	if (!framework || !framework->of_node ||
	    !of_device_is_compatible(framework->of_node, "samsung,exynos-acpm") ||
	    !of_device_is_available(framework->of_node) ||
	    of_property_read_u32(framework->of_node, "acpm-ipc-channel", &channel) ||
	    channel != 4)
		return -EINVAL;
	has_fv_offset = !of_property_read_u32(framework->of_node,
					     "fvmap_offset", &fv_offset);

	/* Mutex spans every MMIO read; remove waits before devres can unmap. */
	mutex_lock(&acpm_state_lock);
	ipc = acpm_ipc;
	if (framework != acpm_framework_device ||
	    !smp_load_acquire(&acpm_ipc_ready) || !ipc ||
	    ipc->dev->of_node != acpm_framework_supplier ||
	    dev_get_drvdata(ipc->dev) != ipc)
		goto unlock;
	ret = acpm_sram_copy(ipc, ipc->initdata_base, &header, sizeof(header));
	if (ret)
		goto unlock;
	table_bytes = (u64)header.num_plugins * sizeof(descriptor);
	if (header.num_plugins > ACPM_SNAPSHOT_MAX_PLUGINS ||
	    header.plugins % sizeof(u32) ||
	    !acpm_sram_range(header.plugins, table_bytes)) {
		dev_err(ipc->dev, "ACPM snapshot rejects plugin table count=%u offset=%#x bytes=%llu\n",
			header.num_plugins, header.plugins, table_bytes);
		ret = -EPROTO;
		goto unlock;
	}
	/* Zero plugins is a valid empty snapshot, as in the original loop. */
	if (header.num_plugins) {
		snapshot->plugins = kcalloc(header.num_plugins,
					    sizeof(*snapshot->plugins), GFP_KERNEL);
		if (!snapshot->plugins) {
			ret = -ENOMEM;
			goto unlock;
		}
	}
	snapshot->num_plugins = header.num_plugins;
	for (i = 0; i < header.num_plugins; i++) {
		struct acpm_plugin_snapshot *entry = &snapshot->plugins[i];
		u32 descriptor_offset = header.plugins + i * sizeof(descriptor);
		char name_again[ACPM_PLUGIN_NAME_SIZE];
		u64 base;

		ret = acpm_sram_copy(ipc, descriptor_offset, &descriptor, sizeof(descriptor));
		if (ret)
			goto unlock;
		if (descriptor.is_attached > 1 || descriptor.stay_attached > 1) {
			ret = -EPROTO;
			goto failed_plugin;
		}
		if (descriptor.is_attached != descriptor.stay_attached) {
			ret = -EOPNOTSUPP;
			goto failed_plugin;
		}
		entry->id = descriptor.id;
		entry->base = descriptor.base_addr & ~1U;
		entry->bytes = descriptor.size;
		entry->attached = descriptor.is_attached;
		ret = acpm_snapshot_name(ipc, descriptor.fw_name, entry->name);
		if (ret)
			goto failed_plugin;
		if (entry->attached &&
		    !acpm_sram_range(entry->base, entry->bytes)) {
			ret = -EPROTO;
			goto failed_plugin;
		}
		if (entry->attached && has_fv_offset &&
		    (strstr(entry->name, "DVFS") || strstr(entry->name, "dvfs"))) {
			/* Original attached plugins locate FVMap by DT offset; the
			 * dynamic path appends it AFTER plugin.size. That field is
			 * not a documented FVMap-containing allocation extent.
			 */
			base = (u64)entry->base + fv_offset;
			if (snapshot->fvmap || base > U32_MAX || base % sizeof(u32) ||
			    !acpm_sram_range((u32)base, ACPM_SNAPSHOT_FVMAP_BYTES)) {
				ret = -EPROTO;
				goto failed_plugin;
			}
			snapshot->fvmap = kmalloc(ACPM_SNAPSHOT_FVMAP_BYTES, GFP_KERNEL);
			if (!snapshot->fvmap) {
				ret = -ENOMEM;
				goto failed_plugin;
			}
			ret = acpm_sram_copy(ipc, (u32)base, snapshot->fvmap,
					     ACPM_SNAPSHOT_FVMAP_BYTES);
			if (ret)
				goto failed_plugin;
			snapshot->fvmap_plugin = entry->id;
		}
		/* Detect metadata/name changes rather than rescanning without a bound. */
		ret = acpm_sram_copy(ipc, descriptor_offset, &again, sizeof(again));
		if (ret)
			goto failed_plugin;
		ret = acpm_snapshot_name(ipc, descriptor.fw_name, name_again);
		if (ret)
			goto failed_plugin;
		if (memcmp(&descriptor, &again, sizeof(descriptor)) ||
		    memcmp(entry->name, name_again, sizeof(name_again))) {
			ret = -EAGAIN;
			goto failed_plugin;
		}
		continue;
failed_plugin:
		dev_err(ipc->dev, "ACPM snapshot rejects plugin index=%u id=%u name_offset=%#x ret=%d\n",
			i, descriptor.id, descriptor.fw_name, ret);
		goto unlock;
	}
	ret = acpm_sram_copy(ipc, ipc->initdata_base, &after, sizeof(after));
	if (ret)
		goto unlock;
	if (memcmp(&header, &after, sizeof(header))) {
		ret = -EAGAIN;
		goto unlock;
	}
	ret = 0;
unlock:
	mutex_unlock(&acpm_state_lock);
	if (ret)
		acpm_ipc_free_firmware_snapshot(snapshot);
	return ret;
}

static int acpm_ipc_probe_core(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct resource *res;
	int ret = 0, len;
	const __be32 *prop;

	if (!node) {
		dev_err(&pdev->dev, "driver doesnt support"
				"non-dt devices\n");
		return -ENODEV;
	}

	dev_info(&pdev->dev, "acpm_ipc probe\n");

	acpm_ipc = devm_kzalloc(&pdev->dev,
			sizeof(struct acpm_ipc_info), GFP_KERNEL);

	if (!acpm_ipc)
		return -ENOMEM;


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	acpm_ipc->intr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(acpm_ipc->intr))
		return PTR_ERR(acpm_ipc->intr);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return -ENODEV;
	if (res->start % sizeof(u32))
		return -EPROTO;
	acpm_sram_size = resource_size(res);
	acpm_ipc->sram_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(acpm_ipc->sram_base))
		return PTR_ERR(acpm_ipc->sram_base);

	prop = of_get_property(node, "initdata-base", &len);
	if (prop) {
		acpm_ipc->initdata_base = be32_to_cpup(prop);
	} else {
		dev_err(&pdev->dev, "Parsing initdata_base failed.\n");
		return -EINVAL;
	}
	if (acpm_ipc->initdata_base % sizeof(u32) ||
	    !acpm_sram_range(acpm_ipc->initdata_base, sizeof(struct acpm_framework)))
		return -EPROTO;
	acpm_ipc->initdata = devm_kzalloc(&pdev->dev,
				       sizeof(*acpm_ipc->initdata), GFP_KERNEL);
	if (!acpm_ipc->initdata)
		return -ENOMEM;
	ret = acpm_sram_copy(acpm_ipc, acpm_ipc->initdata_base,
			     acpm_ipc->initdata, sizeof(*acpm_ipc->initdata));
	if (ret)
		return ret;

	acpm_ipc->dev = &pdev->dev;

	ret = log_buffer_init(&pdev->dev, node);
	if (ret)
		return ret;
	ret = channel_init();
	if (ret)
		return ret;
	acpm_ipc->irq = irq_of_parse_and_map(node, 0);

	ret = devm_request_threaded_irq(&pdev->dev, acpm_ipc->irq, acpm_ipc_irq_handler,
			acpm_ipc_irq_handler_thread,
			IRQF_ONESHOT,
			dev_name(&pdev->dev), acpm_ipc);

	if (ret) {
		dev_err(&pdev->dev, "failed to register acpm_ipc interrupt:%d\n", ret);
		return ret;
	}

	acpm_irq_requested = true;

	update_log_wq = create_freezable_workqueue("acpm_update_log");
	if (!update_log_wq)
		return -ENOMEM;
	INIT_WORK(&acpm_debug->update_log_work, acpm_update_log);

	if (acpm_debug->period) {
		debug_logging_wq = create_freezable_workqueue("acpm_debug_logging");
		if (!debug_logging_wq)
			return -ENOMEM;
		INIT_DELAYED_WORK(&acpm_debug->periodic_work, acpm_debug_logging);


	}

	/* Publish instance ownership only after the complete successful probe. */
	platform_set_drvdata(pdev, acpm_ipc);
	smp_store_release(&acpm_ipc_ready, true);
	if (debug_logging_wq)
		queue_delayed_work(debug_logging_wq, &acpm_debug->periodic_work,
				    msecs_to_jiffies(10000));
	return ret;
}

/* Called with acpm_state_lock held. Snapshots have completed before entry.
 * IRQ/work callbacks may have passed ready already: quiesce them before
 * withdrawing their legacy globals. This does not retrofit lifetime pins
 * into arbitrary concurrent legacy transport callers.
 */
static void acpm_ipc_retire(struct platform_device *pdev)
{
	smp_store_release(&acpm_ipc_ready, false);
	if (acpm_irq_requested) {
		devm_free_irq(&pdev->dev, acpm_ipc->irq, acpm_ipc);
		acpm_irq_requested = false;
	}
	if (debug_logging_wq) {
		cancel_delayed_work_sync(&acpm_debug->periodic_work);
		destroy_workqueue(debug_logging_wq);
		debug_logging_wq = NULL;
	}
	if (update_log_wq) {
		cancel_work_sync(&acpm_debug->update_log_work);
		destroy_workqueue(update_log_wq);
		update_log_wq = NULL;
	}
	if (acpm_debug) {
		if (acpm_debug->dump_base)
			iounmap(acpm_debug->dump_base);
		kfree(acpm_debug->dump_dram_base);
	}
	platform_set_drvdata(pdev, NULL);
	acpm_debug = NULL;
	acpm_ipc = NULL;
	acpm_sram_size = 0;
}

static int acpm_ipc_probe(struct platform_device *pdev)
{
	int ret;

	mutex_lock(&acpm_state_lock);
	if (acpm_ipc ||
	    (acpm_framework_supplier && pdev->dev.of_node != acpm_framework_supplier)) {
		ret = -EBUSY;
		goto out;
	}
	ret = acpm_ipc_probe_core(pdev);
	if (ret)
		acpm_ipc_retire(pdev);
out:
	mutex_unlock(&acpm_state_lock);
	return ret;
}

static void acpm_ipc_remove(struct platform_device *pdev)
{
	mutex_lock(&acpm_state_lock);
	if (platform_get_drvdata(pdev) == acpm_ipc && acpm_ipc)
		acpm_ipc_retire(pdev);
	mutex_unlock(&acpm_state_lock);
}

static const struct of_device_id acpm_ipc_match[] = {
	{ .compatible = "samsung,exynos-acpm-ipc" },
	{},
};

static struct platform_driver samsung_acpm_ipc_driver = {
	.probe	= acpm_ipc_probe,
	.remove	= acpm_ipc_remove,
	.driver	= {
		.name = "exynos-acpm-ipc",
		.owner	= THIS_MODULE,
		.of_match_table	= acpm_ipc_match,
	},
};

static int __init exynos_acpm_ipc_init(void)
{
	return platform_driver_register(&samsung_acpm_ipc_driver);
}
arch_initcall(exynos_acpm_ipc_init);
