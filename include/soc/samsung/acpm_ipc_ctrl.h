#ifndef __ACPM_IPC_CTRL_H__
#define __ACPM_IPC_CTRL_H__

typedef void (*ipc_callback)(unsigned int *cmd, unsigned int size);

struct ipc_config {
	unsigned int *cmd;
	unsigned int *indirection_base;
	unsigned int indirection_size;
	bool response;
	bool indirection;
};

#define ACPM_IPC_PROTOCOL_OWN			(31)
#define ACPM_IPC_PROTOCOL_RSP			(30)
#define ACPM_IPC_PROTOCOL_INDIRECTION		(29)
#define ACPM_IPC_PROTOCOL_ID			(26)
#define ACPM_IPC_PROTOCOL_IDX			(0x7 << ACPM_IPC_PROTOCOL_ID)
#define ACPM_IPC_PROTOCOL_DP_A			(25)
#define ACPM_IPC_PROTOCOL_DP_D			(24)
#define ACPM_IPC_PROTOCOL_DP_CMD		(0x3 << ACPM_IPC_PROTOCOL_DP_D)
#define ACPM_IPC_PROTOCOL_TEST			(23)
#define ACPM_IPC_PROTOCOL_STOP			(22)
#define ACPM_IPC_PROTOCOL_SEQ_NUM		(16)

#ifdef CONFIG_EXYNOS_ACPM
int acpm_ipc_request_channel(struct device_node *np, ipc_callback handler,
		unsigned int *id, unsigned int *size);
int acpm_ipc_release_channel(struct device_node *np, unsigned int channel_id);
int acpm_ipc_send_data(unsigned int channel_id, struct ipc_config *cfg);
int acpm_ipc_send_data_sync(unsigned int channel_id, struct ipc_config *cfg);
int acpm_ipc_set_ch_mode(struct device_node *np, bool polling);
/* 1 = AP polls (responses copied into cfg->cmd), 0 = interrupt, <0 error. */
int acpm_ipc_get_ch_mode(unsigned int channel_id);

/* Outcome of the 4.9-style dynamic plugin attach done by plugins_init(). */
struct exynos_acpm_plugin_status {
	int result;		/* 0: plugin attached; <0: not usable */
	bool found;		/* name present in the firmware plugin table */
	bool loaded;		/* this kernel copied the image and sent DP_A */
	bool post_attached;	/* descriptor is_attached re-read after DP_A */
	int attach_ret;		/* DP_A IPC result (only if loaded) */
	u32 index;
	u32 id;
	u32 base;
	u32 size;		/* declared plugin size */
	u32 image_bytes;
	u32 dp_response[4];
};
int exynos_acpm_get_plugin_status(const char *fw_name,
				  struct exynos_acpm_plugin_status *status);
#if IS_ENABLED(CONFIG_EXYNOS9810_DIAGNOSTIC_RESTART)
struct device;
/* Requires the exact supplier reference plus a managed device link, or a
 * successfully acquired supplier device lock held across the complete call.
 * A device reference alone does not protect drvdata or supplier devres.
 */
int acpm_ipc_prepare_restart(struct device *supplier, struct device_node *framework);
#endif

void exynos_acpm_reboot(void);
void acpm_stop_log(void);
#else

static inline int acpm_ipc_request_channel(struct device_node *np, ipc_callback handler,
		unsigned int *id, unsigned int *size)
{
	return 0;
}

static inline int acpm_ipc_release_channel(struct device_node *np, unsigned int channel_id)
{
	return 0;
}

static inline int acpm_ipc_send_data(unsigned int channel_id, struct ipc_config *cfg)
{
	return 0;
}

static inline int acpm_ipc_send_data_sync(unsigned int channel_id, struct ipc_config *cfg)
{
	return 0;
}

static inline int acpm_ipc_set_ch_mode(struct device_node *np, bool polling)
{
	return 0;
}

static inline int acpm_ipc_get_ch_mode(unsigned int channel_id)
{
	return -ENODEV;
}

struct exynos_acpm_plugin_status;
static inline int exynos_acpm_get_plugin_status(const char *fw_name,
		struct exynos_acpm_plugin_status *status)
{
	return -ENODEV;
}

static inline void exynos_acpm_reboot(void)
{
	return;
}

static inline void acpm_stop_log(void)
{
	return;
}
#endif

#ifdef CONFIG_EXYNOS_ACPM_S2D
void exynos_acpm_set_s2d_enable(int en);
void exynos_acpm_select_s2d_blk(u32 sel);
int exynos_acpm_s2d_update_en(void);
#else
static inline void exynos_acpm_set_s2d_enable(int en)
{
	return;
}

static inline int exynos_acpm_s2d_update_en(void)
{
	return 0;
}
static inline void exynos_acpm_select_s2d_blk(u32 sel)
{
	return;
}
#endif

#endif
