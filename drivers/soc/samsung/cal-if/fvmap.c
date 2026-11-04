#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <soc/samsung/cal-if.h>

#include "fvmap.h"
#include "cmucal.h"
#include "vclk.h"
#include "ra.h"
#include "vclk.h"

#define FVMAP_SIZE		(SZ_8K)
#define STEP_UV			(6250)

void __iomem *fvmap_base;
static bool fvmap_ready;
/* Domains whose FVMap entry has no matching ECT level table (4.9 logs
 * "ECT ... not found" for dvfs_disp_evt1 and dvfs_cp). They keep their copied
 * rate/voltage rows but publish no voltage table and get no PLL fixups. */
static unsigned long fvmap_unusable;
#define FVMAP_MAX_DOMAINS	BITS_PER_LONG
void __iomem *sram_fvmap_base;

static int init_margin_table[MAX_MARGIN_ID];
static int volt_offset_percent = 0;
#if defined(CONFIG_SAMSUNG_VST_CAL)
static int volt_vst_cal;
int volt_vst_cal_bdata;
EXPORT_SYMBOL(volt_vst_cal_bdata);
#endif
static int percent_margin_table[MAX_MARGIN_ID];

static int __init get_mif_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_MIF] = volt;

	return 0;
}
early_param("mif", get_mif_volt);

static int __init get_int_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_INT] = volt;

	return 0;
}
early_param("int", get_int_volt);

static int __init get_big_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_BIG] = volt;

	return 0;
}
early_param("big", get_big_volt);

static int __init get_lit_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_LIT] = volt;

	return 0;
}
early_param("lit", get_lit_volt);

static int __init get_g3d_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_G3D] = volt;

	return 0;
}
early_param("g3d", get_g3d_volt);

static int __init get_intcam_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_INTCAM] = volt;

	return 0;
}
early_param("intcam", get_intcam_volt);

static int __init get_cam_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_CAM] = volt;

	return 0;
}
early_param("cam", get_cam_volt);

static int __init get_disp_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_DISP] = volt;

	return 0;
}
early_param("disp", get_disp_volt);

static int __init get_g3dm_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_G3DM] = volt;

	return 0;
}
early_param("g3dm", get_g3dm_volt);

static int __init get_cp_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_CP] = volt;

	return 0;
}
early_param("cp", get_cp_volt);

static int __init get_fsys0_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_FSYS0] = volt;

	return 0;
}
early_param("fsys0", get_fsys0_volt);

static int __init get_aud_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_AUD] = volt;

	return 0;
}
early_param("aud", get_aud_volt);

static int __init get_iva_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_IVA] = volt;

	return 0;
}
early_param("iva", get_iva_volt);

static int __init get_score_volt(char *str)
{
	int volt;

	get_option(&str, &volt);
	init_margin_table[MARGIN_SCORE] = volt;

	return 0;
}
early_param("score", get_score_volt);

static int __init get_percent_margin_volt(char *str)
{
	int percent;

	get_option(&str, &percent);
	volt_offset_percent = percent;

	return 0;
}
early_param("volt_offset_percent", get_percent_margin_volt);

#if defined(CONFIG_SAMSUNG_VST_CAL)
static int __init get_vst_margin_volt(char *str)
{
	unsigned int vst_volt;

	get_option(&str, &vst_volt);
	volt_vst_cal = vst_volt;

	return 0;
}
early_param("vst_adjust", get_vst_margin_volt);
#endif

/* Evidence-only validation. No publication, margins, calibration or IO. */
int fvmap_validate_snapshot(const void *snapshot, size_t bytes)
{
	const u8 *base = snapshot;
	unsigned int count = cmucal_get_list_size(ACPM_VCLK_TYPE);
	unsigned int i, j;

	if (!snapshot || bytes != FVMAP_SIZE || !count ||
	    (u64)count * sizeof(struct fvmap_header) > bytes)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		struct fvmap_header header;
		struct vclk *vclk = cmucal_get_node(ACPM_VCLK_TYPE | i);

		memcpy(&header, base + i * sizeof(header), sizeof(header));
		if (i < FVMAP_MAX_DOMAINS && vclk && !vclk_get_lv_num(ACPM_VCLK_TYPE | i) &&
		    header.num_of_lv <= 48 && header.o_ratevolt <= bytes &&
		    (u64)header.num_of_lv * sizeof(struct rate_volt) <= bytes - header.o_ratevolt) {
			pr_warn("fvmap: domain %u %s has %u FVMap levels but no ECT table; voltage unavailable\n",
				i, vclk->name, header.num_of_lv);
			continue;
		}
		if (!vclk || header.num_of_pll > vclk->num_list ||
		    (header.num_of_lv && !vclk->lut) ||
		    header.num_of_lv > 48 || header.num_of_pll > header.num_of_members ||
		    header.num_of_lv != vclk_get_lv_num(ACPM_VCLK_TYPE | i) ||
		    header.o_ratevolt > bytes ||
		    (u64)header.num_of_lv * sizeof(struct rate_volt) > bytes - header.o_ratevolt ||
		    header.o_members > bytes ||
		    (u64)header.num_of_pll * sizeof(u16) > bytes - header.o_members) {
			pr_err("fvmap: snapshot rejects domain %u %s pll=%u/%u lv=%u ect_lv=%u members=%u o_rv=%#x o_mem=%#x\n",
			       i, vclk ? vclk->name : "?", header.num_of_pll,
			       vclk ? vclk->num_list : 0, header.num_of_lv,
			       vclk_get_lv_num(ACPM_VCLK_TYPE | i), header.num_of_members,
			       header.o_ratevolt, header.o_members);
			return -EINVAL;
		}
		for (j = 0; j < header.num_of_lv; j++) {
			struct rate_volt entry;

			memcpy(&entry, base + header.o_ratevolt + j * sizeof(entry), sizeof(entry));
			if (entry.rate != vclk->lut[j].rate || !entry.volt) {
				pr_err("fvmap: snapshot domain %u %s level %u rate %u/%u volt %u\n",
				       i, vclk->name, j, entry.rate, vclk->lut[j].rate, entry.volt);
				return -EPROTO;
			}
		}
		for (j = 0; j < header.num_of_pll; j++) {
			u16 offset;

			memcpy(&offset, base + header.o_members + j * sizeof(offset), sizeof(offset));
			if (offset > bytes || sizeof(struct pll_header) > bytes - offset)
				return -EINVAL;
		}
	}
	return 0;
}

int fvmap_set_raw_voltage_table(unsigned int id, int uV)
{
	struct fvmap_header *fvmap_header;
	struct rate_volt_header *fv_table;
	int num_of_lv;
	int idx, i;

	if (!smp_load_acquire(&fvmap_ready) || !sram_fvmap_base)
		return -ENODEV;
	if (!IS_ACPM_VCLK(id) || GET_IDX(id) >= cmucal_get_list_size(ACPM_VCLK_TYPE))
		return -EINVAL;
	idx = GET_IDX(id);

	fvmap_header = sram_fvmap_base;
	fv_table = sram_fvmap_base + fvmap_header[idx].o_ratevolt;
	num_of_lv = fvmap_header[idx].num_of_lv;

	for (i = 0; i < num_of_lv; i++)
		fv_table->table[i].volt += uV;

	return 0;
}

int fvmap_get_voltage_table(unsigned int id, unsigned int *table)
{
	struct fvmap_header *fvmap_header = fvmap_base;
	struct rate_volt_header *fv_table;
	int idx, i;
	int num_of_lv;

	if (!smp_load_acquire(&fvmap_ready) || !table || GET_IDX(id) >= cmucal_get_list_size(ACPM_VCLK_TYPE))
		return 0;
	if (!IS_ACPM_VCLK(id))
		return 0;

	idx = GET_IDX(id);
	if (idx < FVMAP_MAX_DOMAINS && test_bit(idx, &fvmap_unusable))
		return 0;

	fvmap_header = fvmap_base;
	fv_table = fvmap_base + fvmap_header[idx].o_ratevolt;
	num_of_lv = fvmap_header[idx].num_of_lv;

	for (i = 0; i < num_of_lv; i++)
		table[i] = fv_table->table[i].volt;

	return num_of_lv;

}

int fvmap_get_raw_voltage_table(unsigned int id)
{
	struct fvmap_header *fvmap_header;
	struct rate_volt_header *fv_table;
	int idx, i;
	int num_of_lv;
	unsigned int table[20];

	if (!smp_load_acquire(&fvmap_ready) || !sram_fvmap_base)
		return -ENODEV;
	if (!IS_ACPM_VCLK(id) || GET_IDX(id) >= cmucal_get_list_size(ACPM_VCLK_TYPE))
		return -EINVAL;
	idx = GET_IDX(id);

	fvmap_header = sram_fvmap_base;
	fv_table = sram_fvmap_base + fvmap_header[idx].o_ratevolt;
	num_of_lv = fvmap_header[idx].num_of_lv;

	for (i = 0; i < num_of_lv; i++)
		table[i] = fv_table->table[i].volt;

	for (i = 0; i < num_of_lv; i++)
		printk("dvfs id : %d  %d Khz : %d uv\n", ACPM_VCLK_TYPE | id, fv_table->table[i].rate, table[i]);

	return 0;
}

static void check_percent_margin(struct rate_volt_header *head, unsigned int num_of_lv)
{
	int org_volt;
	int percent_volt;
	int i;

	if (!volt_offset_percent)
		return;

	for (i = 0; i < num_of_lv; i++) {
		org_volt = head->table[i].volt;
		percent_volt = org_volt * volt_offset_percent / 100;
		head->table[i].volt = org_volt + rounddown(percent_volt, STEP_UV);
	}
}

#if defined(CONFIG_SAMSUNG_VST_CAL)
static void check_vst_margin(void)
{
	int i;
	int bit = 1;
	int tmp = 0;
	int crc = 0;
	int magic = 0;
	int margin = volt_vst_cal;

	/* initialize vst cal bdata before using it */
	volt_vst_cal_bdata = 0;

	if (!volt_vst_cal) {
		pr_info("[svst] no vst cal\n");
		return;
	}

	/* check magic */
	magic = margin & 0xffffffc0;
	if ((magic >> 6) != NAD_CRC_MAGIC) {
		pr_info("[svst] invalid magic = %d\n", magic);
		return;
	}

	/* check crc */
	crc = margin & 0x3;
	margin = margin >> 2;

	for (i = 0; i < 30; i++) {
		if (bit & margin)
			tmp++;
		bit = bit << 1;
	}
	tmp = (tmp % 4);
	if (tmp != 0)
		tmp = 4 - tmp;

	pr_info("[svst] crc = %d, tmp = %d\n", crc, tmp);

	if (crc != tmp) {
		pr_info("[svst] broken crc\n");
		return;
	}

	margin = margin & 0xf;
	volt_vst_cal_bdata = margin;
	pr_info("[svst] result = %d, bdata = %d\n", margin, volt_vst_cal_bdata);

	bit = 1;
	for (i = 0; i < 4; i++) {
		if (bit & margin) {
			if (i == 0) {
				pr_info("[svst] big[%d]\n", NAD_VST_CAL_VOLT);
				init_margin_table[MARGIN_BIG] += NAD_VST_CAL_VOLT;
			} else if (i == 1) {
				pr_info("[svst] lit[%d]\n", NAD_VST_CAL_VOLT);
				init_margin_table[MARGIN_LIT] += NAD_VST_CAL_VOLT;
			} else if (i == 2) {
				pr_info("[svst] g3d[%d]\n", NAD_VST_CAL_VOLT);
				init_margin_table[MARGIN_G3D] += NAD_VST_CAL_VOLT;
			} else {
				pr_info("[svst] mif[%d]\n", NAD_VST_CAL_VOLT);
				init_margin_table[MARGIN_MIF] += NAD_VST_CAL_VOLT;
			}
		}
		bit = bit << 1;
	}
}
#endif
static int get_vclk_id_from_margin_id(int margin_id)
{
	int size = cmucal_get_list_size(ACPM_VCLK_TYPE);
	int i;
	struct vclk *vclk;

	for (i = 0; i < size; i++) {
		vclk = cmucal_get_node(ACPM_VCLK_TYPE | i);

		if (vclk->margin_id == margin_id)
			return i;
	}

	return -EINVAL;
}

#define attr_percent(margin_id, type)								\
static ssize_t show_##type##_percent								\
(struct kobject *kobj, struct kobj_attribute *attr, char *buf)					\
{												\
	return snprintf(buf, PAGE_SIZE, "%d\n", percent_margin_table[margin_id]);		\
}												\
												\
static ssize_t store_##type##_percent								\
(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)		\
{												\
	int input, vclk_id;									\
												\
	if (!sscanf(buf, "%d", &input))								\
		return -EINVAL;									\
												\
	if (input < -100 || input > 100)							\
		return -EINVAL;									\
												\
	vclk_id = get_vclk_id_from_margin_id(margin_id);					\
	if (vclk_id == -EINVAL)									\
		return vclk_id;									\
	percent_margin_table[margin_id] = input;						\
	cal_dfs_set_volt_margin(vclk_id | ACPM_VCLK_TYPE, input);				\
												\
	return count;										\
}												\
												\
static struct kobj_attribute type##_percent =							\
__ATTR(type##_percent, 0600,									\
	show_##type##_percent, store_##type##_percent)

attr_percent(MARGIN_MIF, mif_margin);
attr_percent(MARGIN_INT, int_margin);
attr_percent(MARGIN_BIG, big_margin);
attr_percent(MARGIN_LIT, lit_margin);
attr_percent(MARGIN_G3D, g3d_margin);
attr_percent(MARGIN_INTCAM, intcam_margin);
attr_percent(MARGIN_CAM, cam_margin);
attr_percent(MARGIN_DISP, disp_margin);
attr_percent(MARGIN_CP, cp_margin);
attr_percent(MARGIN_FSYS0, fsys0_margin);
attr_percent(MARGIN_AUD, aud_margin);
attr_percent(MARGIN_IVA, iva_margin);
attr_percent(MARGIN_SCORE, score_margin);

static struct attribute *percent_margin_attrs[] = {
	&mif_margin_percent.attr,
	&int_margin_percent.attr,
	&big_margin_percent.attr,
	&lit_margin_percent.attr,
	&g3d_margin_percent.attr,
	&intcam_margin_percent.attr,
	&cam_margin_percent.attr,
	&disp_margin_percent.attr,
	&cp_margin_percent.attr,
	&fsys0_margin_percent.attr,
	&aud_margin_percent.attr,
	&iva_margin_percent.attr,
	&score_margin_percent.attr,
	NULL,
};

static const struct attribute_group percent_margin_group = {
	.attrs = percent_margin_attrs,
};

static void fvmap_copy_from_sram(void __iomem *map_base, void __iomem *sram_base)
{
	volatile struct fvmap_header *fvmap_header, *header;
	struct rate_volt_header *old, *new;
	struct clocks *clks;
	struct pll_header *plls;
	struct vclk *vclk;
	struct cmucal_clk *clk_node;
	unsigned int paddr_offset, fvaddr_offset;
	int size, margin;
	int i, j;

	fvmap_header = map_base;
	header = sram_base;

	size = cmucal_get_list_size(ACPM_VCLK_TYPE);

#if defined(CONFIG_SAMSUNG_VST_CAL)
	check_vst_margin();
#endif

	for (i = 0; i < size; i++) {
		/* load fvmap info */
		fvmap_header[i].dvfs_type = header[i].dvfs_type;
		fvmap_header[i].num_of_lv = header[i].num_of_lv;
		fvmap_header[i].num_of_members = header[i].num_of_members;
		fvmap_header[i].num_of_pll = header[i].num_of_pll;
		fvmap_header[i].num_of_mux = header[i].num_of_mux;
		fvmap_header[i].num_of_div = header[i].num_of_div;
		fvmap_header[i].gearratio = header[i].gearratio;
		fvmap_header[i].init_lv = header[i].init_lv;
		fvmap_header[i].num_of_gate = header[i].num_of_gate;
		fvmap_header[i].reserved[0] = header[i].reserved[0];
		fvmap_header[i].reserved[1] = header[i].reserved[1];
		fvmap_header[i].block_addr[0] = header[i].block_addr[0];
		fvmap_header[i].block_addr[1] = header[i].block_addr[1];
		fvmap_header[i].block_addr[2] = header[i].block_addr[2];
		fvmap_header[i].o_members = header[i].o_members;
		fvmap_header[i].o_ratevolt = header[i].o_ratevolt;
		fvmap_header[i].o_tables = header[i].o_tables;

		vclk = cmucal_get_node(ACPM_VCLK_TYPE | i);
		if (vclk == NULL)
			continue;
		pr_info("dvfs_type : %s - id : %x\n",
			vclk->name, fvmap_header[i].dvfs_type);
		pr_info("  num_of_lv      : %d\n", fvmap_header[i].num_of_lv);
		pr_info("  num_of_members : %d\n", fvmap_header[i].num_of_members);

		old = sram_base + fvmap_header[i].o_ratevolt;
		new = map_base + fvmap_header[i].o_ratevolt;

		check_percent_margin(old, fvmap_header[i].num_of_lv);

		if (i < FVMAP_MAX_DOMAINS && test_bit(i, &fvmap_unusable)) {
			pr_info("  voltage unavailable: no ECT level table\n");
			continue;
		}
		margin = init_margin_table[vclk->margin_id];
		if (margin)
			cal_dfs_set_volt_margin(i | ACPM_VCLK_TYPE, margin);

		for (j = 0; j < fvmap_header[i].num_of_lv; j++) {
			new->table[j].rate = old->table[j].rate;
			new->table[j].volt = old->table[j].volt;
			pr_info("  lv : [%7d], volt = %d uV (%d %%) \n",
				new->table[j].rate, new->table[j].volt,
				volt_offset_percent);
		}

		for (j = 0; j < fvmap_header[i].num_of_pll; j++) {
			clks = sram_base + fvmap_header[i].o_members;
			plls = sram_base + clks->addr[j];
			clk_node = cmucal_get_node(vclk->list[j]);
			if (clk_node == NULL)
				continue;
			paddr_offset = clk_node->paddr & 0xFFFF;
			fvaddr_offset = plls->addr & 0xFFFF;
			if (paddr_offset == fvaddr_offset)
				continue;

			clk_node->paddr += fvaddr_offset - paddr_offset;
			clk_node->pll_con0 += fvaddr_offset - paddr_offset;
			if (clk_node->pll_con1)
				clk_node->pll_con1 += (unsigned long long)(fvaddr_offset - paddr_offset);
		}
	}
}

static int fvmap_validate(void __iomem *base)
{
	struct fvmap_header *h = (struct fvmap_header *)base;
	unsigned int i, j, count = cmucal_get_list_size(ACPM_VCLK_TYPE);
	if (!base || count * sizeof(*h) > FVMAP_SIZE)
		return -EINVAL;
	fvmap_unusable = 0;
	for (i = 0; i < count; i++) {
		struct clocks *clks;
		struct vclk *vclk = cmucal_get_node(ACPM_VCLK_TYPE | i);
		struct rate_volt_header *rates;
		/* SRAM is mapped as device memory: unaligned accesses fault. */
		if (h[i].o_ratevolt % sizeof(u32) || h[i].o_members % sizeof(u16))
			return -EINVAL;
		if (i < FVMAP_MAX_DOMAINS && vclk && !vclk_get_lv_num(ACPM_VCLK_TYPE | i) &&
		    h[i].num_of_lv <= 48 &&
		    h[i].o_ratevolt + h[i].num_of_lv * sizeof(struct rate_volt) <= FVMAP_SIZE) {
			__set_bit(i, &fvmap_unusable);
			continue;
		}
		if (!vclk || h[i].num_of_pll > vclk->num_list ||
		    (h[i].num_of_lv && !vclk->lut))
			return -EINVAL;
		if (h[i].num_of_lv > 48 || h[i].num_of_pll > h[i].num_of_members ||
		    h[i].num_of_lv != vclk_get_lv_num(ACPM_VCLK_TYPE | i) ||
		    h[i].o_ratevolt + h[i].num_of_lv * sizeof(struct rate_volt) > FVMAP_SIZE ||
		    h[i].o_members + h[i].num_of_pll * sizeof(u16) > FVMAP_SIZE)
			return -EINVAL;
		rates = (struct rate_volt_header *)(base + h[i].o_ratevolt);
		for (j = 0; j < h[i].num_of_lv; j++)
			if (rates->table[j].rate != vclk->lut[j].rate ||
			    !rates->table[j].volt)
				return -EPROTO;
		clks = (struct clocks *)(base + h[i].o_members);
		for (j = 0; j < h[i].num_of_pll; j++)
			if (clks->addr[j] % sizeof(u32) ||
			    clks->addr[j] + sizeof(struct pll_header) > FVMAP_SIZE)
				return -EINVAL;
	}
	return 0;
}

int fvmap_init(void __iomem *sram_base)
{
	void __iomem *map_base;

	int ret;
	if (smp_load_acquire(&fvmap_ready))
		return -EBUSY;
	ret = fvmap_validate(sram_base);
	if (ret)
		return ret;
	map_base = kzalloc(FVMAP_SIZE, GFP_KERNEL);
	if (!map_base)
		return -ENOMEM;

	fvmap_base = map_base;
	sram_fvmap_base = sram_base;
	pr_info("%s:fvmap initialize %pK\n", __func__, sram_base);
	fvmap_copy_from_sram(map_base, sram_base);

	/*
	 * 4.9 also created /sys/power/percent_margin here. Its store sends
	 * MARGIN_REQ with the full vclk id, whose top byte lands on the DP_D/DP_A
	 * plugin-command bits of word 0, and passes a percentage as microvolts.
	 * Not exposed until that encoding is reworked; boot-time margins above
	 * are unaffected.
	 */
	(void)percent_margin_group;
	smp_store_release(&fvmap_ready, true);
	return 0;
}
