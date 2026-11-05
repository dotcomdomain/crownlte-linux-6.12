#ifndef __ACPM_MFD_H__
#define __ACPM_MFD_H__
#include <linux/types.h>
#include <linux/errno.h>

/* Define shift */
#define SHIFT_BYTE			(8)
#define SHIFT_REG			(0)
#define SHIFT_TYPE			(8)
#define SHIFT_FUNC			(0)
#define SHIFT_DEST			(8)
#define SHIFT_CNT			(8)
#define SHIFT_WRITE_VAL			(8)
#define SHIFT_UPDATE_VAL		(8)
#define SHIFT_UPDATE_MASK		(16)
#define SHIFT_RETURN			(24)
#define SHIFT_BULK_VAL   		SHIFT_BYTE

/* Define mask */
#define MASK_BYTE			(0xFF)
#define MASK_REG			MASK_BYTE
#define MASK_TYPE			(0xF)
#define MASK_FUNC       		MASK_BYTE
#define MASK_DEST			MASK_BYTE
#define MASK_CNT			MASK_BYTE
#define MASK_WRITE_VAL			MASK_BYTE
#define MASK_UPDATE_VAL			MASK_BYTE
#define MASK_UPDATE_MASK		MASK_BYTE
#define MASK_RETURN			MASK_BYTE
#define MASK_BULK_VAL 			MASK_BYTE

/* Command */
#define set_protocol(data, protocol)	        	((data & MASK_##protocol) << SHIFT_##protocol)
#define set_bulk_protocol(data, protocol, n)	        ((data & MASK_##protocol) << (SHIFT_##protocol * (n)))
#define read_protocol(data, protocol)	        	((data >> SHIFT_##protocol) & MASK_##protocol)
#define read_bulk_protocol(data, protocol, n)	        ((data >> (SHIFT_##protocol * (n))) & MASK_##protocol)

#define ACPM_MFD_PREFIX		"ACPM-MFD: "

#ifndef pr_fmt
#define pr_fmt(fmt) fmt
#endif

#ifdef ACPM_MFD_DEBUG
#define ACPM_MFD_PRINT(fmt, ...) printk(ACPM_MFD_PREFIX pr_fmt(fmt), ##__VA_ARGS__)
#else
#define ACPM_MFD_PRINT(fmt, ...)
#endif

enum mfd_func {
        FUNC_READ,
        FUNC_WRITE,
        FUNC_UPDATE,
        FUNC_BULK_READ,
        FUNC_BULK_WRITE,
};

/*
 * S2MPS18 slave "types" carried in cmd[0] bits 11:8. 4.9 passed the dummy
 * I2C client address (s2mps18_core.c I2C_ADDR_TOP/PMIC/RTC/DEBUG).
 */
#define ACPM_MFD_TYPE_TOP		0x0
#define ACPM_MFD_TYPE_PMIC		0x1
#define ACPM_MFD_TYPE_RTC		0x2

/*
 * G3D rail request (4.9 cal-if.c cal_pd_control(), pd index 8): FUNC_UPDATE
 * of "type 3, reg 0x2a" with mask 0xc0. Type 3 is no SPEEDY slave. The APM
 * firmware MFD update handler (sboot copy, APM address 0x5064) intercepts
 * address 0x32a with mask 0xc0 before its generic SPEEDY read-modify-write
 * and calls its G3D rail routine (APM 0x4e4c) with the value:
 *   0xc0: PMU 0x14064678 = 1, wait 0x1406467c bit 0 set (<= 1999 polls),
 *         then S2MPS18 PMIC B6CTRL (0x12a) bits 3:2 = 0b10;
 *   0x00: B6CTRL bits 3:2 = 0b00, PMU 0x14064678 = 0, wait bit 0 clear;
 *   other: nothing.
 * A poll timeout is only logged ("g3d_cnt"): the reply still says success.
 */
#define ACPM_MFD_TYPE_G3D_RAIL		0x3
#define ACPM_MFD_G3D_RAIL_REG		0x2a
#define ACPM_MFD_G3D_RAIL_MASK		0xc0
#define ACPM_MFD_G3D_RAIL_ON		0xc0
#define ACPM_MFD_G3D_RAIL_OFF		0x00

/*
 * Touch IC rails (4.9 s2mps18.c s2m_enable() for the sec_ts consumers
 * tsp_io/tsp_avdd): FUNC_UPDATE type 1 (PMIC), mask 0xc0 = the enable/mode
 * field. The APM firmware treats every update other than 0x32a/0xc0 as a
 * SPEEDY read-modify-write (rd & ~mask) | (val & mask), so only bits 7:6
 * of the register can change; the voltage field (bits 5:0) is never sent.
 *   LDO35 L35CTRL 0x6c, vsel 0x2c (1.8 V): on 0x80 (mode 0b10, 4.9
 *         SEC_OPMODE_MIF) -> 0xac, off 0x00 -> 0x2c
 *   LDO43 L43CTRL 0x74, vsel 0x30 (3.0 V): on 0xc0 (mode 0b11, always on)
 *         -> 0xf0, off 0x00 -> 0x30
 * Before sending, the transport reads the PMIC and also refuses:
 *   - LDO35 on unless 0x6c reads vsel 0x2c with mode 0b00 or 0b10;
 *   - LDO43 on unless 0x6c reads exactly 0xac (LDO35 on) and 0x74 reads
 *     vsel 0x30 with mode 0b00 or 0b11;
 *   - LDO35 off unless 0x74 reads mode 0b00 (LDO43 goes off first);
 *   - the third and later "on" request per rail and boot.
 */
#define ACPM_MFD_TOUCH_RAIL_MASK	0xc0
#define ACPM_MFD_TOUCH_VSEL_MASK	0x3f
#define ACPM_MFD_TOUCH_L35_REG		0x6c
#define ACPM_MFD_TOUCH_L35_VSEL		0x2c
#define ACPM_MFD_TOUCH_L35_ON		0x80
#define ACPM_MFD_TOUCH_L43_REG		0x74
#define ACPM_MFD_TOUCH_L43_VSEL		0x30
#define ACPM_MFD_TOUCH_L43_ON		0xc0
#define ACPM_MFD_TOUCH_RAIL_OFF		0x00
#define ACPM_MFD_TOUCH_MAX_ON		2

/*
 * Write policy of the ACPM PMIC transport. Until a policy is sealed every
 * write/bulk-write/update request is refused with -EPERM. A sealed policy
 * can only open registers 0..31 of the RTC slave (type 2); writes to TOP,
 * PMIC (regulators, IRQ masks, power control) and any other type are always
 * refused. A write to @update_reg must not set any bit in @update_forbidden.
 * FUNC_UPDATE is refused, except, if @g3d_rail is set, the two exact G3D
 * rail requests above (type 3, reg 0x2a, mask 0xc0, value 0xc0 or 0x00),
 * and, if @touch_rails is set, the four exact touch rail requests above
 * with their read-back preconditions. FUNC_WRITE/FUNC_BULK_WRITE to the
 * PMIC stay refused in every case. The policy can be sealed once per boot.
 */
struct acpm_mfd_write_policy {
	u32 rtc_reg_mask;
	u8 update_reg;
	u8 update_forbidden;
	bool g3d_rail;
	bool touch_rails;
};

#ifdef CONFIG_EXYNOS_ACPM
bool exynos_acpm_mfd_is_ready(void);
int exynos_acpm_mfd_seal_write_policy(const struct acpm_mfd_write_policy *policy);
int exynos_acpm_mfd_touch_rails_policy(void);
extern int exynos_acpm_read_reg(u16 type, u8 reg, u8 *dest);
extern int exynos_acpm_bulk_read(u16 type, u8 reg, int count, u8 *buf);
extern int exynos_acpm_write_reg(u16 type, u8 reg, u8 value);
extern int exynos_acpm_bulk_write(u16 type, u8 reg, int count, u8 *buf);
extern int exynos_acpm_update_reg(u16 type, u8 reg, u8 value, u8 mask);
#else
static inline bool exynos_acpm_mfd_is_ready(void) { return false; }
static inline int
exynos_acpm_mfd_seal_write_policy(const struct acpm_mfd_write_policy *policy)
{
	return -ENODEV;
}
static inline int exynos_acpm_mfd_touch_rails_policy(void)
{
	return -ENODEV;
}
static inline int exynos_acpm_read_reg(u16 type, u8 reg, u8 *dest)
{
	return -ENODEV;
}
static inline int exynos_acpm_bulk_read(u16 type, u8 reg, int count, u8 *buf)
{
	return -ENODEV;
}
static inline int exynos_acpm_write_reg(u16 type, u8 reg, u8 value)
{
	return -ENODEV;
}
static inline int exynos_acpm_bulk_write(u16 type, u8 reg, int count, u8 *buf)
{
	return -ENODEV;
}
static inline int exynos_acpm_update_reg(u16 type, u8 reg, u8 value, u8 mask)
{
	return -ENODEV;
}
#endif
#endif /* __ACPM_MFD_H__ */
