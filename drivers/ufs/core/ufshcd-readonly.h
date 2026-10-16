/* SPDX-License-Identifier: GPL-2.0 */
#ifndef UFSHCD_READONLY_DIAGNOSTIC_H
#define UFSHCD_READONLY_DIAGNOSTIC_H

/* Pure command policy shared with the offline fault fixture. Unknowns fail. */
static inline int ufshcd_read_only_command(const unsigned char *cdb,
					 unsigned int length, int data_out)
{
	if (!cdb || !length || length > 16 || data_out || cdb[length - 1])
		return 0;
	switch (cdb[0]) {
	case 0x00: /* TEST UNIT READY */
	case 0x03: /* REQUEST SENSE */
	case 0x08: /* READ(6) */
	case 0x12: /* INQUIRY */
	case 0x1a: /* MODE SENSE(6) */
		return length == 6;
	case 0x25: /* READ CAPACITY(10) */
	case 0x28: /* READ(10) */
	case 0x5a: /* MODE SENSE(10) */
		return length == 10;
	case 0xa0: /* REPORT LUNS */
	case 0xa8: /* READ(12) */
		return length == 12;
	case 0x88: /* READ(16) */
		return length == 16;
	case 0x9e: /* SERVICE ACTION IN(16): READ CAPACITY only */
		return length == 16 && cdb[1] == 0x10;
	case 0x1b: /* UFS/standard device activation only, no stop/eject. */
		return length == 6 && !(cdb[1] & ~1U) && !cdb[2] && !cdb[3] &&
			(cdb[4] == 1 || cdb[4] == 0x10);
	default:
		return 0;
	}
}
#endif
