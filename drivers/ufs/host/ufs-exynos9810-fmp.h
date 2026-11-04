/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Exynos9810 standard-key FMP adapter for ACK 6.12.
 * Descriptor layout and SMC calls follow the running crownlte 4.9 driver.
 * Encoding follows the FMP implementation in drivers/ufs/host/ufs-exynos.c.
 * This is a staged port. Hardware operation has not been verified.
 */
#ifndef _UFS_EXYNOS9810_FMP_H
#define _UFS_EXYNOS9810_FMP_H

#include <crypto/aes.h>
#include <linux/arm-smccc.h>
#include <linux/blk-crypto-profile.h>
#include <linux/dma-mapping.h>
#include <linux/unaligned.h>
#include <ufs/ufshcd.h>

#define EXYNOS9810_FMP_SECURITY 0xc2001810
#define EXYNOS9810_FMP_SMU 0xc2001850
#define EXYNOS9810_FMP_SMU_RESUME 0xc2001860
#define EXYNOS9810_FMP_DATA_UNIT 4096

struct exynos9810_fmp_sg_entry {
	struct ufshcd_sg_entry base;
	struct_group(fmp,
		__be64 file_iv[2];
		__be64 file_enckey[4];
		__be64 file_twkey[4];
		__be64 disk_iv[2];
		__be64 reserved[2];
	);
};

#ifdef CONFIG_SCSI_UFS_CRYPTO
/* No persistent disk-key SMCs are used for this ROM's fscrypt file keys. */
static int exynos9810_fmp_secure_config(struct ufs_hba *hba, bool resume)
{
	struct arm_smccc_res res;

	arm_smccc_smc(EXYNOS9810_FMP_SECURITY, 0, 0, 3,
		      0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(hba->dev, "FMP descriptor3 configuration failed: %ld\n", res.a0);
		return -EIO;
	}
	if (resume)
		arm_smccc_smc(EXYNOS9810_FMP_SMU_RESUME, 0, 0, 0,
			      0, 0, 0, 0, &res);
	else
		arm_smccc_smc(EXYNOS9810_FMP_SMU, 0, 0, 0,
			      0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(hba->dev, "FMP SMU %s failed: %ld\n",
			resume ? "resume" : "init", res.a0);
		return -EIO;
	}
	return 0;
}

static int exynos9810_fmp_init(struct ufs_hba *hba)
{
	struct blk_crypto_profile *profile = &hba->crypto_profile;
	int err;

	BUILD_BUG_ON(sizeof(struct exynos9810_fmp_sg_entry) != 128);
	err = exynos9810_fmp_secure_config(hba, false);
	if (err)
		return err;
	ufshcd_set_sg_entry_size(hba, sizeof(struct exynos9810_fmp_sg_entry));
	err = devm_blk_crypto_profile_init(hba->dev, profile, 0);
	if (err)
		return err;
	/* Match the current vendor driver: standard keys and 64-bit DUNs. */
	profile->max_dun_bytes_supported = 8;
	profile->key_types_supported = BLK_CRYPTO_KEY_TYPE_STANDARD;
	profile->modes_supported[BLK_ENCRYPTION_MODE_AES_256_XTS] =
		EXYNOS9810_FMP_DATA_UNIT;
	profile->dev = hba->dev;
	hba->caps |= UFSHCD_CAP_CRYPTO;
	hba->quirks |= UFSHCD_QUIRK_CUSTOM_CRYPTO_PROFILE |
		       UFSHCD_QUIRK_BROKEN_CRYPTO_ENABLE |
		       UFSHCD_QUIRK_KEYS_IN_PRDT;
	/* Do not let the DMA layer combine adjacent 4K data units. */
	hba->host->max_segment_size = EXYNOS9810_FMP_DATA_UNIT;
	hba->host->dma_boundary = EXYNOS9810_FMP_DATA_UNIT - 1;
	dma_set_max_seg_size(hba->dev, EXYNOS9810_FMP_DATA_UNIT);
	return 0;
}

static int exynos9810_fmp_resume(struct ufs_hba *hba)
{
	return exynos9810_fmp_secure_config(hba, true);
}

static __be64 exynos9810_fmp_key_word(const u8 *key, unsigned int word)
{
	return cpu_to_be64(get_unaligned_le64(key + AES_KEYSIZE_256 -
					      (word + 1) * sizeof(u64)));
}

static int exynos9810_fmp_fill_prdt(struct ufs_hba *hba,
				 const struct bio_crypt_ctx *ctx,
				 void *table, unsigned int segments)
{
	struct exynos9810_fmp_sg_entry *prdt = table;
	const struct blk_crypto_key *key = ctx->bc_key;
	u64 dun = ctx->bc_dun[0];
	unsigned int i, j;

	if (!(hba->caps & UFSHCD_CAP_CRYPTO) ||
	    ufshcd_sg_entry_size(hba) != sizeof(*prdt))
		return -EIO;
	if (key->crypto_cfg.crypto_mode != BLK_ENCRYPTION_MODE_AES_256_XTS ||
	    key->crypto_cfg.key_type != BLK_CRYPTO_KEY_TYPE_STANDARD ||
	    key->crypto_cfg.data_unit_size != EXYNOS9810_FMP_DATA_UNIT ||
	    key->size != 2 * AES_KEYSIZE_256 || ctx->bc_dun[1])
		return -EINVAL;
	/* Validate the entire request before copying any keys into DMA memory. */
	for (i = 0; i < segments; i++)
		if (prdt[i].base.size != cpu_to_le32(EXYNOS9810_FMP_DATA_UNIT - 1))
			return -EINVAL;
	if (segments && dun > U64_MAX - (segments - 1))
		return -EOVERFLOW;
	if (!memcmp(key->raw, key->raw + AES_KEYSIZE_256, AES_KEYSIZE_256))
		return -EINVAL;
	for (i = 0; i < segments; i++, dun++) {
		/* Zero unused fields; no disk encryption, no stale keys or IVs. */
		memset(&prdt[i].fmp, 0, sizeof(prdt[i].fmp));
		prdt[i].base.size |= cpu_to_le32(BIT(26) | (2U << 28));
		prdt[i].file_iv[0] = 0;
		prdt[i].file_iv[1] = cpu_to_be64(dun);
		for (j = 0; j < AES_KEYSIZE_256 / sizeof(u64); j++) {
			prdt[i].file_enckey[j] = exynos9810_fmp_key_word(key->raw, j);
			prdt[i].file_twkey[j] = exynos9810_fmp_key_word(
				key->raw + AES_KEYSIZE_256, j);
		}
	}
	/* UFSHCD_QUIRK_KEYS_IN_PRDT makes ufshcd-core wipe after completion. */
	return 0;
}
#else
static int exynos9810_fmp_init(struct ufs_hba *hba)
{
	/* Do not silently build the hardware-FMP image without blk-crypto. */
	return -EOPNOTSUPP;
}
static int exynos9810_fmp_resume(struct ufs_hba *hba)
{
	return -EOPNOTSUPP;
}
#define exynos9810_fmp_fill_prdt NULL
#endif
#endif
