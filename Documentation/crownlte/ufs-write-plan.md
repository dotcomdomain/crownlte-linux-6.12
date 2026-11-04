# crownlte UFS: from the read-only guard to a safe read-write /data

Status: this is analysis and design only. No code in this document writes to UFS. The 6.12 kernel still runs with `CONFIG_SCSI_UFS_CROWNLTE_DIAGNOSTIC_RO=y` plus DT `crownlte,read-only-diagnostic`.

## 0. Where we are

The guard is `drivers/ufs/core/ufshcd-readonly.h`, called from `ufshcd_queuecommand()` (`drivers/ufs/core/ufshcd.c:3058`).

**What it covers.** It allows only:

- TEST UNIT READY, REQUEST SENSE, INQUIRY, MODE SENSE(6/10)
- READ(6/10/12/16), READ CAPACITY(10/16), REPORT LUNS
- START STOP UNIT with start or active power condition only

It rejects any data-out transfer.

The guard is at the last SCSI hop, so it covers every path that ends in a SCSI command: the filesystem, raw `/dev/sdX`, `BLKDISCARD`/`BLKZEROOUT`, and SG_IO through `/dev/bsg/*`. `CONFIG_BLK_DEV_BSG=y`; `CONFIG_CHR_DEV_SG` and `CONFIG_SCSI_UFS_BSG` are off.

**What it does not cover.**

- UPIU query requests from ufshcd itself:
  - `fDeviceInit` SET_FLAG (ufshcd.c:4862)
  - auto-BKOPS flag and attribute writes (5994ff)
  - WriteBooster flags (6236ff; the device is UFS 2.1, so WB is absent)
  - `bRefClkFreq` WRITE_ATTR in `ufshcd_set_dev_ref_clk`, persistent, only written if it differs
  - `SECONDS_PASSED` (UFS 4.0 only)

  None of these touch media, but `bRefClkFreq` and the forbidden list in 2.3 are persistent device configuration.
- Device-internal activity such as background operations and wear levelling. This is not under host control and is harmless.

**FMP state.** The staged adapter is `drivers/ufs/host/ufs-exynos9810-fmp.h`, used through `.fill_crypto_prdt`.

- It handles standard 64-byte AES-256-XTS keys, a 4096-byte data unit and 64-bit DUNs.
- Each PRDT entry gets `file_iv[1] = cpu_to_be64(dun)` and both key halves word-reversed big-endian.
- Every segment must be exactly 4096 bytes (`ufs_max_seg_size = 4096`, `ufs-exynos9810.c:589`).
- **Hardware operation has never been verified.**

## 1. Partition map (the authority for every rule below)

Source: `C:\Users\Patrick\Downloads\Codex Projects\kernel-upgrade\backups\offline-20261011T0116\{block-topology.txt,partition-layout.txt}` (by-name links plus start and size, 4.9, 2026-10-11) and `baseline-live/{partitions.txt,fstab.txt,mounts.txt}`.

UFS logical blocks are 4 KiB, so SCSI LBA = 512-byte sector / 8.

LUN0 (`sda`) has 31,236,096 blocks of 4 KiB. The protective MBR and primary GPT are at LBA 0-1023. The tail after USERDATA (LBA 31,234,176-31,236,095) holds the backup GPT.

| by-name | dev | start LBA | blocks | size | 4.9 use | class |
|---|---|---|---|---|---|---|
| (GPT head) | sda | 0 | 1024 | 4 MiB | GPT | **H** |
| BOTA0 | sda1 | 1024 | 1024 | 4 MiB | bootloader OTA | **H** |
| BOTA1 | sda2 | 2048 | 2048 | 8 MiB | bootloader OTA | **H** |
| EFS | sda3 | 4096 | 5120 | 20 MiB | /efs rw (IMEI, calibration, keystore auth token) | **H** (TEE plan only, explicit user go) |
| PARAM | sda4 | 9216 | 2048 | 8 MiB | sboot params (debug level, upload) | **H** |
| UP_PARAM | sda5 | 11264 | 2048 | 8 MiB | sboot images | **H** |
| KEYSTORAGE | sda6 | 13312 | 2 | 8 KiB | keymaster storage | **H** |
| BOTA2 | sda7 | 13314 | 2048 | 8 MiB | bootloader OTA | **H** |
| DTB | sda8 | 15362 | 2048 | 8 MiB | sboot DTB | **H** |
| DTBO | sda9 | 17410 | 2048 | 8 MiB | DTBO | **H** |
| BOOT | sda10 | 19458 | 14080 | 55 MiB | boot image | **H** (Odin or recovery only) |
| RECOVERY | sda11 | 33538 | 16638 | 65 MiB | recovery / trial images | **H** (Odin or 4.9 only) |
| RADIO | sda12 | 50176 | 12800 | 50 MiB | modem firmware | **H** |
| NAD_REFER | sda13 | 62976 | 256 | 1 MiB | Samsung NAD | **H** |
| PERSISTENT | sda14 | 63232 | 128 | 512 KiB | FRP (persistent_data_block) | **C** (late, backed up) |
| MISC | sda15 | 63360 | 256 | 1 MiB | bootloader message (BCB) | **C** (late; back up first) |
| STEADY | sda16 | 63616 | 1024 | 4 MiB | Samsung (vendor rc chowns it) | **H** |
| DQMDBG | sda17 | 64640 | 4096 | 16 MiB | Samsung debug | **H** |
| SYSTEM | sda18 | 68736 | 1205760 | 4710 MiB | / ro | **H** |
| VENDOR | sda19 | 1274496 | 172800 | 675 MiB | /vendor ro | **H** |
| ODM | sda20 | 1447296 | 221696 | 866 MiB | /odm ro | **H** |
| CACHE | sda21 | 1668992 | 128000 | 500 MiB | /cache ext4 rw | **A1** (first write target) |
| HIDDEN | sda22 | 1796992 | 2560 | 10 MiB | Samsung preload | **H** |
| OMR | sda23 | 1799552 | 12800 | 50 MiB | Samsung OMR | **H** |
| CP_DEBUG | sda24 | 1812352 | 1280 | 5 MiB | modem dump | **H** |
| USERDATA | sda25 | 1813632 | 29420544 | 114924 MiB | /data f2fs, FBE v2 | **A2** (after KAT and TEE) |
| (GPT tail) | sda | 31234176 | 1920 | 7.5 MiB | backup GPT | **H** |
| — | sdb | 0 | 1024 | 4 MiB | boot LUN: sboot / BL31 / APM / EL3 (sha256 09d0dd96…) | **H** |
| — | sdc | 0 | 1024 | 4 MiB | second boot LUN (bb9f8df6…) | **H** |
| CPEFS | sdd1 (sdd) | 6 | 1536 | 6 MiB | CP EFS, mounted **ro** even on 4.9 | **H** (whole LUN) |
| — | sde | 0 | 2048 | 8 MiB | unpartitioned (fa343a62…) | **H** |
| RPMB / BOOT / DEVICE W-LUNs | — | — | — | — | RPMB = TEE replay-protected store | **H** (DEVICE W-LUN: START STOP UNIT power modes only, later) |

Classes:

- **H**: hard-blocked forever from 6.12.
- **C**: conditional, opened only when the Android service that needs it is in scope, and only after a fresh backup.
- **A1/A2**: allowlisted in stages.

The sd letter order (sdb..sde = LUN1..4) is a GUESS from scan order. The guard must key on the SCSI LUN number plus capacity, never on the letter.

**Backups.** Fresh raw backups exist for EFS, CPEFS, PERSISTENT, PARAM, UP_PARAM, STEADY, KEYSTORAGE, ODM, VENDOR and SYSTEM (`backups-20261103/partitions`, SHA256SUMS), and for BOOT, DTB, DTBO and RECOVERY (`backups-20261103/*.img`). sdb, sdc, sdd and sde are imaged in `offline-20261011T0116`, along with a full 2026-10-11 host copy of sda (sha256 `c58a10e2…`; userdata in it is stale).

**Missing before any write stage:** CACHE (W3) and MISC (W6).

## 2. Write guard design: policy "allowlist"

### 2.1 Two independent layers

**Block layer: partition read-only flags.** A late initcall (after the UFS SCSI scan) does the following:

1. Walk LUN0's gendisk partitions.
2. Match each GPT partition name (`bdev->bd_meta_info->volname`) and its start and length against a compiled-in copy of the table above.
3. Set every partition that is not allowlisted read-only (`bdev_set_flag(part, BD_READ_ONLY)`), and set the whole disks of LUN0..4 read-only (`set_disk_ro`).

Any write through the page cache, a raw open, discard, zeroout or blkdev write then fails with EPERM before any I/O.

If any name, start, size or LUN capacity differs from the table, the initcall leaves **everything** read-only and logs the first mismatch. A changed GPT means the table is stale.

**SCSI layer: LBA allowlist in `ufshcd_queuecommand`.** This replaces the opcode-only check.

The allowlist is held under RCU as an array of `{lun, first_lba, nr_blocks}` in 4 KiB units, built from the verified table.

Per command:

- **Writes on LUN0 only:**
  - WRITE(10) 0x2a and WRITE(16) 0x8a: `[lba, lba+len)` must lie inside one allowlisted range. Range checks use 64-bit math with overflow checks.
  - SYNCHRONIZE CACHE(10/16): LUN0 only. It has no data, but the guard still requires the LUN.
  - UNMAP 0x42: copy the parameter list out of `scsi_sglist()` and check every block descriptor against the ranges. One bad descriptor rejects the whole command. f2fs and ext4 mount with `discard`.
- **Always refused:** WRITE(6/12), WRITE SAME(10/16), WRITE AND VERIFY, FORMAT UNIT, WRITE BUFFER (its modes include microcode download), SECURITY PROTOCOL IN/OUT (RPMB), MODE SELECT(6/10), SEND DIAGNOSTIC, SANITIZE, PERSISTENT RESERVE OUT, and any unknown opcode. The read set stays as in the current guard.
- **LUNs 1-4:** read commands only.
- **W-LUNs:**
  - RPMB 0xC4 and BOOT 0xB0: TEST UNIT READY, REQUEST SENSE and INQUIRY only.
  - DEVICE 0xD0: those plus START STOP UNIT with a power condition, needed later for runtime PM and suspend.

A rejected command completes with DATA_PROTECT (sense key 7, ASC 0x27), exactly as today. Each rejection is logged rate-limited with LUN, opcode, LBA and length, and a counter is exported read-only in sysfs.

### 2.2 Policy selection

The policy is a DT string `crownlte,ufs-write-policy = "ro" | "allowlist"`. A missing property means "ro". The allowlist itself is in a `crownlte,ufs-write-allow` list of partition names, checked against the compiled table; names outside class A or C are refused at parse time.

The Kconfig `CONFIG_SCSI_UFS_CROWNLTE_WRITE_ALLOWLIST` builds the code. `CONFIG_SCSI_UFS_CROWNLTE_DIAGNOSTIC_RO=y` keeps forcing "ro" whatever the DT says.

### 2.3 Query (UPIU) gate in `ufshcd_query_*` / `ufshcd_exec_raw_upiu_cmd`

| Request | Rule |
|---|---|
| READ_DESC, READ_ATTR, READ_FLAG | Always allowed |
| SET_FLAG fDeviceInit | Allowed only during `ufshcd_complete_dev_init` |
| SET/CLEAR fBackgroundOpsEn | Allowed (later; ufshcd auto-BKOPS) |
| WRITE_ATTR bRefClkFreq | Allowed only if the read value already equals the target |
| WRITE_DESC (any: Configuration descriptor = LUN re-provisioning) | **Forbidden forever** |
| fPermanentWPEn, fPowerOnWPEn, fPurgeEnable | **Forbidden forever** |
| bBootLunEn, bConfigDescrLock | **Forbidden forever** |
| Any other write | **Forbidden forever** |

`CONFIG_SCSI_UFS_BSG` stays off, so there is no raw UPIU from userspace.

### 2.4 Userspace and config

- Set `CONFIG_BLK_DEV_WRITE_MOUNTED=n` (currently y).
- No `/dev/block/by-name` writes from init rc for class H partitions. The vendor rc only chowns STEADY and PERSISTENT.
- Watch `persistent_data_block` and `misc` writers (recovery commands, FRP). Until stage W6 they get EPERM. Lineage tolerates that; FRP just reports an error.

## 3. FMP known-answer test: prove 6.12 FMP output equals 4.9's, without 6.12 writing UFS

### Background

fstab: `fileencryption=aes-256-xts:aes-256-cts:v2+inlinecrypt_optimized`, with `inlinecrypt` on the /data mount.

- `inlinecrypt_optimized` is `FSCRYPT_POLICY_FLAG_IV_INO_LBLK_64`.
- Contents use one per-mode key per master key and filesystem: HKDF-SHA512 of the master key with context `HKDF_CONTEXT_IV_INO_LBLK_64_KEY`, info `mode_num` followed by the 16-byte f2fs UUID (`fs/crypto/keysetup.c`, `hkdf.c`).
- DUN = `ino << 32 | lblk` (64 bit, `fscrypt_generate_dun`). The data unit is 4096 (the fs block size).
- XTS tweak = DUN as a 16-byte little-endian integer.
- Filenames (CTS) are software and never touch FMP.

Keys are standard, not wrapped: vold unwraps them with keymaster and hands the raw 64-byte master key to the kernel. **A test master key added from userspace takes exactly the same kernel path as the real key.**

### Rejected approaches

- **"Encrypt a RAM block device or loop through FMP."** Not possible. FMP is inline in the UFSHCI DMA engine and only sees UFS transfers. blk-crypto on loop, zram or ram disks uses `blk-crypto-fallback` (software), which proves nothing about FMP.
- **WRITE BUFFER / READ BUFFER echo through the device buffer.** It would exercise FMP's encrypt direction without media writes, but it means opening an opcode whose other modes download firmware, and UFS 2.1 support for data or echo modes is unknown. Rejected.
- **Reading real /data with the user's key.** That needs the Trustonic keymaster, which is a separate plan with irreversible steps (`trustonic-tee.md`). Not needed for the KAT.

### Proposed KAT

K1 to K3 involve no 6.12 writes at all. K4 is the first and only 6.12 write, confined to a 1 MiB window.

**K1. Make reference ciphertext on 4.9 (the phone's normal kernel).**

All writes here are ordinary 4.9 /data writes. As root (su):

1. Create `/data/fmp-kat`. The /data root has no policy, and children of an unencrypted directory may get a new one.
2. `fscryptctl add_key /data < kat.key` with a fixed 64-byte test key (recorded in the repo, not secret). Then `set_policy --contents=AES_256_XTS --filenames=AES_256_CTS --iv-ino-lblk-64 <id> /data/fmp-kat`. fscryptctl must be a static aarch64 binary.
3. `fallocate` and pin the 1 MiB test file (`f2fs_io pinfile set`), so f2fs GC never moves it. Write 256 blocks of a deterministic pattern P (block i = SHA-256 counter stream seeded with i), then `fsync`.
4. Record the inode number (`stat -c %i`), `filefrag -v` physical blocks and the f2fs UUID (superblock).
5. Remove the key (`fscryptctl remove_key`) and `echo 3 > drop_caches`.
6. `dd` the physical blocks raw from `/dev/block/sda25` to get C49. Re-read the file through the filesystem with the key added again: it must equal P.

**K2. Check C49 offline on gekko.** A software reference (Python; HKDF and AES-XTS from the `cryptography` package, port of `fs/crypto` derivation) computes `XTS(K_mode, DUN = ino<<32|i, P_i)`.

- Expected: C49 == reference. This confirms that 4.9 FMP output is plain AES-256-XTS under fscrypt's IV_INO_LBLK_64 rules.
- If it differs, C49 is still ground truth, but the 4.9 layout must be understood before going further (for example, Samsung-specific IV handling). **STOP.**

**K3. Read-only on 6.12.** Use a RECOVERY diag boot with the guard still "ro", from initramfs or ROM-in-RAM userspace:

- **K3a.** Raw `O_DIRECT` read of the recorded blocks of `/dev/sda25`. It must equal C49 byte for byte, which checks the LBA and partition math of the 6.12 read path.
- **K3b.** `mount -t f2fs -o ro,norecovery /dev/sda25` **without** `inlinecrypt`, then add the test key and read the file. It must equal P. This proves 6.12 fscrypt, f2fs and key derivation in software, independent of FMP. Any write f2fs attempts is rejected by the guard (DATA_PROTECT) and only stops its in-memory checkpointing.
- **K3c.** Remount with `inlinecrypt` and read again. It must equal P.

  For K3c, build with `CONFIG_BLK_INLINE_ENCRYPTION_FALLBACK=n` so an unsupported crypto config fails instead of silently going to software. Add a read-only counter of crypto PRDTs filled by `exynos9810_fmp_fill_prdt`, which must increase.

  XTS decryption gives P only with the right key halves, key word and byte order, tweak (DUN) encoding and data-unit mapping. So K3c proves that 6.12's PRDT encoding equals what 4.9 used to encrypt. The encrypt direction uses the same `fill_prdt` fields; only the hardware's write datapath stays untested.
- **K3d. Negative control.** The raw read (K3a) must not equal P, and the PRDT counter must be non-zero.

**K4. First 6.12 write (stage W3).** This tests the encrypt direction inside a 1 MiB CACHE window.

1. On 4.9: `fallocate -l 1M /cache/fmp-kat.bin`, write zeros, `sync`, and record extents. ext4 does not move blocks of a closed file.
2. Boot 6.12 with policy "allowlist" and `crownlte,ufs-write-allow` limited to that LBA window. This is a K4-only DT that names the window instead of CACHE.
3. A debug-only, DT-gated in-kernel test opens `/dev/sda21` and does:
   1. Write 256 blocks P through a bio crypto context (test key, DUN base D).
   2. Raw-read them back, without a context, and compare against software `xts(aes)` from the kernel crypto API, computed in the same test.
   3. Read again with the context; the result must equal P.
4. On 4.9: `/cache/fmp-kat.bin` must equal the expected ciphertext. Then delete it.

## 4. Stages

| Stage | 6.12 writes | Content | Pass criterion |
|---|---|---|---|
| W0 (now) | none | RO opcode guard | — |
| W1 | none | Allowlist code with policy "ro" | Table verification log matches section 1 on hardware; all reads unchanged; zero write commands issued |
| W2 | none | FMP KAT K1-K3 | C49 == software reference; K3a/b/c PASS; PRDT counter > 0 |
| W3 | 1 MiB CACHE window | K4 | Ciphertext on media == software, 4.9 agrees; nothing outside the window changed (CACHE before/after hash outside window) |
| W4 | CACHE | /cache ext4 rw on 6.12 | 4.9 `e2fsck -n` clean afterwards |
| W5 | USERDATA | /data rw | Needs TEE plan T2-T4 (keymaster) and the explicit user go from that plan |
| W6 | PERSISTENT, MISC | Android services (FRP, BCB) | Backups first; only if needed |

### First /data write (W5)

Mount f2fs with `checkpoint=disable` for the first rw boots: f2fs writes out of place and does not commit a checkpoint. A later 4.9 mount rolls back to the last checkpoint, so the 4.9-visible /data state survives a bad 6.12 session.

This does **not** protect keystore or keymaster state in the TEE. A KEY UPGRADE is irreversible: see the TEE hard rules (`trustonic-tee.md`).

## 5. Risks

1. **An LBA math error in the guard** is the main brick risk if it ever lets a write through to class H. Mitigations:
   - two layers (partition read-only flags and the SCSI LBA check);
   - the compiled table is cross-checked against the GPT, and any mismatch means everything stays read-only;
   - W1 validates the classification on hardware before any write is allowed.
2. **Partition read-only flags can be cleared by root** (`BLKROSET`). The SCSI layer is the real barrier, and the read-only flags only catch mistakes early. The SCSI layer cannot be changed from userspace.
3. **UNMAP parsing errors** could unmap class H blocks: treat a parse failure as reject. Until W4, mount with `nodiscard` to keep UNMAP out of the picture.
4. **f2fs GC moves the K1 file** if it is not pinned. Pin it, and re-check FIEMAP on 4.9 before each 6.12 run.
5. **Hidden fallback** (`BLK_INLINE_ENCRYPTION_FALLBACK=y`) would make K3c pass in software. The KAT build turns it off and checks the PRDT counter.
6. **Persistent device writes outside the SCSI path** (query requests in section 2.3). These are gated separately, and WRITE_DESC is never allowed.
