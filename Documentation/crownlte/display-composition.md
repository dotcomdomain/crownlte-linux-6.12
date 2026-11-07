# crownlte DPU composition: 4.9 parity for the vendor HWC

Branch `crownlte-dpu-scale`, based on `crownlte-panel-s2dos05-diag` 233829a14 (the newest
ROM-in-RAM display kernel). `drivers/video/fbdev/exynos/dpu_9810/` and
`include/uapi/video/exynos9810_decon.h` are identical in `crownlte-integration-4` b4f756bed
(`git diff` is empty), so the driver commit applies there unchanged. The DTS commit needs the
v23a source, which only this base has.

**Status: build-only, untested on hardware.**

## 1. Problem

ROM-in-RAM v23a run4 (2026-11-06): Settings rendered, Home went black. Every S3CFB_WIN_CONFIG
with the launcher wallpaper failed:

```
win0 rejected: scaling not supported (-22) [state 2 fmt 2 idma 4 comp 0 rot 0 prot 0 blend 1
  pa 255 hdr 0 ... src 1132x2328@57,116 f 2560x2560 dst 1440x2960@0,0]
```

That is an RGBA_8888 crop of a 2560x2560 buffer on VGF0 (HWC `DPP_VGFS0`), scaled up 1.27x.
That is within 4.9 limits. The HWC does not fall back to GPU composition when the ioctl fails, so
the last frame stayed up. The HWC dump shows that every window has `dst_f_w = 1440,
dst_f_h = 2960` and RGB layers have `eq_mode 0x41` (BT.709, full range).

## 2. What the HWC reads from the kernel

| Interface | 4.9 | 6.12 before | 6.12 now |
|---|---|---|---|
| `'F' 261` EXYNOS_DISP_RESTRICTIONS (3336 B, the HWC calls it first) | not implemented: -ENOTTY | -ENOTTY | -ENOTTY (unchanged) |
| `'F' 400/401` S3CFB_GET_HDR_CAPABILITIES(_NUM) | panel ddi_info: HDR10 + HLG, 5400000 / 1200000 / 5 | none | from DT `samsung,hdr-*`. v23a-dpu DTB: the 4.9 values. None without them |
| `'F' 500` EXYNOS_DPU_GET_ACLK | not implemented | -ENOTTY | -ENOTTY |
| `'F' 260` EXYNOS_DISP_INFO | psr 2, 9810, mres | same | same |

The HWC (`/vendor/lib64/libexynosdisplay.so`, strings `EXYNOS_DISP_RESTRICTIONS ioctl failed`,
`makeDPURestrictions`, `dpp_restriction`) uses its built-in exynos9810 table when 261 fails, on 4.9
too. Keeping -ENOTTY therefore gives the HWC exactly the 4.9 view. Its channel names match the 4.9
DPP features: G0/G1 RGB, VG0/VG1 + YUV, VGFS0 (VGF0) + scaling/AFBC, VGRFS0 (VGF1) + rotation/HDR.
I did not decompile the table itself.

## 3. Rejection table, before and after

Before, every row failed the whole WIN_CONFIG. 4.9 has two levels.
`decon_check_limitation()` and the buffer import run at ioctl time, and a failure there fails the
ioctl. `dpp_check_limitation()` runs when the frame is programmed, and a failure there only
disables that window for that frame (`decon_set_dpp_config()`). The driver now does the same.

| Check | Before | 4.9 | Now |
|---|---|---|---|
| format >= DECON_PIXEL_FORMAT_MAX | reject | reject | reject |
| blending > COVERAGE | reject | reject | reject |
| plane alpha outside 0..255 | silently "no plane alpha" | reject | reject |
| dst w/h 0, x/y < 0, dst.w < 16, outside dst.f_w/f_h or the panel | reject (no f_w, no w>=16) | reject | reject |
| idma_type > VGF1 | reject | reject | reject |
| colour window outside the panel | reject | unchecked | reject (kept: no window past the panel) |
| unknown window state | reject | reject | reject |
| dma-buf import, size, contiguity, acquire fence | reject | reject | reject; every plane size-checked (below) |
| **scaling** (src != dst) | **reject on all channels** | VGF0/VGF1, 1/2x..8x | **implemented**; outside VGF or ratio: window disabled |
| rotation / flip | reject | flips + 180 all channels, 90/270 VGF1 | implemented; 90/270 outside VGF1: window disabled |
| YUV formats | reject | VG0/VG1/VGF0/VGF1 | implemented; on G0/G1: window disabled |
| RGB 10-bit (ARGB2101010 family) | reject | all channels | implemented |
| other formats (YUV 3-plane, 422, 5551/4444, BGR565) | reject | DPP refuses | window disabled |
| block area | reject | block mode | implemented (see deviations) |
| AFBC outside VGF | reject | DPP refuses | window disabled |
| AFBC + HDR / AFBC + rotation / AFBC + YUV420 | reject (HDR only) | DPP refuses | window disabled |
| HDR outside VGF1, unknown standard, HDR + rotation | accepted, not programmed | DPP refuses | window disabled |
| HDR on VGF1 | accepted, not programmed | EOTF/GM/TM LUTs | implemented |
| crop/size/alignment (img 16..4096, rot height 2160, scaled 16..4096, YUV even, crop inside buffer) | reject (no YUV/rot rules) | DPP refuses | window disabled |
| odd x for RGB565 | reject | no such check | removed |
| channel used by two windows | reject | later window disabled (`decon_reg_chmap_validate`) | later window disabled |
| protected content | reject | secure path (SMC) | window disabled (no secure DPU path here) |

A disabled window is logged once per rate-limit window:
`win%d disabled for this frame: <reason> (0) [...]`. It gets no release fence (-1), and the rest
of the frame goes out. Counters: `EXYNOS_DPU_DUMP` now prints
`dpp_drops scaled yuv rot hdr block`.

## 4. What was implemented (decon_hw.c, decon_drv.c)

- **Scaler**: 12.20 ratios `(src << 20) / dst`, with width and height swapped for 90/270. The 4.9
  coefficient sets are chosen by ratio (`h_coef_8t`/`v_coef_4t`, `dpp_coef.h`, already
  verbatim), and `DPP_SCALED_IMG_SIZE` is written on VGF. As on 4.9, the initial phase registers are not
  written.
- **Formats**: the full 4.9 `dpp_reg_set_format()` table. That covers RGB32, RGB565 (BGR565 for
  AFBC), the 2101010 family, NV12/NV21(M), NV12N, NV12N_10B, NV12M/NV21M P010 and S10B. It keeps the
  4.9 `dpu_translate_fmt_to_dpp()` NV12<->NV21 swap.
- **Planes**: two-plane formats import `fd_idma[1]`. NV12N chroma, the S10B and 8+2 2-bit planes,
  and the 2-bit strides are derived as in 4.9 `dpp_get_params()`. Every plane's dma-buf must hold
  what the DPP reads (`dpu_fmt_planes()`). 4.9 only checked plane 0 and relied on the SysMMU.
  Without the SysMMU, the IDMA would read physical memory past the buffer.
- **Bounce** (no SysMMU, the ROM-RAM default): both planes of a window go into one slot, with the
  chroma plane page-aligned after the luma plane.
- **CSC**: type and range from `eq_mode` on VG/VGF (4.9 `dpp_reg_set_csc_config()`).
- **Rotation**: `IDMA_IN_CON` rotation field, with the DPP image size swapped for 90/270.
- **Block area**: `IDMA_BLOCK_OFFSET/SIZE/EN` from the HWC's `block_area`.
- **HDR on VGF1**: `DPP_VGRF_HDR_CON` is written on every VGF1 frame, as on 4.9 (GM on unless
  DCI-P3). For ST2084 and HLG, the EOTF (1000/4000 nit, HLG), BT.2020->P3 gamut and 1/2.2 tone-map
  LUTs are loaded. The tables in `dpp_hdr_lut.h` were compared number for number with 4.9
  `hdr_lut.h`.
- **HDR capabilities**: from DT. `exynos9810-crownlte-romram-v23a-dpu.dts` sets the 4.9 panel
  values.
- **Plane alpha, blending, colour windows**: the code was already a port of 4.9
  `decon_reg_set_win_bnd_function()` and the winmap. Only the plane-alpha range check is new.

Not changed: fences, the latch timeout and stale-commit handling, the SysMMU identity map, IOVA
cache and fault handler, `dpu_pending_resolve()`, and all panel, DSIM, brightness and PMIC
code.

### Deviations from 4.9

1. Block mode is not used for AFBC windows, or for a block outside the crop. 4.9 disables the
   whole window for AFBC + block, and programs an out-of-range block as is. Block mode is only a
   fetch optimisation. 4.9's own kernel-side blocking search (`decon_set_win_blocking_mode()`,
   `CONFIG_DECON_BLOCKING_MODE`) is not ported, so the only cost is bandwidth.
2. An 8-bit two-plane format with `fd_idma[1] < 0` takes its chroma from `fd_idma[0] + f_w*f_h`.
   4.9 failed the import instead.
3. For one-plane formats, `IN_BASE_ADDR_C` mirrors Y. 4.9 wrote the user's `dpp_parm.addr[1]`.
4. Protected windows are disabled, because there is no secure DPU path.
5. Colour windows must lie inside the panel.
6. The DPP HDR-metadata path to DisplayPort (`decon_update_hdr_info()`) does not exist for DSI on
   4.9 either.

## 5. AFBC (`ro.vendor.ddk.set.afbc`)

4.9 has `CONFIG_EXYNOS_AFBC=y`, and the vendor build.prop has `ro.vendor.ddk.set.afbc=1`, so
the HWC client target is AFBC on VGF0. The 6.12 driver decodes AFBC on VGF0/VGF1 (4e8632cae: IN_CON
AFBC_EN, recovery, DMA common recovery number). ROM-RAM v14 set `afbc=0` in its overlay as an
A/B test: the panel was black and AFBC decode was the suspect. Later runs showed that the
darkness came before the kernel, from the panel state SBoot left (v15/v16 camera analysis, "the
kernel image does not determine it"). v12 had shown real AFBC payload and 4.9-identical
registers. AFBC was therefore never shown to be broken. It was also never shown on screen. This
change does not touch it. Suggested later A/B: drop the `afbc=0` overlay line once trial A below
passes.

## 6. Self-test (DT `samsung,decon-selftest`)

After the existing fb0 A/B flips, `dpu_st_compose()` sends three in-kernel commits through the
normal path: `dpu_build_commit()`, the DPP checks, `dpu_apply()` and latch. The buffers come from
decon-pool: a 720x1480 RGBX buffer, and a 720x720 NV12 frame in two dma-bufs (the `fd_idma[0]`
and `fd_idma[1]` path). Each step is held for 1.5 s. The test stops if userspace calls
WIN_CONFIG; it then also skips the fb0 restore.

| Step | Windows | Expect on screen |
|---|---|---|
| C1 | win0 VGF0 RGBX 720x1480 -> 1440x2960 (2x up) | full-screen red / green / blue / white quadrants, black grid (soft edges), white border, "SCALED 720x1480" |
| C2 | C1 + win1 VG0 NV12 720x720 at 360,1120, unscaled | the same, with a 720x720 square in the centre: red / green / blue / white vertical bars and a black horizontal band |
| C3 | win0 VGF1 RGBX -> 360x740 at 0,0 (1/2 down); win1 VGF0 NV12 -> 1440x1440 at 0,1520 (2x up); win2 colour 0xff00ff 720x740 at 720,0 | small quadrants top-left, magenta top-right, the bars full width below, black between y 740 and 1520 |

If red and blue bars are swapped, the NV12/NV21 order (the 4.9 swap) does not hold on this DPP.
If the bars look washed out or tinted green, the CSC range is wrong. Neither would affect RGB.

## 7. Artifacts (gekko)

| File | sha256 |
|---|---|
| `panel-bright-work/images/Image-romram-dpu-scale-263d938dc` | `8b8722ca8b4d10f10ad093a313d53b81899a4399c85a2387845f698c8e6c841e` |
| `panel-bright-work/dtb/exynos9810-crownlte-romram-v23a-dpu.dtb` | `fc7201b5655698cac5e98f0b6cbcc1239cb94de3ddee5a8320d73aa1ddf67f36` |
| `panel-bright-work/dtb/exynos9810-crownlte-romram-v23a-dpu-selftest.dtb` | `283c10a83f07efaee9d7e97fd668bb62cf2d9847dfed66fd6fb6c40bbe98047b` |
| `panel-bright-work/dtb/exynos9810-crownlte-romram-v23a.dtb` (unchanged; rebuilt here byte-identical) | `84bf4f034fa989b31c3077ad94376bf4219b3298ae78088ea1d8ca6181939cd7` |
| `panel-bright-work/ko-dpu-scale/config-romram-dpu-scale` (= the v23a build's .config) | `9e144fd87da63406f35debd27215a04d5ea00735b2f0724a1c45a79e6af30712` |

The kernel was built with clang 18 (`LLVM=-18`, -j3). `W=1` gives no warnings in `dpu_9810/`. The
decompiled v23a-dpu DTB differs from v23a only by the four `samsung,hdr-*` properties, and the
selftest DTB adds only the three `samsung,decon-selftest*` properties.

## 8. Trial plan

Pack both trials like v23a, with the same ramdisk and overlay, and replace the Image and DTB.

**Trial A, functional: Image + v23a-dpu.dtb.** Boot to Lineage, unlock, press Home, and open
a few apps.

Expected kernel lines:
```
exynos9810-decon 16030000.decon_f: HDR capabilities: 2 types (2 3 0 0), luminance max 5400000 avg 1200000 min 5
exynos9810-decon 16030000.decon_f: ioctl 0x40104790 (type 0x47 nr 144 size 16) first call by ...: 0
exynos9810-decon 16030000.decon_f: win0 bounce buffer 0: >= 26214400 bytes at 0x...   (the 2560x2560 wallpaper, first scattered frame)
```
After Home there should be **no** `win0 rejected: scaling not supported`. Any remaining
`rejected:` line is a 4.9 ioctl-level failure. `disabled for this frame:` lines name a DPP
limit; the rest of the frame is still shown. Failure signs: a black screen after Home, `latch`
timeouts (`shadow update did not latch`), `active IDMA base ... programmed` mismatches, or
SysMMU/SError messages right after the first scaled frame.

**Trial B, self-test: Image + v23a-dpu-selftest.dtb**, with a camera on the panel from boot to
~15 s. Expected:
```
decon selftest: start at 3.0xx s: FRAME_COUNT=... TE=... commits=1 fb_taken=1
decon selftest: flip 0..3 -> A/B: 0, fence after <=34000 us, FRAME_COUNT n (+1..3) ... OK
decon selftest: compose: RGBX 720x1480 at 0x..., NV12 720x720 Y at 0x... CbCr at 0x...
decon selftest: compose C1 scale up 2x: ...: commit 0, fence after <=34000 us, FRAME_COUNT a -> a+1..3
decon selftest:   win0 dpp4: IDMA base active Y .. C .., want .. OK; DPP ratio h 0x80000 v 0x80000 scaled 0xb9005a0, want 0x80000 0x80000 0xb9005a0 OK; IDMA format 5
decon selftest: compose C1 held 1500 ms: FRAME_COUNT +~90 TE +~90, latch timeouts 0, DPP drops 0
decon selftest:   win1 dpp2: ... OK; DPP ratio h 0x100000 v 0x100000 scaled 0, want 0x100000 0x100000 0 OK; IDMA format 25      (C2, NV12 -> YVU420_2P)
decon selftest:   win0 dpp5: ... DPP ratio h 0x200000 v 0x200000 scaled 0x2e40168 ... OK; IDMA format 5                     (C3, 1/2 down)
decon selftest:   win1 dpp4: ... DPP ratio h 0x80000 v 0x80000 scaled 0x5a005a0 ... OK; IDMA format 25                      (C3, NV12 2x up)
decon selftest:   win2 colour 0xff00ff: DP0 active 0x... (window on, map colour on)
decon selftest: compose PASS: 3/3 steps, IDMA bases right 5/5, DPP ratios right 5/5, min FRAME_COUNT advance ~90 per 1500 ms hold, latch timeouts +0, DPP drops +0
decon selftest: restored fb0: 0, active IDMA base 0xcc000000 OK
decon selftest: PASS: 4/4 flips latched, ...
```
It should be done by ~9.5 s, before HWC (~11.6 s in run4). If HWC starts first, the line ends
with `(stopped: HWC started)` and fb0 is not restored; the next HWC frame replaces the picture.
The camera has to confirm the pictures in section 6. The log only proves that the registers
latched.

## 9. Untested and risks

- Nothing here has run on hardware: not the scaler, not YUV, rotation, block mode, or the HDR
  LUTs. VG0 and VGF1 may never have been programmed under 6.12 (the run4 HWC used idma 0, 1 and 4); first use runs the 4.9
  `dpp_reg_init` subset, as on the other channels.
- The HWC's built-in restriction table was not decompiled. Parity rests on the 4.9 kernel limits
  being what this HWC worked against on 4.9.
- Bounce cost: without the SysMMU, the wallpaper buffer (2560x2560x4 = 26 MB) is copied whole on
  every new frame. That is ~17 ms at the v12 copy rate, ~30 ms together with a full-screen layer,
  so launcher animations may run at about 30 fps. Two slots per window: wallpaper 52 MB + launcher
  35 MB + bars, within the 192 MiB decon-pool. The real fix is the DPU SysMMU (`iommus`, see
  display-iommu.md and section 10). Hardware confirmed the risk (section 10.1): the pool
  fragmented and win0 got no slot.
- HDR10/HLG are now advertised (v23a-dpu DTB). HDR video then reaches VGF1 with the 4.9 LUTs,
  untested. To advertise none, use plain v23a.dtb: the driver change works without it.
- The window-level checks are stricter than before in two places, both required by 4.9: dst
  inside `dst.f_w/f_h`, and dst width >= 16. All run4 dumps have `dst_f_w 1440 / dst_f_h 2960`.
- Disabled windows have no release fence (-1).
- Panel, brightness and PMIC paths: not touched.

## 10. DPU SysMMU scanout (branch `crownlte-dpu-scale-iommu`)

**Status: build-only, untested on hardware.** Base: `crownlte-dpu-scale` 4ac2e1d38.

### 10.1 Problem (ROM-in-RAM adb-dpu run, 2026-11-06)

Image dpu-scale 4ac2e1d38 + DTB v23a-dpu. SurfaceFlinger composed the home screen (`adb
screencap` showed wallpaper, clock and dock), but the camera showed the panel frozen on the
setup-wizard frame. The kernel log repeated:

```
exynos9810-decon 16030000.decon_f: win0 bounce buffer 0: cannot allocate 26628096 bytes below 4 GiB from decon-pool
cma: decon-pool: alloc failed, req-size: 6501 pages
```

The v23a-dpu DTB has no `iommus` on decon_f, so every gralloc buffer (system dma-heap,
scattered) is copied into a contiguous decon-pool slot pair per window. With six windows and
slots of 17-26 MB, the 208 MB CMA pool had ~97 MB free but no contiguous 26 MB run. Every
frame with the scaled wallpaper on win0 therefore failed (`-ENOMEM`, release fence with an
error, nothing programmed).

### 10.2 What the base already had

`crownlte-dpu-scale` already contains the reviewed SysMMU series of `crownlte-followups`, through
the `crownlte-integration-3` merge d4af69833:

- 0b5ae748f: exynos-iommu 9810 mode;
- 2baaea489: DT nodes;
- cf8abdf68: IOVA cache;
- 7f0bfdbe5..f8cd991c7: fault recovery and quiesce;
- docs up to 5b6ef27b4.

That series passed Sol R4 (APPROVE). It already ports the `crownlte-dpu-iommu` work (d41526fcb)
to the corrected DT naming: `dpp-dma` = IDMA block 0x16071000 and `dpp` = DPP block 0x16021000,
as in v23a. Trial v22b (2026-11-06 01:15, ROM-in-RAM with HWC) ran it: boot_completed and no
SysMMU faults. Nothing needed another cherry-pick. `crownlte-dpu-iommu` itself sits on the old
display-s2 base and is superseded. The dpu-scale composition (scaling, YUV, two planes) was
written on top of the same import path, so it needs no separate IOMMU path: each plane goes
through `dpu_iova_get()`.

Two pieces were missing: a DTB that opts in, and proof on the phone that the windows really
scan IOVAs.

### 10.3 Design (decon_drv.c)

| Requirement | Where |
|---|---|
| Map each HWC dma-buf through the DPU SysMMU | `dpu_buf_import()` -> `dpu_iova_get()`: `dma_buf_attach(db, decon_f)` + `dma_buf_map_attachment()`. decon_f uses iommu-dma DMA ops on the SysMMU domain with max segment size `UINT_MAX`, so the sg_table is one IOVA run below 4 GiB (domain aperture `U32_MAX`). That address is the DPP base (`b->addr`) |
| Map before the fence / latch | in the `S3CFB_WIN_CONFIG` ioctl (`dpu_build_commit()` -> `dpu_import_win()`), before the commit is queued. The worker waits for the acquire fences and then programs the IOVA |
| Two-plane YUV | each plane gets its own `dpu_iova_get()`; the derived NV12 chroma is luma IOVA + `cofs` |
| IOVA cache | per dma-buf, an LRU of at most 32 entries (the reuse 4.9 got from `ion_iovmm_map()`). Each entry holds its own dma-buf reference, attachment and mapping; `users` counts the commits that scan it |
| Unmap only after the hardware stopped reading | a commit holds its entries until it is freed, and it is freed only after a later update latched. `d->active` is freed after the next latch. After a latch timeout the old and the new commit both stay (`d->stale`) until a latch is seen. Unused entries are unmapped when the cache holds the last dma-buf reference, or when they fall beyond 32 |
| 4.9 release fences | one fence per commit, signalled after its shadow update latched, or with an error if nothing was programmed. It is returned as the release fence of every buffer window and as the retire fence. Once commit N latched, the hardware no longer reads commit N-1's buffers |
| Bounce only as a fallback | without `iommus` (plain v23a-dpu DTB) nothing changes. With `iommus`, a dma-buf that cannot be mapped to one IOVA run below 4 GiB is logged (`decon iommu: no IOVA for a …-byte dma-buf (…): per-frame import and bounce copy`) and bounced. The slots then come from iommu-dma (any pages, not decon-pool CMA) |
| Boot framebuffer identity map | `iommu-addresses` on the 0xcc000000 reservation makes the IOMMU core map it 1:1 before the domain is attached. Probe refuses unless all 18 1 MiB probes translate 1:1. The fb0 adopt/takeover, the self-tests and the panel recovery scan 0xcc000000 = identity IOVA |
| Faults never panic | in 9810 mode, exynos-iommu returns from the fault IRQ. `dpu_iommu_fault()` counts the fault, masks the TE trigger and logs it. Recovery is bounded (3 unmasks), then DECON quiesces until a latch (display-iommu.md) |

New in this branch: the scan proof and its counters.

- `dpu_process()` marks a latched user commit for proof if it scans at least one window through a
  cached IOVA, and it is one of the first 3 such commits or every 1000th after that.
- `dpu_apply()` runs under `hw_lock` with the DPU domain on. It reads back the **active**
  (latched) IDMA base Y of every window, and C for two-plane formats. For proof commits it also
  reads CTRL bit 0 of all three SysMMUs through the read-only views.
- `dpu_iova_proof()` runs after the fence was signalled. It logs one line per buffer window, then
  a verdict. PASS needs all of these:
  - every buffer window was scanned through its cached IOVA, and the latched base equals that
    IOVA (Y and C);
  - the IOVA translates to a physical page (`iommu_iova_to_phys()`);
  - all three SysMMUs translate (0x7);
  - no window was bounce-copied in this commit;
  - no bounce slot is allocated at all (0 bytes).
- `EXYNOS_DPU_DUMP` prints a second line with the IOVA counters (only with `iommus`).

### 10.4 DTB

`arch/arm64/boot/dts/exynos/exynos9810-crownlte-romram-v23a-dpu-iommu.dts` includes v23a-dpu.
The decompiled diff against v23a-dpu.dtb shows only these changes:

- `sysmmu@160a0000/160b0000/160c0000`: compatible `"samsung,exynos9810-sysmmu",
  "samsung,exynos-sysmmu"`, `samsung,qos = <15>`, `samsung,pmu-syscon` = PMU (0x0d),
  `samsung,pd-status = <0x4024 0xf 0xf>`, `status = "okay"` (dtc phandles 0x2a..0x2c);
- reserved-memory `framebuffer@cc000000`: `iommu-addresses = <decon_f 0x0 0xcc000000 0x1400000>`
  (phandle 0x29);
- decon_f: `reg`/`reg-names` without the sysmmu0..2 windows, `iommus = <0x2a 0x2b 0x2c>`,
  `memory-region = <decon-pool framebuffer@cc000000>` (phandle 0x28).

These are the changes of the v22b SysMMU DTB (romram-iommu, a patched decompiled v18), now as
source. decon-pool stays 208 MB: nothing uses it while every buffer gets an IOVA, and the
self-test DTBs still allocate from it. The DTB is built with
`KSRC=<worktree> KO=<build dir> rom-ram/src/build-dtb.sh`. The same command rebuilds
v23a-dpu.dtb byte-identical (fc7201b5).

### 10.5 Trial (ROM-in-RAM, adb-dpu setup)

Pack like adb-dpu (build-19 ADB ramdisk), with the Image and DTB from 10.7. Boot to Lineage,
finish or skip setup, press Home, open a few apps and return Home. Point the camera at the
panel: it must follow `adb screencap` (the failure was a panel frozen on the setup-wizard frame).

Expected kernel lines (`dmesg | grep -E "decon|sysmmu"`):

```
exynos-sysmmu 160a0000.sysmmu: Exynos9810 SysMMU v7.x registered: 4.9 CFG 0x…f80 (QoS 15), …   (×3)
exynos9810-decon 16030000.decon_f: decon iommu: boot fb 0x00000000cc000000+0x1042800 identity map: 18/18 probes 1:1
exynos9810-decon 16030000.decon_f: decon iommu: SysMMUs translating (0x7 -> 0x7)…
exynos9810-decon 16030000.decon_f: fb0 takeover: … : 0, latched …
… once HWC runs:
decon iommu: map #1: 17305600-byte dma-buf, N physical chunks -> IOVA 0x… (cached 1, hits 0, unmapped 0)
decon iommu: scan proof #1 win0 dpp4: active IDMA base 0x…, IOVA 0x… -> PA 0x… (17305600-byte dma-buf, N physical chunks): OK
decon iommu: scan proof #1: 1/1 buffer windows scanned via IOVA (latched base == IOVA), 0 bounce-copied; SysMMUs 0x7 of 0x7 translating; bounce slots 0 bytes, bounce copies since boot 0; IOVA cache 1 (maps 1, hits 0, unmapped 0), SysMMU faults 0: PASS
… proofs #2 and #3, then one at every 1000th IOVA commit
… after Home: a proof (or a later `map #k`) with the 26214400-byte (or larger) wallpaper on win0 dpp4
```

None of these may appear: `bounce buffer`, `scattered buffer (… segments …) copied`,
`cma: decon-pool: alloc failed`, `decon iommu: no IOVA`, `SysMMU … fault`, `scan proof … FAIL`,
`active IDMA base … programmed`. `N physical chunks` > 1 shows that the buffer is scattered in
RAM and contiguous only through the SysMMU.

Counters: `EXYNOS_DPU_DUMP` (and every proof line) reports `IOVA commits windows proofs
proof_fails cache maps hits unmapped faults quiesced`. During steady animation `hits` grows with
every frame. `maps` stays near the number of distinct gralloc buffers (HWC triple-buffers each
layer).

How to read failures:

| Log | Meaning |
|---|---|
| no `decon iommu:` lines, `bounce buffer` lines | the DTB without `iommus` was packed |
| `decon iommu: boot fb IOVA … not identity` / `"iommus" lists …` / `SysMMUs 0x… of 0x7 translating` | DECON refused to probe. The SysMMUs went back to bypass and the splash stays; see display-iommu.md |
| `scan proof … BASE != IOVA: FAIL` | the DPP latched an address other than the IOVA: a driver bug (no buffer leaves the IOVA path silently) |
| `scan proof … not a SysMMU mapping: FAIL` | that window took the fallback (a `no IOVA` warning comes before it) |
| `scan proof …: SysMMUs 0x3 of 0x7 …: FAIL` | a SysMMU stopped translating (runtime PM), so IOVAs are fetched as physical addresses. Expect garbage or an SError |
| `SysMMU read fault #n at IOVA …` | the DPP fetched an IOVA after it was unmapped (lifetime bug) or one that was never mapped. The TE trigger is masked and the panel holds its last frame; see the recovery in display-iommu.md |

### 10.6 Kernel-only check

There is no new self-test step. The v23a-dpu-selftest DTB's fb0 flips and compose steps C1-C3
use in-kernel buffers. With `iommus` those are `dma_alloc_wc()` IOVAs, so their IDMA-base checks
then compare IOVAs. The proof above needs gralloc buffers, so it runs in ROM-in-RAM. No selftest
DTB with `iommus` was built; this branch does not change the self-test compose path.

### 10.7 Artifacts (gekko)

| File | sha256 |
|---|---|
| `panel-bright-work/images/Image-romram-dpu-scale-iommu-7d0935d7a` (commits 9d4fdf396 + 7d0935d7a) | `1a32a91d4c60873f14dcfb36dac6cea68205a3d04f1daf512a73a2ac56e76168` |
| `panel-bright-work/dtb/exynos9810-crownlte-romram-v23a-dpu-iommu.dtb` | `a9747cb9d61fc8a86eaea7bc6bfd46699956d767be07acea346587baaa52223a` |
| `panel-bright-work/dtb/exynos9810-crownlte-romram-v23a-dpu-iommu.dts` (source copy) | `e18537aded20cfce659d6c9abb33ae5fd68033faaa55e58fc95637c16545672f` |
| config: `panel-bright-work/ko-dpu-scale/config-romram-dpu-scale` (unchanged; `CONFIG_EXYNOS_IOMMU=y`, `CONFIG_IOMMU_DMA=y`) | `9e144fd87da63406f35debd27215a04d5ea00735b2f0724a1c45a79e6af30712` |

The kernel was built with clang 18 (`LLVM=-18`, -j3). `W=1` gives no warnings in `dpu_9810/`, and
checkpatch reports nothing on the diff (long-line rules ignored).

### 10.8 Untested and risks

- Not run on hardware. The SysMMU path ran in v22b (bounce-free HWC frames, no faults), but not
  with the dpu-scale composition (scaled VGF windows, YUV planes, rotation) and not with the scan
  proof.
- The IOVA cache keeps up to 32 dma-bufs mapped and referenced after HWC stops using them, until
  the cache holds the last reference or they fall off the LRU. That pins at most those buffers'
  pages.
- TLB way/port tuning is not ported (display-iommu.md), so heavy multi-layer scaling could
  underrun.
- While DECON is quiesced after a held fault, ALPM and panel-recovery command windows return
  -EBUSY. v23a arms panel recovery L2 only on the ELVDD error flag, as v22b did.
- decon-pool keeps its 208 MB CMA reservation. CMA is reusable, so movable pages still use it.
- Panel power, PMIC and brightness code: not touched.
