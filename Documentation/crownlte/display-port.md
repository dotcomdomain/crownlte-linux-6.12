# crownlte display port (Exynos9810 DPU, S6E3HA8) — status and plan

Branch `crownlte-display` (stage 1, based on 401c26eaa, merged into `crownlte-integration` 31caf697c) and `crownlte-display-s2` (stage 2, based on 31caf697c; section 6).
Everything below is **build-only and has not run on the phone**.

| Stage | What | State |
|---|---|---|
| 1 | simplefb + fbcon on SBoot's scanout buffer, DECON TE trigger, on-screen self-test | built, untested |
| 2 | dpu_9810 fbdev DECON/DPP/DSIM + S6E3HA8 + backlight "panel", vendor-HWC ABI | round 1 built (branch `crownlte-display-s2`, section 6): DECON adopt driver, fb0 + full ioctl set, fences, TE vsync, contiguous-buffer window configs, read-only backlight; untested |

## 1. Hardware facts used

From the 4.9 crownlte_eur_open_26 DT (`K49` = /root/crownlte-hid-kernel), static SBoot
analysis (`C:\Users\Patrick\Downloads\Codex Projects\kernel-upgrade\research\sboot\static-evidence.json`,
sboot N960FXXU9FVG2, sdb sha 09d0dd96…) and 4.9 driver headers.

| Item | Value | Source |
|---|---|---|
| DECON-F | 0x16030000 (64 KiB), SPI 182/183/188/421, `psr_mode=2` (command), `trig_mode=0` (HW/TE), `dsi_mode=0`, `max_win=6`, `default_win=5`, `default_idma=0` | 4.9 DT `decon_f@369295360` |
| DECON-T (DP) | 0x16050000, SPI 194–196 | 4.9 DT |
| DPP IDMA G0,G1,VG0,VG1,VGF0,VGF1 | 0x16021000 + n·0x1000 (alias dpp0..5), DPP 0x16071000+…, DPU DMA common 0x16070000 | 4.9 DT `dpp@…` |
| DECON channel map → DPP | ch0 VG0, ch1 VGF0, ch2 VGF1, ch3 G1, ch4 VG1, ch5 G0 | K49 `dpu_9810/helper.c` `dpu_dma_type_to_channel()` |
| DSIM0 | 0x16080000 (+ M4S4 PHY 0x16150000, 16 KiB), SPI 189; PHY isolation PMU 0x70c | 4.9 DT `dsim@369623040`, `phy_m4s4top_dsi0` |
| DPU power domain | PMU 0x14064020 CONFIGURATION, 0x14064024 STATUS (on = bits[3:0] 0xf) | 4.9 DT `pd-dpu@14064020`, `flexpmu_cal_local_exynos9810.h` |
| DPU SysMMUs | 0x160a0000, 0x160b0000, 0x160c0000 (non-secure SFRs; secure base 0x160d0000…), CTRL 0x0 (0x5 = enable), VERSION 0x34 | 4.9 DT `sysmmu@160A0000`…, K49 `exynos-iommu.h` |
| Panel | S6E3HA8 "s6e3ha8_crown_a3_s0", 1440x2960, MIPI **command mode**, 4 lanes, HS 898 MHz, PMSK <2 276 2 20165 …>, DSC 2 slices × 40 lines, 60 Hz, mres 1440/1080/720 | 4.9 DT `s6e3ha8_ddi` |
| Panel rails | S2DOS05 on hsi2c@14350000 addr 0x60: buck1 "vdd1" 1.5–1.6 V, ldo1 "vdd2" 1.8 V, ldo4 "vdd5" 3.0 V (all boot-on), ldo2/3, elvss; reset GPIO, disp-det GPIO | 4.9 DT `s2dos05_pmic@60`, `panel_drv@001` |
| SBoot scanout | DPP IDMA base 0xcc000000, IDMA format 0 (ARGB8888), SRC/IMG 1440x2960, offset 0, packed 5760-byte rows; CPU draws at `0xcc000000 + 4*(y*1440+x)` via an identity map; "lcd_buffer" 0xcc000000..0xcd400000 | static SBoot analysis (raw 0xfb914, 0x100d74, 0xd96e4, 0xa1620) |
| SBoot panel init | F0 5A5A, FC 5A5A, 1A…, **35 00 (TE on)**, ED 4C, C5…, B9…, B0 03, F6 43, C2…, CA gamma, B1 04 40 (AOR), B5…, F4…, F7 03, FC A5A5, F0 A5A5 | static-evidence.json `panel_initialization` |

The 4.9 recovery fb (`recovery-framebuffer.json`) reports smem_start 0x10000000. That is a
SysMMU IOVA of the 4.9 driver's own buffer, not SBoot's buffer.

## 2. Stage 1 — console on the SBoot buffer

### 2.1 What is built

| Commit | Content |
|---|---|
| 8640d1b1c | `drivers/video/fbdev/exynos/exynos9810-decon-handover.c` (`CONFIG_FB_EXYNOS9810_DECON_HANDOVER`) |
| e78cff06c | `exynos9810-crownlte-diag.dts`: simple-framebuffer `/chosen/framebuffer@cc000000` (a8r8g8b8, 1440x2960, stride 5760); no-map reservation grown to SBoot's whole lcd_buffer (0xcc000000 + 20 MiB); read-only `display-handover@16030000` node |
| 0397b97a2 | `exynos9810-crownlte-delayed-ufs-diag.dts`: `samsung,decon-trigger = "hw-te"`, `samsung,display-selftest` |
| a63d07e98 | `crownlte_android_defconfig`: FRAMEBUFFER_CONSOLE, FONTS, FONT_8x16, FONT_TER16x32, FB_EXYNOS9810_DECON_HANDOVER |

Config delta against `build/integration-android-6.12.93/.config`:
`+FB_EXYNOS9810_DECON_HANDOVER=y +FRAMEBUFFER_CONSOLE=y +FONTS=y +FONT_SUPPORT=y +FONT_8x16=y
+FONT_TER16x32=y +VT_HW_CONSOLE_BINDING=y` (the last one is pulled in by fbcon). DRM stays off
(`# CONFIG_DRM is not set`), so there is no `/dev/dri/card0`, which would switch the vendor HWC
to its DRM backend. FB_SIMPLE was already on. fbcon auto-selects TER16x32 (90x92 cells).

Artifacts: `build/display-6.12.93/stage1/` (Image, DTB, .config, System.map, SHA256SUMS).

### 2.2 Why a register write is needed at all

The panel is a command-mode panel with its own GRAM. It keeps showing the last frame it
received, the Note9 logo, even if DECON stops completely. In HW-trigger mode DECON sends a
frame on a TE pulse only while `HW_SW_TRIG_CONTROL` (DECON-F + 0x70) has HW_TRIG_EN (bit 0)
set and HW_TRIG_MASK_DECON (bit 4) clear. 4.9 `decon_reg_set_trigger(DECON_HW_TRIG, ENABLE)`
does exactly that before each frame, and `…DISABLE` (clear bit 0, set bit 4) after it. If SBoot
leaves the trigger masked, nothing the kernel writes into 0xcc000000 ever reaches the panel.

The boot shim's disabled option wrote the constant 0x1281 (bits 12, 8, 7, 0). Bit 7 is
undocumented in the 4.9 headers, and the store would also clear any other bits SBoot set, so
this port does not use it.

### 2.3 Exactly what the handover driver does

At probe (device_initcall, after simplefb by link order):

1. Reads PMU DPU_STATUS (0x14064024) through the PMU syscon. If it is not 0xf, **no DPU
   register is touched at all**: reads of a powered-down block stall the bus.
2. Reads and logs DECON-F GLOBAL_CONTROL (0x0), HW_SW_TRIG_CONTROL (0x70), DATA_PATH_CONTROL_0/1/2
   (0x214/0x218/0x230, plus 0x7218 shadow), FRAME_COUNT (0x2a0) and INTERRUPT_ENABLE/PENDING
   (0x40/0x4c). For every enabled window it reads the mapped DPP's IDMA ENABLE/IN_CON/SRC_SIZE/
   IMG_SIZE/SRC_OFFSET/IN_BASE_ADDR_Y (0x0/0x8/0x10/0x18/0x14/0x40). It also reads the three
   DPU SysMMUs' CTRL/CFG/STATUS/PT_BASE/VERSION (0x0/0x4/0x8/0xc/0x34).
3. **Verified scanout** means all of the following: an enabled window maps to a DPP with
   base == 0xcc000000, format ARGB8888, SRC width·4 == 5760, SRC height ≥ 2960, IMG 1440x2960,
   offset 0; DECON_EN and DECON_EN_F set; every DPU SysMMU CTRL bit 0 clear (bypass).
4. Only with `samsung,decon-trigger = "hw-te"`, a verified scanout and command mode, it does one
   read-modify-write:

   ```
   DECON-F + 0x70 (HW_SW_TRIG_CONTROL): new = (old & ~BIT(4)) | BIT(0)
   ```

   This is identical to 4.9 `decon_reg_set_trigger(HW, ENABLE)`. If the register already reads
   bit0 = 1 and bit4 = 0, nothing is written. It then logs FRAME_COUNT before and 100 ms after
   the change.
5. On shutdown/reboot it writes the original value back.

`"sw-periodic"` (one SW-triggered frame per period, 4.9 SW-trigger sequence) exists as an
**untested fallback** and is not set in any DT. Without `samsung,decon-trigger` (base diag DTS,
other diag variants) the driver only reads.

Not touched: clocks, power domains, regulators, GPIOs/pinctrl, DSIM, the MIPI PHY, panel DCS
commands, video timing, DPP/window/blender configuration, SysMMU and BTS. The legacy clock
driver registers all vclks with CLK_IGNORE_UNUSED, and the DPU Q-channel gates' enable/disable
are no-ops, so `clk_disable_unused` cannot gate the DPU. The DT has no genpd, so nothing powers
the DPU domain off.

### 2.4 Self-test (delayed-UFS diag only)

At ~108 s uptime (after the cpufreq/cpuidle/devfreq tests at 100/101/105 s, before the 120 s
PID1 restart):

1. It re-runs the survey, finds simplefb's fb_info and checks its geometry against the DT.
2. It calls `fb_set_suspend(info, 1)` under console_lock, so fbcon stops drawing.
3. It draws the pattern: dark grey background, 8 px white frame, 120 px corner squares
   (TL red, TR green, BL blue, BR yellow), eight bars labelled WHT YEL CYN GRN MAG RED BLU BLK,
   a grey ramp, "DISPLAY SELFTEST", "LINUX 6.12 CROWNLTE", "SIMPLEFB 0x00000000cc000000",
   "1440x2960 STRIDE 5760", "TIME BELOW MUST COUNT UP", a clock line `T+<uptime>S F<FRAME_COUNT>`,
   a heartbeat square, a checkerboard and a magenta diagonal.
4. It reads back eight pixels, then updates the clock and heartbeat once a second for 10 s.
5. It resumes fbcon (console redrawn) and logs a verdict.

Log prefixes: `display handover:` (probe) and `display selftest:` (test).

### 2.5 Expected results

**Expected log, all good:**
```
display handover: fb 0x00000000cc000000 1440x2960 stride 5760, trigger mode "hw-te"
display handover: DECON-F GLOBAL_CONTROL=0x… (en=1/1 run=… idle=… command mode)
display handover: HW_SW_TRIG_CONTROL=0x… (hw_trig_en=? mask=? sw_trig=?) FRAME_COUNT=N
display handover: win5 -> ch5 dpp0(G0): … (fmt 0) SRC=1440x2960 IMG=1440x2960 OFF=0,0 BASE=0xcc000000
display handover: sysmmu_dpu0..2 CTRL=0x0 … (bypass)
display handover: scanout of 0x00000000cc000000 1440x2960 stride 5760: verified
display handover: WRITE DECON-F+0x70 HW_SW_TRIG_CONTROL 0x…10 -> 0x…01 (readback …)
display handover: FRAME_COUNT a -> b over 100 ms after trigger change   (b-a ≈ 6)
display selftest: scheduled at 108000 ms uptime, pattern held 10 s
…
display selftest: tick k t=… FRAME_COUNT=…   (increasing ≈ 60/s)
display selftest: PASS: readback 8/8, FRAME_COUNT x -> y (advanced in 10/10 ticks); trigger written by kernel; fbcon resumed
```
(The window index and channel are SBoot's choice. The values above are what the static analysis
predicts, not observed values.)

**Expected on screen:** the Note9 logo disappears within ~2 s of kernel start. Black screen with
white boot text in Terminus 16x32. At ~108 s the test pattern appears with a clock counting up
for 10 s. Then the console text returns. At 120 s the diag restarts into Lineage.

**How to read failures:**

| Observation | Meaning |
|---|---|
| logo stays, log `PMU DPU domain is off` | SBoot powered the DPU down: Stage 1 cannot work, go straight to Stage 2 DSIM/DECON init |
| logo stays, `refusing trigger write`, sysmmu `ENABLED` | SBoot scans through a SysMMU: 0xcc000000 is an IOVA. Read PT_BASE; Stage 2 must own the SysMMU first |
| logo stays, `NOT verified`, window BASE ≠ 0xcc000000 | wrong buffer address. Point the simplefb reg at the logged BASE (if it is in RAM) and retry |
| logo stays, `no write` (already unmasked), FRAME_COUNT not advancing | DECON stalled (DSIM/TE). Try `sw-periodic` in a later trial |
| text visible but red↔blue swapped (bar labelled RED shows blue) | format is a8b8g8r8: change the simplefb `format` |
| picture sheared or diagonal broken | stride mismatch |
| pattern visible but clock frozen | only one frame transferred: the trigger is re-masked by something; use `sw-periodic` |
| `FRAME_COUNT` advancing, readback 8/8, but nothing visible | the window may be blended away (alpha/colormap) or DSIM is not sending: Stage 2 |

### 2.6 Stage 1 risks

- **Panel power, voltages and timing are not changed.** No regulator, GPIO, DSIM, PHY, PLL or DCS
  write exists in the Stage 1 code. The panel only receives frames in the format and timing
  SBoot already uses for the logo, exactly as 4.9 does on every frame. The trigger write cannot
  change voltages or timing. There is no known path to panel damage.
- Bus hang: a read of a powered-down DPU register would hang the CPU (trial stuck at the logo,
  RECOVERY-loop procedure needed). This is mitigated by the PMU DPU_STATUS gate. The SysMMU SFR reads
  rely on HWACG Q-channel wake on APB access, as 4.9 does (its gate vclks are no-ops too).
- Continuous refresh: with the trigger unmasked DECON fetches 17 MB per TE (~1 GB/s at 60 Hz).
  This is normal Android load; MIF devfreq floors keep it fed. An underrun would show as
  glitches only.
- OLED burn-in: a static console for 120 s is harmless. Long-running use should blank the
  console (consoleblank=…) once Stage 2 can turn the panel off.
- fbcon and 4.9 parity: once the DECON driver exists it must remove simplefb
  (`aperture_remove_conflicting_devices(0xcc000000, …)`) so that it becomes fb0.

## 3. Vendor ABI constraints (from roadmap/vendor-abi)

- `/dev/graphics/fb0` must be DECON-F; `/dev/graphics/fb1` DECON-T (DP, late).
- Device names are literal: `/sys/devices/platform/16030000.decon_f/{vsync,psr_info}`,
  `/sys/devices/platform/16050000.decon_t/vsync`, backlight
  `/sys/devices/platform/panel_drv@001/backlight/panel/{brightness,max_brightness}` (max 44700),
  `/sys/class/lcd/panel/*`, `/sys/class/mdnie/mdnie/*`. **Consequence: decon_f, decon_t and
  panel_drv must be root-level DT nodes** (named `decon_f@16030000`, `panel_drv@001`), not
  children of `soc@0`. Otherwise sysfs paths become `/sys/devices/platform/soc@0/…`. selinux
  genfscon entries use the same paths.
- ioctls: `include/uapi/video/exynos9810_decon.h`. The HWC sends **S3CFB_WIN_CONFIG_OLD**
  (1416 B: 7 × 200-byte configs); 4.9 also accepts the 1648-byte S3CFB_WIN_CONFIG. Unknown nrs
  (`'F'` 211, 261, 610–615) must return -ENOTTY. `S3CFB_POWER_MODE` takes DISP_PWR_* (doze path).
- vsync: sysfs `vsync` attribute (format `%llu` plus newline: ktime ns of the last vsync),
  `sysfs_notify` from a vsync kthread while `S3CFB_SET_VSYNC_INT` is on.
- Fences: sync_file acquire/release/retire fences (4.9 DPU_2_0_RELEASE_FENCES).
- No `/dev/dri/card0`, ever, while the HWC runs in fbdev mode.

## 4. Stage 2 plan — dpu_9810 forward port

### 4.1 What the 4.9 tree has (`K49/drivers/video/fbdev/exynos`)

| Part | Files (lines) | Port decision |
|---|---|---|
| DECON core + fbdev + ioctls | decon_core.c 4455, decon_dsi.c 1328, decon.h 1787, helper.c 843 | port |
| Register layer | decon_reg.c 2476, dpp_reg.c 1512, dsim_reg.c 2315, regs-*.h 1642 | port as is (pure SFR code) |
| DPP | dpp_drv.c 1135, dpp.h 400, dpp_coef.h, hdr_lut.h | port (HDR LUT later) |
| DSIM + M4S4 PHY | dsim_drv.c 1833, dsim.h 483, K49 `drivers/phy/…mipi…` | port; PHY folded into DSIM at first |
| Window update (partial) | win_update.c 655 | later (CONFIG off: update slot ignored) |
| Cursor async | cursor.c 231 | port (small) |
| DSU / multi-resolution | dsu.c 197 | later (single 1440x2960 mode first) |
| BTS | bts.c 569 (`soc/samsung/bts.h`) | replace: MIF/INT floors through the ACPM devfreq (dev_pm_qos) |
| Event log / systrace | event_log.c 1183 | drop or reduce to a tracepoint |
| DisplayPort / HDCP / secdp | ~11k | drop (M7) |
| Panel framework | panel.c 1610, panel_drv.c 2720, panel_bl.c 867, dimming.c 1594, sysfs.c 3122, mdnie.c 1625, copr.c 872, timenval.c | port panel, panel_drv, panel_bl, dimming, timenval, sysfs; mdnie/copr after |
| S6E3HA8 | s6e3ha8.c 2441 + crown_a3_s0 tables (~133k lines of .h) | port verbatim tables |
| AOD, POC (panel flash over SPI), DPUI, active clock | aod/, panel_poc.c, spi.c, dpui.c, active_clock.c | drop first |
| S2DOS05 PMIC | K49 `drivers/regulator/s2dos05*` | port late (rails stay boot-on) |

### 4.2 4.9 → 6.12 API mapping

| 4.9 | 6.12 |
|---|---|
| ION client, `ion_import_dma_buf_fd`, `ion_alloc` | `dma_buf_get(fd)`, `dma_buf_attach`, `dma_buf_map_attachment_unlocked`. The fb0 memory reuses 0xcc000000 (seamless handover) or `dma_alloc_wc` from CMA |
| `ion_iovmm_map/unmap`, `iovmm_activate` (on the DSIM device, all three DPU SysMMUs) | DMA API through the IOMMU (`iommus = <&sysmmu_dpu0 &sysmmu_dpu1 &sysmmu_dpu2>`); `dma_set_max_seg_size(UINT_MAX)` so a mapped buffer is one IOVA run |
| no IOMMU driver yet | dma-direct: only physically contiguous buffers (CMA heap, the reserved fb). DPP takes one base per plane, so a scattered system-heap buffer **cannot** be scanned out without the SysMMU |
| `sync_file`, `dma_fence` (DPU_2_0 fence) | same APIs; drop `.enable_signaling`/`.wait` ops boilerplate |
| `exynos_pd` + `pm_runtime` | phase A: DPU domain kept on (no runtime PM, blank = display off + DECON stop). Phase B: genpd provider on `pmucal_local` `dpu_on/off` (tables already in the 6.12 cal-if) |
| `pm_qos` DISP/INT/MIF throughput, `bts_update_bw` | devfreq QoS on `legacy_devfreq_mif/int` (+ a DISP domain in the ACPM devfreq driver later) |
| clk "aclk" vclk 850, DPU HWACG gates | legacy vclks exist (`GATE_DPU*`, `UMUX_CLKCMU_DPU_BUS`); request but never rely on gating |
| `of_get_gpio`, pinctrl `hw_te_on/off` | gpiod + pinctrl states on pinctrl@10830000 |
| `v4l2_subdev` between decon/dpp/dsim/panel | keep: enable MEDIA_SUPPORT + VIDEO_DEV (no video nodes are created). Replacing it would touch every panel call site |
| `setup_timer`, `kthread_worker`, `exynos_ss_*`, `sec_debug`, `smc` content protection, `switch` class | timer_setup / same / drop / drop / reject protected windows / drop |
| fbdev `fb_info` flags/ops | `FB_DEFAULT_IOMEM_OPS` + own `fb_ioctl`, `fb_mmap` (dma-buf aware), `fb_pan_display` |

### 4.3 IOMMU (SysMMU) question

- SBoot scans 0xcc000000 as a **physical** address (identity CPU map, DPP base written
  directly). If the Stage 1 log shows all three DPU SysMMUs with CTRL bit 0 clear, bypass works
  and DPP can DMA physical addresses with no SysMMU driver.
- The vendor gralloc allocates HWC layers from `/dev/dma_heap/system`, which is scattered. The
  HWC therefore needs the SysMMU. Mainline `drivers/iommu/exynos-iommu.c` (already in the 6.12
  tree, off) supports v1–v7 register layouts and picks by VERSION (0x34). Stage 1 logs VERSION.
  If it is 7.x this is mostly DT work; 4.9's `sysmmu,tlb_property` / way config are performance
  tuning and can come later. The secure SFR block (0x160d0000…) must not be touched from the NS world.
- Interim (S2-5): CMA-backed buffers only, tested with a small tool, before the SysMMU is ported.

### 4.4 DT layout for stage 2

Root-level nodes, mirroring 4.9 names: `decon_f@16030000` (`compatible = "samsung,exynos9-decon"`
plus a 9810 string), `dpp@16021000`…`dpp@16026000` with aliases dpp0..5, `dsim@16080000` with
the PHY, `disp_ss@16010000` (0x16011000 reg), `panel_drv@001`, `sysmmu@160a0000`….
`s6e3ha8_ddi` timing node (copy verbatim). The Stage 1 handover node and simplefb stay as a
fallback: the DECON driver takes the aperture over at probe.

### 4.5 Milestones (each one is one or more commits that build, gated by DT/Kconfig)

| Step | Content | Hardware test | Panel risk |
|---|---|---|---|
| S2-1 ✔ | UAPI header + compile-time ABI checks (cd4f9a72b) | — | none |
| S2-2 partly done (4.9 regs headers + DECON/DPP window code) | Import the register layer (`regs-*.h`, `decon_reg.c`, `dpp_reg.c`, `dsim_reg.c`) behind a slim `dpu9810_hw.h`, build-only | — | none |
| S2-3 | DSIM driver in **adopt** mode: if PLL locked (4.9 `dsim%d PLL is stabled in bootloader`) skip link/D-PHY init; DCS read of panel ID (0x04/0xDA–0xDC) with the DECON trigger masked | ID logged (expect 0x912047 per 4.9 "id:0x00912047") | low (read commands only) |
| S2-4 done, round 1 | DECON-F core in adopt mode + fbdev fb0 on 0xcc000000 (takes over from simplefb), frame-done IRQ, `vsync` sysfs, FBIO_WAITFORVSYNC, pan_display with per-frame trigger enable/mask as 4.9 | fbcon via DECON; vsync timestamps at 60 Hz | none (same timing/format) |
| S2-5 done for unscaled RGB, round 1 | DPP driver + S3CFB_WIN_CONFIG(_OLD) for contiguous dma-bufs, acquire/release/retire fences | test tool composes 2 windows from `/dev/dma_heap/linux,cma` | none |
| S2-6 | SysMMU v? for DPU (mainline exynos-iommu + DT), fault handler; system-heap buffers | HWC from Lineage reaches first composed frame | none |
| S2-7 done, round 1 | EXYNOS_DISP_INFO, display/color mode ioctls, HDR caps, cursor; -ENOTTY for unknown nrs | HWC init without errors, SurfaceFlinger boots | none |
| S2-8 | Panel framework + S6E3HA8 crown_a3_s0 in keep-alive mode (no regulator/reset control): backlight class "panel" (max 44700) through the 4.9 dimming/AOR/gamma tables verbatim, lcd class "panel" sysfs | brightness steps, compare luminance with 4.9 | **medium**: wrong gamma/AOR/ELVSS tables can over-drive the OLED. Use only verbatim 4.9 tables, cap at normal brightness (no HBM) until verified |
| S2-9 | Blank/unblank: DSIM display off/sleep-in and back, DECON stop/start; then S2DOS05 regulator driver + full 4.9 power sequence; DPU genpd | screen off/on cycles, suspend | **medium-high**: rail order/voltages (vdd1 1.5–1.6 V, vdd2 1.8 V, vdd5 3.0 V, ELVSS) must follow 4.9 exactly; never set voltages outside the 4.9 DT constraints |
| S2-10 | DISP DVFS/BTS QoS, multi-resolution (DSU, mres 1080/720), doze/AOD, MDNIE/COPR, DP | — | low–medium |

Rules for every hardware step: register writes are DT-gated; first boots are adopt-only; any
PLL/PHY/porch/DSC value comes only from the 4.9 DT `s6e3ha8_ddi` node; HBM and AOD stay off
until luminance has been compared with 4.9.

### 4.6 Things that could damage or degrade the panel (stage 2)

1. ELVSS/AOR/gamma (CA, B1, B5, F4 registers): wrong values change OLED drive current. Only the
   4.9 crown_a3_s0 tables, and no interpolation code changes.
2. HBM (high-brightness mode, above the normal maximum) held for long periods: burn-in and
   heat. Keep it disabled.
3. Power sequencing: enabling ELVDD/ELVSS before vdd1/vdd2, or removing logic supply while
   ELVSS is on. Follow 4.9 `panel_drv` order and delays.
4. DSIM HS clock/PMSK, porches, DSC: a mismatch gives garbage or no picture, not damage. Still,
   copy from 4.9, never invent.
5. Static images: the fbcon console must be blanked once DPMS works.

## 5. Open questions the Stage 1 trial answers

1. Is the DPU domain still on at kernel entry? (PMU DPU_STATUS)
2. Does SBoot leave the TE trigger masked? (HW_SW_TRIG_CONTROL)
3. Which window/DPP does SBoot use, and is the base really 0xcc000000? (survey)
4. Are the DPU SysMMUs in bypass, and what VERSION are they? (decides S2-6)
5. Does FRAME_COUNT advance at ~60/s after the trigger change? (DECON/DSIM/TE path alive)
6. Is the pixel order a8r8g8b8? (RED/BLU bars)

## 6. Stage 2 round 1 — DECON-F adopt-mode driver (branch `crownlte-display-s2`)

Based on `crownlte-integration` 31caf697c (stage 1 + watchdog + S2MPS18). Build-only,
**untested on hardware**.

### 6.1 What is built

| Path | Content |
|---|---|
| `drivers/video/fbdev/exynos/dpu_9810/decon_drv.c` | platform driver `samsung,exynos9810-decon`: adopt, fb0, ioctls, update worker, fences, TE vsync, sysfs, self-test |
| `drivers/video/fbdev/exynos/dpu_9810/decon_hw.c` | register layer ported from 4.9 decon_reg.c / dpp_reg.c / helper.c (unscaled RGB only) |
| `drivers/video/fbdev/exynos/dpu_9810/regs-decon.h`, `regs-dpp.h`, `dpp_coef.h` | 4.9 headers, verbatim (coefficient tables made static) |
| `drivers/video/fbdev/exynos/dpu_9810/panel_ro.c` | read-only backlight "panel" (`samsung,exynos9810-panel-readonly`) |
| `arch/arm64/boot/dts/exynos/exynos9810-crownlte-delayed-ufs-display-s2-diag.dts` | trial DTB: root-level `decon_f@16030000`, `panel_drv@001`, `decon-pool` (48 MiB CMA below 4 GiB), stage 1 helper disabled |
| `arch/arm64/boot/dts/exynos/exynos9810.dtsi` | `sysmmu_dpu0..2`, disabled (see `display-iommu.md`) |

Config: `FB_EXYNOS9810_DPU=y`, `FB_EXYNOS9810_PANEL_RO=y`, `BACKLIGHT_CLASS_DEVICE=y`
(also in `crownlte_android_defconfig`). Both drivers are inert unless the DT has their
nodes. The normal delayed-UFS diag DTB gets only the disabled SysMMU nodes.

### 6.2 Register writes, in order

1. Probe (adopt): the same gates as stage 1: PMU 0x14064024 == 0xf, DECON_EN|EN_F,
   command mode, all DPU SysMMU CTRL bit 0 clear, and an enabled window whose DPP scans
   the boot fb at offset 0 with 1440x2960 and 5760-byte rows. Then the same single trigger RMW
   (DECON+0x70: set bit 0, clear bit 4) if it is not already set.
2. TE IRQ request: pinctrl writes for gpd0-0, i.e. CON = EINT function 0xf (the value
   SBoot/4.9 `hw_te_on` already uses), EINT_CON rising edge, EINT mask cleared. This is
   identical to what 4.9 does.
3. fb takeover (`samsung,fb-takeover`), as a normal commit:
   - **DPP of SBoot's window:** IDMA SRC_OFFSET/SRC_SIZE/IMG_SIZE, IN_CON rotation 0,
     block off, format BGRA8888 (3); DPP IMG_SIZE, IN_CON alpha-sel per-pixel, format
     ARGB8888; scale ratio 1<<20 and the 8:8 coefficient set (4.9 writes these every frame);
     IN_BASE_ADDR_Y/C = 0xcc000000.
   - **DECON:** WIN_CONTROL_0/1 (COPY), WIN_START/END_POSITION full screen,
     WIN_START_TIME 0, DATA_PATH_CONTROL_0/1 (all other windows disabled, chmap), then
     SHADOW_REG_UPDATE_REQ global bit 31, with the trigger masked around the programming
     and unmasked again.
4. A DPP used for the first time gets the 4.9 `dpp_reg_init` values: IRQs masked
   (4.9 unmasks them; we have no DPP IRQ handler), clock-gate enables 0, DMA common QoS
   LUTs 0x44444444, dynamic gating 0, frame alpha 0xff.
5. Not written: DECON INTERRUPT_ENABLE (completion is polled through
   SHADOW_REG_UPDATE_REQ, as in 4.9 `decon_reg_wait_for_update_timeout`), blender
   background, data path / DSC / splitter, DSIM, PHY, PMU, CMU, the SysMMUs.

Why IDMA format 3 for fb0: 4.9 sets up its own fb0 window as DPP `BGRA_8888`, and its fb0
var is red@16/green@8/blue@0/alpha@24 (Linux ARGB8888, recovery capture). So the IDMA
format names follow memory byte order, which makes SBoot's format 0 most likely bytes
A,R,G,B. The stage 1 simplefb `a8r8g8b8` guess may therefore show wrong colours in q25.
The stage 2 takeover makes fb0 match 4.9 whatever q25 shows.

### 6.3 Expected log (display-s2 DTB)

```
exynos9810-decon 16030000.decon_f: SBoot state: GLOBAL_CONTROL=0x… TRIG=0x… DP0=0x… DP1=0x… FRAME_COUNT=N
exynos9810-decon 16030000.decon_f: sysmmu_dpu0..2 CTRL=0x0 VERSION=0x7…        (major 7 expected)
exynos9810-decon 16030000.decon_f: win5 -> dpp0: IN_CON=… SRC=0xb9005a0 IMG=0xb9005a0 OFF=0 BASE=0xcc000000
exynos9810-decon 16030000.decon_f: adopted: win5 dpp0 IDMA format 0 at 0x00000000cc000000
exynos9810-decon 16030000.decon_f: WRITE HW_SW_TRIG_CONTROL 0x… -> 0x…        (only if masked)
exynos9810-decon 16030000.decon_f: fb0: 1440x2960 stride 5760 at 0x00000000cc000000
exynos9810-decon 16030000.decon_f: fb0 takeover: win5 dpp0 DPP BGRA_8888 at …: 0, latched after ~17000 us, FRAME_COUNT a -> a+1..2
exynos9810-decon 16030000.decon_f: adopt mode ready: TE irq NN, fb0 window owned
panel-readonly panel_drv@001: read-only backlight "panel": 12800/44700 (no panel driver yet)
decon selftest: scheduled at 108000 ms uptime, 20 flips every 400 ms
…
decon selftest: start at 108.0xx s: FRAME_COUNT=… TE=… commits=1 fb_taken=1
decon selftest: buffer A at 0x…, buffer B at 0x…   (inside decon-pool, below 4 GiB)
decon selftest: flip i -> A/B: 0, fence after ≤ 34000 us, FRAME_COUNT … (+~24) TE … (+~24)
decon selftest: restored fb0: 0
decon selftest: PASS: 20/20 flips latched, FRAME_COUNT x -> y (advanced in 20/20 flips), TE t0 -> t1, max fence wait … us, latch timeouts 0
```
(Window and DPP numbers are SBoot's; the values above are predictions.)

### 6.4 Expected screen

The logo becomes the boot console. If the stage 1 colours were wrong, they are correct from
the takeover on (white text on black). At ~108 s the screen alternates every 400 ms for
~8 s between a **green** buffer showing a big "A" and a **magenta** buffer showing "B".
Both have a white frame, the text "DECON FLIP TEST / CMA BUFFER A|B", and a red bar
(lower in B). Then the console comes back. The diag restarts at 120 s.

### 6.5 Failure reading

| Log | Meaning |
|---|---|
| `PMU DPU domain is off` / `not running in command mode` / `no enabled window scans` | adopt refused; simplefb (stage 1) stays as fb0 |
| `DPU SysMMU enabled by SBoot` | adopt refused; IOMMU work first |
| takeover `-110` (ETIMEDOUT), FRAME_COUNT not advancing | the shadow update never latched: no TE-triggered frames (check the TE count) |
| flips `0` but the screen stays on the console | DECON scans something else: compare the `win… BASE` dump (EXYNOS_DPU_DUMP) with the buffer addresses |
| TE count stays 0 | the EINT on gpd0-0 is not firing (the pinctrl EINT path is wrong); vsync falls back to sleeps |
| `cannot allocate … CMA buffer` | decon-pool missing or too small |

### 6.6 HWC readiness

- fb0 ioctls, `vsync`/`psr_info` sysfs and fences are in place. `EXYNOS_DISP_INFO` reports
  psr_mode 2, chip 9810 and a single 1440x2960 mode (mres disabled: no DSU yet).
- **The HWC's own layers will be rejected** until the SysMMU works, because gralloc's
  system-heap buffers are scattered. Expect `win%d rejected (… ): -22` /
  `buffer not contiguous` and a failing `S3CFB_WIN_CONFIG`. Testing userspace before
  then needs CMA-heap buffers (`/dev/dma_heap/linux,cma` is only 16 MiB in the defconfig;
  a 1440x2960 layer is 17 MB).
- The blank, power-mode and set-display-mode calls are accepted and ignored. Brightness is
  read-only.
- The self-test uses the shared win-config path with kernel-exported dma-bufs. Only the fd
  lookup and copy_from_user of the ioctl are not exercised: a kernel thread cannot hold
  user fds.

### 6.7 Stage 2 risks (this round)

- **No panel power, voltage, timing or DSIM change exists in this code.** The panel only
  receives frames in the format 4.9 itself uses for fb0 (BGRA8888), at the timing SBoot
  set up.
- Wrong DPP/window programming shows garbage or a frozen picture: recoverable, no damage.
  A DPP register write to a non-existent offset could raise an SError. Every offset
  written is one that 4.9 writes for the same DPP id on every frame.
- Probe hangs: all DPU accesses happen after the PMU DPU_STATUS check. The SysMMU SFRs
  read are the non-secure ones that 4.9 reads.
- TE EINT at 60 Hz adds ~60 IRQ/s.
- Self-test buffers come from a reusable CMA pool. Migration on allocation can take a few
  ms; failure is logged and nothing is flipped.

## q25/q26 result: wrong register bank (fixed 2026-11-04)

q25 and q26 both reported `IMG=1440x2960` but `IN_CON=0 SRC=0 BASE=0`, so neither stage verified the
SBoot scanout. Cause: every DPP channel has two blocks. The DPP block at 0x16021000 + n·0x1000 (4.9
`dpp_read`, DT reg[0]) holds the DPP format, IMG_SIZE at 0x18, CSC and scaler. The IDMA block at
0x16071000 + n·0x1000 (4.9 `dma_read`, DT reg[1]) holds the IDMA format, SRC_SIZE, SRC_OFFSET,
IMG_SIZE and IN_BASE_ADDR_Y (0x40). SBoot's DPP init (raw 0x100904) uses the same split and writes
0xcc000000 to 0x16071040. Stage 1 only mapped the DPP block; the stage 2 DT had `dpp-dma`/`dpp` swapped.

Fixes: dd4a8255e (stage 1, new `dpp-dma` resource; also crownlte-display 6ac56b174) and c3b589012 (stage
2 DT un-swap). Both drivers now also require the DPP-block IMG_SIZE to equal the IDMA IMG_SIZE and a
linear (no rotation/AFBC/block) IDMA input. Trials: q38-display3 (stage 1, recovery 4d2c9154) and
q39-display4 (stage 2, recovery d9fd174d). Full analysis: gekko
`/mnt/ssd/host-builds/kernel-upgrade/roadmap/display-scanout-handover.md`.
