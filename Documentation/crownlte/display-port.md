# crownlte display port (Exynos9810 DPU, S6E3HA8) — status and plan

Branch `crownlte-display`, based on `crownlte-integration` 401c26eaa.
Everything below is **build-only and has not run on the phone**.

| Stage | What | State |
|---|---|---|
| 1 | simplefb + fbcon on SBoot's scanout buffer, DECON TE trigger, on-screen self-test | built, untested |
| 2 | dpu_9810 fbdev DECON/DPP/DSIM + S6E3HA8 + backlight "panel", vendor-HWC ABI | step S2-1 of 10 done (ioctl UAPI + checks) |

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
| S2-2 | Import the register layer (`regs-*.h`, `decon_reg.c`, `dpp_reg.c`, `dsim_reg.c`) behind a slim `dpu9810_hw.h`, build-only | — | none |
| S2-3 | DSIM driver in **adopt** mode: if PLL locked (4.9 `dsim%d PLL is stabled in bootloader`) skip link/D-PHY init; DCS read of panel ID (0x04/0xDA–0xDC) with the DECON trigger masked | ID logged (expect 0x912047 per 4.9 "id:0x00912047") | low (read commands only) |
| S2-4 | DECON-F core in adopt mode + fbdev fb0 on 0xcc000000 (takes over from simplefb), frame-done IRQ, `vsync` sysfs, FBIO_WAITFORVSYNC, pan_display with per-frame trigger enable/mask as 4.9 | fbcon via DECON; vsync timestamps at 60 Hz | none (same timing/format) |
| S2-5 | DPP driver + S3CFB_WIN_CONFIG(_OLD) for contiguous dma-bufs, acquire/release/retire fences | test tool composes 2 windows from `/dev/dma_heap/linux,cma` | none |
| S2-6 | SysMMU v? for DPU (mainline exynos-iommu + DT), fault handler; system-heap buffers | HWC from Lineage reaches first composed frame | none |
| S2-7 | EXYNOS_DISP_INFO, display/color mode ioctls, HDR caps, cursor; -ENOTTY for unknown nrs | HWC init without errors, SurfaceFlinger boots | none |
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
