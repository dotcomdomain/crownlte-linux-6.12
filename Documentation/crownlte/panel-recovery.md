# crownlte panel recovery: the 4.9 screen off / on cycle (levels 1 and 2)

Driver: `drivers/video/fbdev/exynos/dpu_9810/panel_recovery.c` (with
`panel_bright.c` / `panel_bright.h` and `dsim_cmd.c`). Research and owner
decisions: gekko
`/mnt/ssd/host-builds/kernel-upgrade/roadmap/decisions-20261105/1-panel-recovery.md`.
The owner approved levels 1 and 2 on 2026-11-05. Level 3 (the S2DOS05 rail
cycle) is not approved and not implemented. No PMIC or regulator register
is touched at any level.

## Why

Since 2026-11-05 02:48 the panel is black from SBoot's logo on. The fault
happens before the kernel starts. The DDI reports RDDPM `0A` = 0x9f (normal
display), TE on and OTP loaded. Under the level 2 key it reports ERR_FG `EE`
= 0x08 (ELVDD error) and `EA` = `80 00 00 00 00`. On the owner's 4.9 Lineage,
a screen off and on relights the panel. This driver sends the panel commands
of that cycle once, with the rails and the DSIM link left on.

## Device tree (panel node, the one with `samsung,fault-display-brightness`)

| property | meaning |
|---|---|
| `samsung,panel-recovery-level = <1>` or `<2>` | enables the recovery. 1 = DCS-only cycle; 2 = the same plus the reset pulse |
| `samsung,panel-recovery-at-ms = <8000>` | uptime to start at (default 8000) |
| `samsung,panel-recovery-only-on-elvdd-error` | optional. Runs only when the keyed `EE` read before shows bit 3. By default the recovery runs regardless. |
| `samsung,panel-reset-gpios = <&gpa3 4 0>` | level 2 only. Must resolve to gpa3-4 of `pinctrl@14050000` |

Requirements: `samsung,fault-display-brightness = <44700>` (the set-bl
sequence comes from `panel_bright.c`) and `samsung,panel-id`.

DTBs (gekko `panel-bright-work/dtb/`, built from
`exynos9810-crownlte-romram-v18.dtb` by decompiling and adding the properties):

- `v19a`: level 1 at 8000 ms. `samsung,alpm-cycle-at-ms` is removed.
- `v19b`: like v19a, but level 2, with `samsung,panel-reset-gpios`. The `gpa3-gpio-bank` node gets `phandle = <0x26>`.

## The owner's 4.9 configuration this follows

Source: `/root/crownlte-hid-kernel` (K49). Configuration:
`build/close-range-fix-4.9/.config`.

| option | value |
|---|---|
| `SUPPORT_DSU`, `SUPPORT_AFC`, `EXYNOS_DECON_LCD_COPR`, `DISPLAY_USE_INFO`, `DISP_PMIC_SSD` | y |
| `OLD_DISP_TIMING`, `ACTIVE_CLOCK` | n |

So the init table also contains the DSU scaler/CASET/PASET, the COPR set and
a self-diag read, and the exit table also contains the AFC off and the
ERR_FG/DSI_ERR dumps. The task list did not include these entries.

Abbreviations below:
- `PNL` = `K49/drivers/video/fbdev/exynos/panel/s6e3ha8/s6e3ha8_crown_a3_s0_panel.h`
- `DRV` = `K49/drivers/video/fbdev/exynos/panel/panel_drv.c`

Flow (4.9):

| event | 4.9 code | what happens |
|---|---|---|
| Screen off | `panel_sleep_in()`, DRV:1129-1174 | `copr_disable()` and `mdnie_disable()` send no packets. Then `panel_display_off()` (DRV:596) and `__panel_seq_exit()` (DRV:290-309). The DECON has already stopped frames. |
| Power off | `__set_panel_power(OFF)`, DRV:119-160 | Reset low, then vdd1, vdd2 and vdd5 off. Not done here, except the reset at level 2. |
| Power on | same function | vdd5, vdd2 and vdd1 on, 10 ms, reset high, 5 ms. Then the DSIM is reset and initialised to LP-11 (`_dsim_enable`, `dpu_9810/dsim_drv.c:761-830`). |
| Screen on | `panel_sleep_out()` → `__panel_seq_init()`, DRV:236-288 | `PANEL_INIT_SEQ`. 4.9 skips it entirely if disp-det is already high. |
| First frame done | `PANEL_IOC_EVT_FRAME_DONE`, DRV:1817-1836 | `fix_green()` if enabled, then `panel_display_on()` (DRV:564), then `mdnie_update` and `copr_enable`. |

Packet transport (`K49/drivers/video/fbdev/exynos/dpu_9810/panels/common_mipi_lcd.c:188-242`, `mipi_write()`):
- A 1-byte write goes as DCS short (0x05). A longer write goes as DCS long (0x39).
- A packet offset > 0 sends DCS short-param `B0 <ofs>` (0x15) first.
- `DSI_PKT_TYPE_COMP` goes as `MIPI_DSI_DSC_PRA` 0x07: short, DATA0 = byte 0, DATA1 = 0.
- `DSI_PKT_TYPE_PPS` goes as `MIPI_DSI_DSC_PPS` 0x0A: long, 128 bytes.
- Type mapping: `K49/.../panel/panel.c:789-793`. DSIM side: `dpu_9810/dsim_drv.c:234-300` (`dsim_write_data()`).

Keys (`KEYINFO`) are reference-counted (`panel.c:838-880`). Within these tables
each key goes 0 → 1 → 0, so every key packet is sent. The COPR table uses
`PKTINFO` keys, which are always sent. Delays use
`usleep_range(us, us + 10)` (`panel.c:541-551`).

## Packet list (in order; all levels)

Window 1 is one DECON command window (`dpu_cmd_window_begin`: TE trigger
masked, no frame in flight).

### 0. Preconditions (read only; on refusal nothing but the reads below is sent)

1. `dpu_dsim_check()`: command mode, not in reset or ULPS, PLL stable, FIFOs empty.
2. DECON DSC check, register reads only:
   - `DATA_PATH_CONTROL_2` path must be 0xB1 (DSCC + DSC ENC0/1 + DSIM IF0).
   - DSC0 and DSC1 `DSC_PPS00_03..DSC_PPS84_87` (DECON + 0x4000/0x5000 + 0x20..0x74), read as big-endian words. Bytes 4-7 and 10-15 must equal the PPS below. Bytes 8-9 (picture width) × 2 must equal 1440.
   - Each encoder carries half the width, and 4.9 `dsc_calc_pps_info()` computes its own rate-control values (decon_reg.c:1168-1300, `comp_cfg` 0x30 at :1299, bpp 8<<4). So the other bytes are logged with a difference count but not compared.
   - The 6.12 DT `samsung,dsc-*` values and the 1440x2960 resolution must match the PPS.
3. `panel_bright.c` `pb_prepare()`:
   1. Status reads `0A 0E 05 0F EE EA`.
   2. RDDPM `0A`: `& 0x7c` must be 0x1c (normal display).
   3. ID `04` = DT `samsung,panel-id` (91 20 47).
   4. `F0 5A 5A`, then B0 02 / read B3 (34 bytes), B0 17 / read B5 (1 byte), the CA/B1/B5/F4/B8 snapshot and keyed `EE`/`EA`, then `F0 A5 A5`.
   5. HBM gamma must match DT `samsung,hbm-gamma-expect`, and F4 must start with EB.
4. Optional: the keyed `EE` must have bit 3 set.
5. Level 2: the reset GPIO descriptor must be gpa3 (gpio_chip label) pin 4 of the device at 0x14050000. The read-only `ioremap` of CON 0x14050080 / DAT 0x14050084 must show CON[19:16] = 1 (output) and DAT bit 4 = 1. `gpiod_get_raw_value()` must be 1.

### 1. Display off: `crown_a3_s0_display_off_cmdtbl`, PNL:1359-1363

| # | bytes | 4.9 |
|---|---|---|
| 1 | `9F A5 A5` | KEY1 on, PNL:648 |
| 2 | `28` | DISPLAY_OFF, PNL:656 |
| 3 | `9F 5A 5A` | KEY1 off, PNL:651 |

### 2. Exit: `crown_a3_s0_exit_cmdtbl`, PNL:1365-1380

| # | bytes | 4.9 |
|---|---|---|
| 4 | `F0 5A 5A` | MDNIE_AFC_OFF_SEQ, panel_mdnie.h:2085-2089, KEY2 on PNL:649 |
| 5 | `E2 00` | AFC off, panel_mdnie.h:2056 (`S6E3HA8_AFC_REG` 0xE2, s6e3ha8.h:274) |
| 6 | `F0 A5 A5` | KEY2 off, PNL:652 |
| – | wait 20 ms | `wait_afc_off`, PNL:1203 |
| 7 | `9F A5 A5` | KEY1 on |
| 8 | `F0 5A 5A` | KEY2 on |
| – | read `EE` (1 byte) | DUMP_ERR_FG, s6e3ha8.h:129 (logged as "exit EE") |
| 9 | `F0 A5 A5` | KEY2 off |
| – | read `05` (1 byte) | DUMP_DSI_ERR, s6e3ha8.h:133 |
| 10 | `10` | SLEEP_IN, PNL:655 |
| 11 | `9F 5A 5A` | KEY1 off |
| – | wait 100 ms | `wait_sleep_in`, PNL:1204 |

### 3. Level 2 only: reset pulse (`__set_panel_power`, DRV:119-160)

1. gpa3-4 low: `gpiod_set_raw_value(0)`, which is a DAT read-modify-write of bit 4. Readback: CON unchanged, the other output pins of gpa3 unchanged, bit 4 = 0.
2. Wait 10 ms (the 10 ms after the rails come on).
3. gpa3-4 high, with the same readback.
4. Wait 5 ms.

### 4. Init: `crown_a3_s0_init_cmdtbl`, PNL:1224-1260

| # | bytes | 4.9 |
|---|---|---|
| – | wait 5 ms | `wait_5msec`, PNL:1201 |
| 12 | `9F A5 A5` | KEY1 on |
| 13 | DSI type 0x07, data `01` | `crown_a3_s0_dsc` PNL:964, `dsc_table[1]` PNL:416-419, `getidx_dsc_table` (s6e3ha8.c:1269-1281; DT49 `s6e3ha8_crown_ddi` dsc_en = 1, line 12162) |
| 14 | DSI type 0x0A, 128 bytes, see below | `crown_a3_s0_pps` PNL:966, `pps_table[0]` PNL:421-440 via `getidx_resolution_table` (s6e3ha8.c:1283-1307, DSU mode 1). The static `CROWN_A3_S0_PPS` (PNL:682-700) is the 1080x2220 row, which the maptbl overwrites. |
| 15 | `9F 5A 5A` | KEY1 off |
| 16 | `11` | SLEEP_OUT, PNL:654 |
| – | wait 10 ms | `wait_sleep_out`, PNL:1202 |
| – | read `0F` (1 byte) | DUMP_SELF_DIAG, s6e3ha8.h:137 |
| 17 | `F0 5A 5A` | KEY2 on (SUPPORT_DSU) |
| 18 | `BA 01 26 08 08 F3` | SCALER, `scaler_table[0]` PNL:481-488, packet PNL:970 |
| 19 | `F0 A5 A5` | KEY2 off |
| 20 | `2A 00 00 05 9F` | CASET, `caset_table[0]` PNL:490-497 |
| 21 | `2B 00 00 0B 8F` | PASET, `paset_table[0]` PNL:499-506 |
| 22 | `9F A5 A5` | KEY1 on |
| 23 | `1A 1F 00 00 00 00` | SET_AREA, PNL:702-704 |
| 24 | `35 00` | TE_ON, PNL:745 |
| 25 | `9F 5A 5A` | KEY1 off |
| 26 | `F0 5A 5A` | KEY2 on |
| 27 | `ED 4C` | ERR_FG, PNL:747 |
| 28 | `FC 5A 5A` | KEY3 on, PNL:650 |
| 29 | `C5 0D 10 B4 3E 01` | FFC, PNL:748-751 |
| 30 | `FC A5 A5` | KEY3 off, PNL:653 |
| 31 | `B9 01 B0 91 09 00 00 00 11 03` | TSP_HSYNC, PNL:753-756 |
| 32 | `B0 03` then `F6 43` | ISC, PNL:827, packet offset 3 at PNL:1125 |
| 33 | `C2 09 00 D8 D8 F4 F4` | EDGE_DIMMING, PNL:829 |
| 34 | `F0 A5 A5` | KEY2 off |
| 35-49 | PANEL_SET_BL_SEQ for 44700 (15 packets) | `panel_bright.c` `pb_build()`. The set-bl table is listed in panel-brightness.md (F0 5A 5A, CA <gamma>, B1 00 0C, B5 19 DC 00, B0 16/B5 <et1>, F4 EB 26, B4 00 55 80 65 26 26, B0 0F/B4 20, 55 02, B0 12/B8 <IRC>, B8 15, B0 06/B1 <POC1>, B0 53/B8 00 00, F7 03, F0 A5 A5) |
| 50 | `F0 5A 5A` | COPR_SET_SEQ, `crown_a3_s0_set_copr_cmdtbl`, panel_copr.h:59-63 |
| 51 | `E1 07 15 15 00 00 00 00 00 00 FF FF 00 00 00 00 00 00 00 00` | `copy_copr_maptbl()` s6e3ha8.c:1519-1559 of the defaults in panel_copr.h:98-122 (ilc, gamma 2.2, enabled, E 0x100, max 0xFFFF, no ROI). The static array (panel_copr.h:42-47) has 00 01 for the max count, which the maptbl overwrites. |
| 52 | `F0 A5 A5` | KEY2 off |

PPS (packet 14), `pps_table[0]` "DSU MODE 1: 1440x2960 (Original), Slice Info: 720x40":

```
11 00 00 89 30 80 0B 90 05 A0 00 28 02 D0 02 D0
02 00 02 68 00 20 04 6C 00 0A 00 0C 02 77 01 E9
18 00 10 F0 03 0C 20 00 06 0B 0B 33 0E 1C 2A 38
46 54 62 69 70 77 79 7B 7D 7E 01 02 01 00 09 40
09 BE 19 FC 19 FA 19 F8 1A 38 1A 78 1A B6 2A F6
2B 34 2B 74 3B 74 6B F4 00 00 00 00 00 00 00 00
00 x 32
```

End of window 1. Frames run again. The driver waits until DECON
FRAME_COUNT moves (up to 500 ms), because 4.9 sends display on at the first
FRAME_DONE.

### 5. Display on: window 2, `crown_a3_s0_display_on_cmdtbl`, PNL:1353-1357

| # | bytes |
|---|---|
| 53 | `9F A5 A5` |
| 54 | `29` (DISPLAY_ON, PNL:657) |
| 55 | `9F 5A 5A` |

### 6. Status: window 3, 100 ms later (read only, plus key 2)

`pb_readback()`: `F0 5A 5A`, then the CA/B1/B5/F4/B8 snapshot and keyed
`EE`/`EA`, then `F0 A5 A5`, then `0A 0E 05 0F EE EA`.

Then one line:
`panel-recovery: RESULT PASS|FAIL: level, first error, packets (failed), set-bl, reset, frames before display on, read-back, RDDPM after, EE (key2) before/after, ...`

PASS means: no transmit error, RDDPM `& 0x7c` = 0x1c (normal, display on), and keyed EE with no ELVDD, VLOUT3 or VLIN1 bit.

The 4.9 byte arrays were cross-checked against the compiled
`build/close-range-fix-4.9/.../s6e3ha8/s6e3ha8.o`:
- PPS row 0, the scaler, CASET, PASET, SET_AREA, FFC, ISC, ERR_FG and the static COPR array appear 4 times (the four panel variants).
- TSP_HSYNC and EDGE_DIMMING appear once (crown only).

## Error handling

- Before the first write, any failed precondition gives `RESULT REFUSED (<why>)`.
- From the first write on, every table sends every packet. After an error, the table still sends its outstanding key offs (levels 3, 2, 1). Every later phase still runs, so the panel ends up as far "on" as possible.
- There is no retry. 4.9 power-cycles and retries up to 30 times (DRV:1290-1300).
- Only the tables above are allowed, matched byte for byte against the static tables. Gate: `d->pr_active`, set only around the tables. `dsim_cmd.c` refuses DCS 10/11/28/29/2A/2B/35 and DSI types 0x07/0x0A whenever `pr_active` is clear.
- It runs once per boot (`pr_done`).
- `panel_lock` serialises it with the brightness, status and ALPM works.
- `dpu_stop()` cancels it if it has not started. If it has started, `dpu_stop()` waits until the display on and status have been sent.

## Not matched exactly to 4.9

- **Rails and DSIM:** the S2DOS05 rails stay on (level 3 is not approved) and so does the DSIM link (adopt mode). 4.9 turns the PHY off, resets the DSIM and re-initialises it to LP-11 between the exit and init tables. Here the link stays in command-mode LP-11 with the TE trigger masked.
- **Reset timing (level 2):** reset is held low for 10 ms. On 4.9 it stays low for the whole time the screen is off (at least the rail off/on time plus 10 ms).
- **Retries:** 4.9's disp-det poll after init (20 ms) and the power-cycle retry loop are not reproduced. The 6.12 DT has no disp-det GPIO.
- **After display on:** `mdnie_enable()` → `mdnie_update()` sends the mDNIe tables, and `copr_enable()` may clear the COPR counters (`SUPPORT_COPR_AVG`). Neither is sent here. After a level 2 reset the mDNIe block is therefore at its power-on default.
- **fix_green:** the ALPM cycle that runs before display on is separate (`samsung,alpm-cycle-at-ms`) and is left out of v19a/v19b.
- **Assumptions:** COPR ROI is off and DSU mode is 1 (WQHD, as SBoot sets it). The owner's Lineage could only change these through Samsung sysfs nodes.
