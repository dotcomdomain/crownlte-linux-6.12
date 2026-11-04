# crownlte: 4.9 fault-display brightness on 6.12

The owner's S6E3HA8 panel looks black below a high brightness. Their 4.9
kernel works around it in `drivers/video/fbdev/exynos/panel/panel_bl.c`:

| 4.9 commit | effect |
|---|---|
| cf5bb86c | `PANEL_FAULT_DISPLAY_BRIGHTNESS 44700`: normal display state forced to the calibrated maximum, via the panel's own tables, no voltage/timing limit change |
| 4cb674aa, 34118204 | prepare at probe without writes; best-effort early update of an inherited active panel |
| d5ef7fbb | policy becomes a floor (16691) that keeps higher requests |
| 13cdc166 | the clamp also covers the ALPM-exit window |
| 0f2943f3 | floor raised to 20000 (the running close-range-fix build has this) |

6.12 did not send any panel command, so SBoot's 12800 stayed and display
trials were probably invisible. This port sends the 44700 sequence once.

## Panel

4.9 log (`research/diagnostic-capture/android-after-restore-20261013/2-last_kmsg.bin`):
`s6e3ha8_panel_read_id READ ID : 91 20 47`, `find_panel_lut, found
s6e3ha8_crown_a3_s0_default (id:0x00912047 lut[6] ...)`, `panel_probe
dim_flash : on`, later `panel_bl_set_brightness bl-0 dim:flash ...`.
So: S6E3HA8, `crown_a3_s0_default` tables, DDI dim flash in use after boot.

## Brightness 44700 in 4.9

`44700 = BRT(447) = crown_a3_s0_brt_tbl[86]`, the extended-HBM row
(`lum 720`; `CONFIG_LCD_EXTEND_HBM` comes from `panel_bl.h`, so the tables
have 87 rows). `panel_bl_set_brightness()` runs `PANEL_SET_BL_SEQ` =
`crown_a3_s0_set_bl_cmdtbl` (`s6e3ha8_crown_a3_s0_panel.h:1303`).
`common_mipi_lcd.c mipi_write()` sends size 1 as DCS short write (0x05),
longer as DCS long write (0x39), and a packet offset as a DCS short write
with parameter `B0 <offset>` (0x15) first.

| # | bytes | source | fixed / calibrated |
|---|---|---|---|
| 1 | `F0 5A 5A` | level 2 key | fixed |
| 2 | `CA` + 34 bytes | `copy_gamma_maptbl` -> `generate_hbm_gamma` (`s6e3ha8.c:755`): 720 > 700 nit, so the HBM gamma resource is copied; round trip `gamma_ctoi_v0`/`copy_tpout_center_v0` keeps all bytes except bits 7..3 of byte 0 | **calibrated**: DCS read `B3` at offset 2, 34 bytes (`S6E3HA8_HBM_GAMMA_*`) |
| 3 | `B1 00 0C` | `aor_table[86]` (HBM: `copy_common_maptbl`) | fixed |
| 4 | `B5 19 DC 00` | TSET 25 C (`NORMAL_TEMPERATURE` at probe), `mps_table[1]` (lum > 39), `dimming_elvss_table[0][86]` | fixed |
| 5 | `B0 16`, `B5 xx` | `ELVSS_TEMP` at offset 22; `init_elvss_temp_table` gives rows >= 74 the OTP `elvss_temp_1` | **calibrated**: DCS read `B5` at offset 23, 1 byte |
| 6 | `F4 EB 26` | VGH fixed `EB`, `dimming_vint_table[0][86]` | fixed |
| 7 | `B4 00 55 80 65 26 26` | ACL: pwrsave on (`adaptive_control` default), HBM start point, `ACL_OPR_08P` (4.9 logs `acl:on(3)` in HBM) | fixed |
| 8 | `B0 0F`, `B4 20` | ACL dimming frames | fixed |
| 9 | `55 02` | ACL on | fixed |
| 10 | `B0 12`, `B8 25 69 D6 62 06 57 53 89 83 83 83 2C 2C 2C 2C 2C 2C` | `irc_interpolation()` at 44600 (ext-HBM is clamped to row 85): row 73 x 700/400 | fixed in table mode (see below) |
| 11 | `B8 15` | IRC on | fixed |
| 12 | `B0 06`, `B1 01 AF 54 68 CC 78 30 CC 64 FF` | POC comp 1 | fixed |
| 13 | `B0 53`, `B8 00 00` | POC comp 2, `poc_comp_table[86]`: the table has 86 initialisers for 87 rows, so row 86 is zero | fixed (4.9 quirk, replicated) |
| 14 | `F7 03` | gamma/LTPS update | fixed |
| 15 | `F0 A5 A5` | level 2 key off | fixed |

All fixed bytes were checked against the compiled 4.9 object of the running
build (`build/close-range-fix-4.9/.../s6e3ha8/s6e3ha8.o`, `.data` at the
`llvm-nm` symbol offsets, script `panel-bright-work/dump49.py` on gekko).

Not part of the sequence: no sleep in/out, display on/off, reset, TE, DSC,
PPS, CASET/PASET, MTP/OTP write, POC/flash or AVC/VGH changes (F4 byte 1 is
always the fixed `EB` 4.9 writes; the driver refuses if the panel reads back
something else first).

### IRC and the dim flash

4.9 starts in table mode (`dim:table` in its log) and ~0.5 s later its
dim-flash work reads the panel's DDI flash through POC commands and replaces
rows 0..85 of the AOR/VINT/ELVSS/IRC tables. Row 86 is outside the flash
range (86 rows), so at 44700 only the IRC packet depends on the flash (it is
derived from row 73). 6.12 sends the table-mode IRC, which is what 4.9 sends
before the flash is loaded. Reading the flash copy needs POC commands and is
not done.

## 6.12 implementation (branch crownlte-panel-bright)

- `dsim_cmd.c`: DSIM0 packet path, port of 4.9 `dsim_write_data`,
  `dsim_long_data_wr`, `dsim_read_data` and the `dsim_reg.c` FIFO helpers.
  Polled (no DSIM IRQ in 6.12), one packet in flight, 50 ms write / 100 ms
  read limits as 4.9, no link reset or retry, DCS power/sleep/display/
  reset/TE/address/memory commands and DSC/PPS/peripheral types refused.
  Writes only PKTHDR/PAYLOAD and W1C INTSRC status bits.
- `decon_drv.c dpu_cmd_window_begin/end`: commands only while DECON is
  adopted and powered, no shadow update pending, TE trigger masked and
  FRAME_COUNT unchanged for > 20 ms (no frame in flight); hw_lock held.
- `panel_bright.c`: preconditions (RDDPM must be normal display: sleep out,
  normal, display on, not idle/partial, as 4.9 `show_rddpm`; panel ID from
  DT; HBM gamma equal to the 4.9-logged `gamma[720]` from DT; `F4[0] = EB`),
  then the sequence above, then a read-back of CA/B1/B5/F4/B8. Every packet
  is logged with a counter (`panel-bright: tx#N`, `rx#N`).
- `panel_ro.c`: `/sys/class/backlight/panel/brightness` reports 44700 once
  set; changes are still refused.
- DT opt-in on `panel_drv@001`: `samsung,fault-display-brightness = <44700>`
  (only value implemented), `samsung,fault-display-brightness-at-ms`,
  `samsung,panel-id`, `samsung,hbm-gamma-expect`; `decon_f` gets the `dsim`
  register window and `samsung,panel`. Trial DTB:
  `exynos9810-crownlte-delayed-ufs-display-s2-bright-diag`.

Note: the running 4.9 build uses a 20000 floor (0f2943f3), not 44700. Other
values need the 4.9 AID dimming (MTP C8 + `process_dimming`) or the DDI
flash tables and are not implemented.
