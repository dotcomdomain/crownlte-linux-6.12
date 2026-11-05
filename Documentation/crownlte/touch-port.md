# crownlte touchscreen (sec_ts Y771), stage 1

Status: build-only, not yet run on the phone.

Driver `drivers/input/touchscreen/sec_ts_y771.c` (`CONFIG_TOUCHSCREEN_SEC_TS_Y771`,
compatible `samsung,sec-ts-y771`). DT nodes are disabled in
`exynos9810-crownlte-clocks.dtsi` and enabled, with the self-test, in
`exynos9810-crownlte-delayed-ufs-diag.dts`.

`K49` below is the 4.9 kernel running on the phone (`/root/crownlte-hid-kernel`
on gekko); `TS` is `K49/drivers/input/touchscreen/sec_ts/y771`; `DT49` is
`K49/arch/arm64/boot/dts/exynos/exynos9810-crownlte_eur_open_26.dts`.

## Hardware, from 4.9

| Item | 4.9 value | Evidence |
|---|---|---|
| Controller | sec_ts, `img_ver[0] == 0x17` means "Y771" | `TS/sec_ts_fn.c:3396-3413`, defconfig `CONFIG_TOUCHSCREEN_SEC_TS_Y771=y` (`K49/arch/arm64/configs/exynos9810-crownlte_defconfig:343`) |
| Bus | `hsi2c@104B0000` (alias hsi2c10), SPI 404, address 0x48 | `DT49:6012-6056` |
| Bus options | `samsung,stop-after-trans`, `samsung,reset-before-trans`, no clock-frequency (so 400 kHz fast mode), `default-clk` 200 MHz | `DT49:6029-6030`; `K49/drivers/i2c/busses/i2c-exynos5.c:172,737,947,1065-1084` |
| Bus clocks | rate 224 = `USI03`, gate 205 = `GATE_USI03` | `DT49` clocks `<26 224 26 205>`; `include/dt-bindings/clock/exynos9810.h` (same numbering in 4.9 and 6.12) |
| Bus pins | gpp1-4/gpp1-5, function 2 (`hsi2c10-bus`) | `DT49:1835-1841` |
| USI mode | PERIC0 SYSREG 0x1041101C = `I2C_SW_CONF` (BIT 2) | `DT49:5434-5440`; `K49/drivers/soc/samsung/usi_v2.c:18,99` |
| IRQ | gpa1-0 (wakeup EINT, GIC SPI 44), `sec,irq_type = 8200` = 0x2008 = `IRQF_ONESHOT \| IRQF_TRIGGER_LOW`; pin state "on_state" = function 0xf, no pull | `DT49:6043-6044`, `DT49:345-351` |
| Supplies | `tsp_io` = S2MPS18 LDO35 1.8 V, `tsp_avdd` = LDO43 3.0 V, both `regulator-boot-on`; driver has `sec,regulator_boot_on` | `DT49:6048-6050`, `DT49:8762-8782` |
| Other | `sec,force_sensor_ch_gpio` on the PCAL6524 expander (pin 21), not used in stage 1 | `DT49:6042` |

6.12: the bus is served by `i2c-exynos5` with the `samsung,exynos9810-hsi2c`
compatible (the same driver and 9810 timing code that already runs the MAX77705
bus at 0x14360000 on the phone), under an `exynos-usi` wrapper
(`samsung,exynos9810-usi`) at 0x104b00c0, which writes the USI03 protocol
selector. Clocks come from `legacy_cmu` (`GATE_USI03`, `USI03`). The IRQ uses
the 9810 alive pinctrl wakeup EINT (gpa1 is an `EINTW` bank).

## What 4.9 does at boot, and what stage 1 keeps

4.9 `sec_ts_probe()` (`TS/sec_ts.c:2245-2654`):

1. power on via regulators (`sec_ts_power()`, `TS/sec_ts.c:1715-1775`); 70 ms delay
   skipped because of `regulator_boot_on` (`TS/sec_ts.c:2394`);
2. `sec_ts_wait_for_ready()` (`TS/sec_ts.c:720-755`, call at 2399): reads events
   while the IRQ line is low;
3. reads device id 0x22, integrity 0x21, boot status 0x55, TS status 0xAF
   (`TS/sec_ts.c:2403-2441`);
4. **firmware update on probe** (`TS/sec_ts.c:2451-2456`, `SEC_TS_FW_UPDATE_ON_PROBE` at
   `TS/sec_ts.h:145`, `TS/sec_ts_fw.c:745-866`): request_firmware, version
   compare, flash erase/write (`TS/sec_ts_fw.c:267,320`), then TCLM calibration;
5. `sec_ts_read_information()` (`TS/sec_ts.c:2055-2140`): 0x52, 0x23, 0x55, 0xAF, 0x30;
6. writes SET_TOUCHFUNCTION 0x30 with `| 0x61` (`TS/sec_ts.c:2469-2470`);
7. writes force sensor version 0xBE (only with the expander GPIO);
8. writes SENSE_ON 0x10 (`TS/sec_ts.c:2484`);
9. registers "sec_touchscreen" (`TS/sec_ts.c:2501`), request_threaded_irq (2524);
10. sponge library writes 0x91/0x93, `sec_ts_read_evt_info()` (flash-read pointer
    writes 0xD0/0xD1, `TS/sec_ts_fn.c:3202`), and 50 ms later
    `sec_ts_read_info_work()`: TCLM calibration check and raw-data tests
    (`TS/sec_ts.c:2571-2574,2889-2913`).

Stage 1 keeps 1-3 (minus power), 5, 6, 8, 9. Not ported at all: 4 (no
`sec_ts_fw.c`, no `request_firmware`), 7, 10, and every sysfs/factory command
(`sec_ts_fn.c`, which contains the NVM 0x85 writes at `TS/sec_ts_fn.c:4463-6868`).
If the IC is in boot mode, has bad integrity or reports flash mode (the cases where
4.9 forces a flash), stage 1 logs and stops without writing anything.

## Exact I2C traffic

Every transaction goes through `sec_ts_read()` / `sec_ts_write()`, which refuse
any opcode outside a fixed allowlist and count refusals. Reads are sent like 4.9
on this bus (stop-after-trans): a 1-byte opcode write ending in STOP, then a
separate data read.

Reads (host sends only the opcode byte):

| Opcode | Len | Meaning | When |
|---|---|---|---|
| 0x22 | 5 | device id | probe, self-test |
| 0x21 | 1 | firmware integrity | probe |
| 0x55 | 1 | boot status (0x20 app / 0x10 boot) | probe, self-test |
| 0xAF | 4 | TS status | probe, self-test |
| 0xA5 / 0xA3 / 0xA4 | 4 each | image / core / config version | probe, self-test (0xA5) |
| 0x52 | 3 | id | probe |
| 0x23 | 11 | panel info (resolution, tx/rx) | probe |
| 0x30 | 2 | get touch functions | probe |
| 0x60 | 8 | pop one event | probe (only while the IRQ line is low), every IRQ |
| 0x61 | 8 x n | pop n remaining events | IRQ |

Writes (the only ones that exist):

| Opcode | Data | When | Why it is RAM-only |
|---|---|---|---|
| 0x30 SET_TOUCHFUNCTION | 2 bytes: value read from 0x30 with bits 0x61 (touch, palm, wet) set | probe, only if those bits are missing; after an IC self-reset event | 4.9 re-sends it after every power-on (`sec_ts_start_device()`, `TS/sec_ts.c:3482`), after IC watchdog reset (`sec_ts_reinit()`, `TS/sec_ts.c:1073,806`) and on cover changes (`TS/sec_ts.c:1580`). A setting stored in flash would not need restoring after every power cycle. |
| 0x10 SENSE_ON | none | probe; after an IC self-reset event (status INFO, id 0, data 0x20) | 4.9 sends it after every power-on (`TS/sec_ts.c:3552`) and after IC watchdog reset (`TS/sec_ts.c:1070`); `sec_ts_stop_device()` simply removes power. Scanning state does not survive power-off. |

Opcodes that write flash/NVM or calibrate in 4.9 and are never sent:
0x13/0x14 (calibration), 0x45 (erase), 0x57 (enter FW mode), 0x85 (NVM),
0x8F (offset cal), 0xD0-0xDA (flash read pointer/erase/write/padding),
0xE9 (pressure cal), 0x62 (clear event stack: 4.9 uses it on queue overflow;
stage 1 instead pops one event per interrupt until the backlog fits).

## Power-on sequence needed later (stage 2)

Implemented, DT opt-in (`samsung,acpm-touch-power` + S2MPS18 `samsung,touch-rail-update`):
see `power-gates.md`.

4.9 `sec_ts_power()` (`TS/sec_ts.c:1715-1775`): enable `tsp_io` (LDO35, 1.8 V),
wait 1 ms, enable `tsp_avdd` (LDO43, 3.0 V); off is the reverse with 4 ms between.
After power-on, 70 ms (`TS/sec_ts.c:3450-3451`) then `sec_ts_wait_for_ready()` for
the BOOT_COMPLETE status event (up to 100 x 20 ms).

S2MPS18 registers (4.9 `K49/include/linux/mfd/samsung/s2mps18-private.h:142,150`,
`K49/drivers/regulator/s2mps18.c:534-551`): L35CTRL 0x6C (MIN1 0.7 V, 25 mV steps:
1.8 V = vsel 0x2C), L43CTRL 0x74 (MIN3 1.8 V, 25 mV steps: 3.0 V = vsel 0x30);
enable = bits 7:6 (`S2MPS18_ENABLE_SHIFT` 6; 0b11 = on in normal mode). Both are
PMIC block writes over ACPM, which the 6.12 ACPM write gate refuses; a stage-2
policy would have to allow exactly these two registers. The q23 PMIC self-test
dump (`pmic[0x69..0x76]` line) shows whether sboot left them on: if on, bits 7:6 are non-zero, e.g. L35CTRL
0xEC and L43CTRL 0xF0 (a GUESS from the 4.9 voltages; vsel is bits 5:0).

## Expected log (diag DTB, IC powered by the bootloader)

```
sec_ts N-0048: stage 1 (no firmware update, no power control), irq NN, line 1
sec_ts N-0048: device id xx xx xx xx xx   (not known before the trial)
sec_ts N-0048: integrity 0x80 boot 0x20 status ...
sec_ts N-0048: chip Y771, image 17 .. .. .. core .. config ..
sec_ts N-0048: id ..., panel 1440x2960 display 1440x2960 tx NN rx NN, functions ..
sec_ts N-0048: wrote op 0x30 data 61 00: 0        (only if bits were missing)
sec_ts N-0048: wrote op 0x10: 0
sec_ts N-0048: input sec_touchscreen, 1440x2960, 10 slots
sec_ts N-0048: running
sec_ts N-0048: [P] tID:0 x:... y:... major:.. minor:.. type:0   (first 32 touches)
touch selftest: start uptime=109xxx ms state=running addr=0x48
touch selftest: chip id ... (probe ...) stable
touch selftest: chip Y771 fw image ... core ... config ... (now ...) boot 0x20 status ...
touch selftest: events since boot: press=N move=M release=R status=S gesture=0 irqs=I idle_irqs=K overflows=0 ic_resets=0
touch selftest: irq line=1 (idle) irq enabled, fingers down=0, i2c errors=0
touch selftest: IC writes: SENSE_ON(0x10)=1 SET_TOUCHFUNCTION(0x30)=0|1, refused=0; no firmware/flash/NVM/calibration code built
touch selftest: PASS
```

The I2C bus number depends on probe order. If the IC does not answer:
`no answer at 0x48 (-6|-110): IC not powered? ...` and the self-test prints
`state=absent` and `FAIL`.

## Not done in stage 1

- `/sys/class/sec/tsp` (sec_cmd: cmd, cmd_list, cmd_result) used by the Lineage touch
  HAL for gestures/glove; touch itself is plain evdev. The vendor has no
  `sec_touchscreen.idc` (only `/vendor/usr/idc/sec_e-pen.idc`), so Android
  classifies the device by `INPUT_PROP_DIRECT`.
- Suspend/resume, LPM/AOD, sponge gestures, force sensor, `ABS_MT_CUSTOM`,
  `KEY_BLACK_UI_GESTURE`, secure touch.
- 4.9 `reset-before-trans` (HSI2C reset only after arbitration loss); 6.12 resets on
  timeout.
