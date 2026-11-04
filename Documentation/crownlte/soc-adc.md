# Exynos9810 SoC ADC and thermistors (crownlte)

Stage: build-only, not yet run on the phone.

The CMGP ADC at 0x14230000 (12 channels, 12 bit) carries the NTC
thermistors. 4.9 took the battery temperature from it and wrote it into
the MAX77705 fuel gauge. The "ac" power-supply temperature also came from
it. This port makes the ADC an IIO device and feeds the MAX77705 read-only
battery driver from it. The driver never writes the fuel gauge.

## Pieces

| piece | file | DT |
|---|---|---|
| ADC (IIO) | `drivers/iio/adc/exynos_adc.c`, new variant `samsung,exynos9810-adc` | `exynos9810.dtsi` `adc: adc@14230000` (disabled) |
| thermistor → temperature | `drivers/power/supply/max77705-ro-battery.c` | `exynos9810-crownlte-max77705.dtsi` `io-channels` + 4.9 tables |
| enable + self-tests | – | `exynos9810-crownlte-delayed-ufs-diag.dts` (`&adc` clock + `samsung,adc-selftest-ms`) |

Config: `CONFIG_IIO=y`, `CONFIG_EXYNOS_ADC=y` (`BATTERY_MAX77705_RO` now
depends on `IIO`).

## Channels used (4.9 crownlte_eur_open_26.dts)

| ch | 4.9 consumer | 4.9 name | 6.12 use |
|---|---|---|---|
| 4 | sec_battery `battery,thermal_source = <2>` (ADC) | `adc-temp` (also `adc-wpc-temp`) | `battery/temp`, thermal zone `battery` |
| 3 | sec_battery `chg_thermal_source = <1>` | `adc-chg-temp` | `ac/temp`, thermal zone `ac` |
| 1 | sec_battery `usb_thermal_source = <2>` | `adc-usb-temp` | `battery/usb_temp`, self-test |
| 11 | sec_battery | `adc-in-bat` | raw only (self-test) |
| 0 / 2 / 10 | sec_thermistor ap / wf / pa (hwmon) | `adc-ap-temp` … | raw only (self-test); not ported, the thermal HAL does not read them |

**Correction to the earlier battery doc:** on 4.9 the `ac` power supply's
TEMP is `battery->chg_temp`, the **charger** thermistor on ch3
(sec_battery.c:4792-4793 in `sec_ac_get_property`). It is not the
USB-connector thermistor on ch1. The thermal HAL's `ac` sensor (Type
USB_PORT, EMERGENCY 60 °C, Monitor) therefore watched the charger NTC on
4.9, and it does again here.

## 4.9 evidence (/root/crownlte-hid-kernel)

* DT `arch/arm64/boot/dts/exynos/exynos9810-crownlte_eur_open_26.dts`
  * :9419-9429 `adc@14230000`: `samsung,exynos-adc-v3`, reg 0x14230000+0x100,
    `interrupts = <0 8 0>`, `clocks = <26 102>` "gate_adcif" (26 = the
    `samsung,exynos9810-clock` node at :4988, 102 = `GATE_ADC_CMGP`), no
    `samsung,adc-phy-control`, no `sysreg` (no CHUB semaphore), no vdd.
  * :13395-13429 sec_thermistor ap/wf/pa (ch0/2/10) tables.
  * :13618-13636 battery node: `temp_adc_type = <1>`, `thermal_source = <2>`,
    `usb_thermal_source = <2>`, `chg_thermal_source = <1>`, the tables
    `temp_table_*` (:13629-13630), `usb_temp_table_*` (:13631-13632) and
    `chg_temp_table_*` (:13633-13634), 39 points each.
  * :13646 `battery,adc_check_count = <5>`, :13656 `temp_check_count = <1>`.
  * :13770-13771 `io-channels = <410 4 410 3 410 11 410 4 410 1>` with names
    adc-temp, adc-chg-temp, adc-in-bat, adc-wpc-temp, adc-usb-temp.
* ADC driver `drivers/iio/adc/exynos_adc.c`
  * :78-81, :129-132 V3 registers (DAT at 0x08), 12 channels.
  * :513-528 `exynos_adc_v3_init_hw`, :530-540 `exit_hw`, :542-557 ISR,
    :559-568 v3 data; :469-486 `v2_clear_irq` / `v2_start_conv`.
  * :643-724 `exynos_read_raw`: enable access, init_hw, start, wait 100 ms,
    exit_hw, disable access, for every conversion.
  * :242-290 `enable_access`/`disable_access`: idle-ip, optional PMU phy write (off on
    crownlte), regulator (none), clk prepare+enable of "gate_adcif".
* Clock: `drivers/clk/samsung/clk-exynos9810.c:103` `HWACG_VCLK(GATE_ADC_CMGP,
  ADC_CMGP_QCH_ADC, ..., NULL, 0, VCLK_GATE, NULL)`;
  `drivers/clk/samsung/composite.c:1375` `cal_vclk_enable` returns 0 at once
  for VCLK_GATE without VCLK_QCH_DIS (same in 6.12
  `clk-exynos9810-composite.c:1375`/:1405).
* Conversion `drivers/battery_v2/sec_adc.c`
  * :50-71 `sec_bat_adc_ap_read` (RETRY_CNT 3, `include/sec_adc.h:29`, prev value on failure).
  * :173-207 `sec_bat_get_adc_data` (count ≥ 3, drop max and min, average;
    "else if" quirk kept).
  * :254-356 `sec_bat_get_value_by_adc` (binary search, linear interpolation).
* Use `drivers/battery_v2/sec_battery.c`
  * :2637 `sec_bat_get_temperature_info`, ADC branch :2667-2745: battery
    ch, usb, chg, wpc; then writes temperature into the FG
    (`psy_do_property(fuelgauge, set, TEMP)` :2742-2746). **Not ported.**
  * :69-74 `sec_ac_props` (ONLINE, TEMP, VOLTAGE_MAX, CURRENT_MAX),
    :4792-4793 ac TEMP = `chg_temp`.
  * :1896-1935 usb-thermistor high-limit (80 °C → OVERHEATLIMIT) and battery
    high/low health: charger actions, not thermal trips.
* `drivers/power/supply/power_supply_core.c:591-606`: every psy with TEMP
  got `thermal_zone_device_register(name, 0, 0, ...)`: **0 trips**, no
  polling. 4.9 had no kernel-side battery/ac thermal trips, so none are
  added here. The HAL's `battery` SHUTDOWN 60 °C and `ac` EMERGENCY 60 °C are
  userspace actions, as on 4.9.

## Register writes

Probe: none (reads CON1, CON2, INT_EN, INT_ST once for the log).

Every conversion (4.9 v3 sequence, counted by `exynos9810_adc_writel`):

| # | register | value |
|---|---|---|
| 1 | CON1 0x00 | 0x4 (SOFT_RESET) |
| 2 | CON1 0x00 | 0x2 (SOFT_NON_RESET) |
| 3 | CON2 0x04 | 0x60 (C_TIME 6) |
| 4 | INT_EN 0x10 | 1 |
| 5 | CON2 0x04 | RMW ACH_SEL = channel |
| 6 | CON1 0x00 | RMW \| EN_START |
| 7 | INT_ST 0x14 | 1 (in the IRQ, after reading DAT 0x08) |
| 8 | CON2 0x04 | RMW clear C_TIME |
| 9 | INT_EN 0x10 | 0 |

No other register anywhere: no PMU (no ADC-PHY bit), no CMU, no CHUB
sysreg, no regulator, no MAX77705 register.

Load: the battery driver converts 3 thermistors × 5 samples at every poll
(10 s with a source, 30 s without), the same number of thermistor
conversions 4.9 made for battery, usb and chg (4.9 also read ch4 again for wpc
and ambient). Userspace reads of `/sys/bus/iio/devices/iio:deviceN/in_voltageN_raw`
also convert. All conversions are serialized by the driver mutex.

## Clock handling

`clocks = <&legacy_cmu GATE_ADC_CMGP>` is 4.9's "gate_adcif". It is a CMU_CMGP
HWACG Q-channel vclk. Its `.enable`/`.disable` return before calling CAL
(VCLK_GATE without VCLK_QCH_DIS), it has no parent, and it is registered
with CLK_IGNORE_UNUSED. So:

* `clk_prepare_enable` at probe and `clk_disable` at suspend/remove
  write no CMU register;
* clk_disable_unused never touches it;
* no PERIS/PERIC (or any) bus clock or mux is referenced, so nothing can
  be "released" and switch a bus mux.

The block stays under hardware auto clock gating, as on 4.9. Upstream keeps the
clock enabled while bound; 4.9 enabled it per conversion. Both are no-ops for this
vclk, so the upstream handling is kept.

## Temperatures

`battery/temp` (0.1 °C) = ch4 through the 4.9 battery table when the ADC
read works; otherwise the FG TEMP register (stale: CONFIG.TEX=1 means the host
must write it, and 6.12 never does). `ac/temp` = ch3 through the 4.9 chg
table. Thermal zones `battery` and `ac` come from the power_supply core
(tripless, value ×100 = m°C) and read the last polled value, as 4.9 did.
Battery health OVERHEAT (≥ 50.0 °C, recovery 45.0) and COLD (≤ 0.0, recovery
5.0) now use the ADC temperature.

Extra read-only attributes on `battery` (4.9 sec_battery_sysfs names):
`batt_temp`, `batt_temp_adc`, `chg_temp`, `chg_temp_adc`, `usb_temp`,
`usb_temp_adc` (last polled value and trimmed-mean ADC code).

## Self-tests (diag DTB)

ADC driver, 104 s (`&adc samsung,adc-selftest-ms = <104000>`):

```
exynos-adc 14230000.adc: 12 channels, programmed per conversion; idle regs CON1=........ CON2=........ INT_EN=........ INT_ST=........
exynos-adc 14230000.adc: adc selftest armed for 104000 ms after boot
exynos-adc 14230000.adc: adc selftest: regs before: CON1=.. CON2=.. DAT=.. INT_EN=.. INT_ST=.. VER=..
exynos-adc 14230000.adc: adc selftest: ch0  raw NNNN NNNN NNNN NNNN NNNN (spread N) NNNus/5
... ch1 .. ch11
exynos-adc 14230000.adc: adc selftest: regs after: CON1=.. CON2=.. INT_EN=00000000 INT_ST=..
exynos-adc 14230000.adc: adc selftest: conversions=60 (total N) timeouts=0 errors=0 rail-codes=K mux-varies=1 reg_writes=540 (9 per conversion, total N) -> PASS
```

PASS: no timeouts or errors, the codes differ between channels (the mux
works), exactly 9 register writes per conversion. `rail-codes` counts channels
reading 0 or 4095 (an unconnected input). It is informational.

Battery driver, 104.5 s (`&max77705_battery samsung,adc-selftest-ms = <104500>`):

```
max77705-ro-battery N-0036: SoC ADC thermistor batt: ch4 (adc-temp), 39-point 4.9 table, 5 samples per read; first read raw NNNN -> TT.TC
max77705-ro-battery N-0036: SoC ADC thermistor chg: ch3 (adc-chg-temp), ...
max77705-ro-battery N-0036: SoC ADC thermistor usb: ch1 (adc-usb-temp), ...
max77705-ro-battery N-0036: battery temperature source: SoC ADC battery NTC; ac temperature: SoC ADC charger NTC (thermal zone "ac"); FG TEMP is not written
...
max77705-ro-battery N-0036: adc selftest: batt ch4 raw a b c d e -> avg X -> TT.TC (table 313..3765 = 90.0..-25.0C) reads=.. errors=0
max77705-ro-battery N-0036: adc selftest: chg ch3 raw ... (table 294..3753 = 90.0..-25.0C) ...
max77705-ro-battery N-0036: adc selftest: usb ch1 raw ... (table 303..3756 = 90.0..-25.0C) ...
max77705-ro-battery N-0036: adc selftest: batt ADC TT.TC vs FG TEMP FF.FC (reg xxxx, CONFIG.TEX=1: host-written, stale, UNCHANGED since probe) diff D.DC; FG TEMP is never written by 6.12
max77705-ro-battery N-0036: adc selftest: thermal zones "battery"=TT.T00 mC "ac"=CC.C00 mC (power_supply TEMP x100, tripless)
max77705-ro-battery N-0036: adc selftest: thermistors wired=3 ok=3 max77705_writes=0 -> PASS
```

PASS: every wired thermistor read without error, inside its table (not
clamped: an open or shorted NTC clamps to -25.0 or 90.0 °C) and within
-20…80 °C. Zones `battery` and `ac` must exist and read. No MAX77705 writes.
A phone resting at room temperature should show batt, chg and usb within a
few °C of each other and near the TMU zones minus the SoC self-heating. FG TEMP
is the value 4.9 last wrote before the reboot, so expect it to differ.

The 107 s `battery selftest:` temp line now prints the source (SoC ADC)
and the FG TEMP separately.

## Risks / open points

1. Untested on hardware. The register layout and sequence are from 4.9
   v3 and were not reverse-checked against a datasheet. If the IRQ never
   arrives, every read times out after 100 ms. The self-test then FAILs
   with `timeouts`, and the battery falls back to FG TEMP. Each poll can
   then hold the battery lock for up to 3 × 3 × 100 ms.
2. The `ac` zone is the charger NTC (ch3), as on 4.9, not the USB
   connector (ch1). USB-connector temperature is in `battery/usb_temp`
   and the self-test only.
3. No kernel trips (4.9 had none). The 4.9 kernel-side protections were
   charger actions: battery OVERHEAT 50 °C and COLD 0 °C stop charging,
   and USB-NTC ≥ 80 °C (OVERHEATLIMIT) cuts charging/VBUS. The RO driver
   cannot do either. It reports health OVERHEAT/COLD only. Charging stays
   under the MAX77705's own control and its junction regulation.
4. CHUB: 4.9 crownlte had no `sysreg` semaphore, so the AP did not
   arbitrate ADC access with the sensor hub. 6.12 does not run CHUB
   firmware.
5. 4.9 marked the ADC busy for SICD (`exynos_update_ip_idle_status`).
   6.12 has no SICD. If system idle is ported later, register the ADC as
   an idle IP.
6. The battery monitor now defers until the ADC probes. Enable both
   (`&adc` with its clock, and `&max77705_battery`) or neither.
7. sec_thermistor ap/wf/pa (ch0/2/10) are not exposed (no HAL reader).
   Their 4.9 tables are at dts:13395-13429.
