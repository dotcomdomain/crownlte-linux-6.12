# MAX77705 battery monitor, stage 1 (read-only)

Driver: `drivers/power/supply/max77705-ro-battery.c` (`CONFIG_BATTERY_MAX77705_RO`).
DT: `exynos9810-crownlte-max77705.dtsi` (node `battery-monitor@36` on `hsi2c_pmic`,
disabled), enabled with the self-test in `exynos9810-crownlte-delayed-ufs-diag.dts`.

This stage never writes to the MAX77705. The battery keeps whatever charger
and fuel-gauge settings it had when 6.12 started (bootloader settings, or the
4.9 shutdown settings, see "Risks").

## What userspace sees

| power_supply | type | attributes | source |
|---|---|---|---|
| `battery` | Battery | status, charge_type, health, present, online, technology, voltage_now, voltage_avg, voltage_ocv, current_now, current_avg, charge_full, charge_full_design, charge_counter, cycle_count, capacity, temp | FG + charger reads |
| `ac` | Mains | online, voltage_max, current_max | CHG_INT_OK.CHGIN_OK, BC1.2 type != SDP/CDP |
| `usb` | USB | online, voltage_max, current_max | CHGIN_OK and BC1.2 SDP/CDP |
| `wireless` | Wireless | online, present, voltage_max, current_max | CHG_INT_OK.WCIN_OK |

The power_supply core adds the tripless thermal zones `battery` (temp in
m°C = battery temperature × 100) and, with the SoC ADC wired, `ac` (charger
thermistor × 100). They have no trip points, so they never trigger any
action (4.9 registered them with 0 trips as well). Temperatures come from the
SoC ADC thermistors with the 4.9 tables; see `soc-adc.md`.

Units match the 4.9 sysfs that Lineage 4.9 reads: voltage in µV, temp in
0.1 °C, charge_* in µAh, battery/online = 4.9 `sec_battery_cable` number
(1 none, 3 TA, 4 USB, 5 CDP, 10 wireless). current_now/current_avg are in
**mA** like 4.9 (`samsung,legacy-current-ma`; remove that property to get µA).
cycle_count = FG Cycles / 100 and charge_counter = DesignCap × capacity(0.1 %),
as in the customised 4.9 tree.

Health HAL (V:/bin/hw/android.hardware.health-service.samsung) reads
`%s/%s/{capacity,capacity_level,charge_counter,charge_full,charge_full_design,
current_avg,current_max,current_now,cycle_count,health,online,present,status,
technology,temp,time_to_full_now,type,voltage_max,voltage_now}`. Not provided:
capacity_level and time_to_full_now (4.9 did not provide them either), and
health_index/state_of_health. The 4.9-only supplies `ps`, `otg`,
`max77705-charger`, `max77705-fuelgauge` and `mfc-charger` are not registered
(the HAL does not read them; sepolicy genfscon labels for them are unused).
`battery/batt_slate_mode` (Lineage charging control) is deliberately absent:
writing it would turn charging off.

Thermal HAL (V:/etc/thermal_info_config.json) wants zones `battery`
(SHUTDOWN at 60 °C) and `ac` (EMERGENCY at 60 °C, monitored). Both exist
when `&adc` is enabled. 4.9 `ac`/TEMP is sec_battery `chg_temp`, the
**charger** NTC on SoC ADC ch3 (sec_battery.c:4792), not the USB-connector
NTC (ch1, `usb_temp`). `battery` also has the read-only 4.9 attributes
`batt_temp`, `batt_temp_adc`, `chg_temp`, `chg_temp_adc`, `usb_temp` and
`usb_temp_adc`.

## Capacity: same pipeline as 4.9

crownlte 4.9 DT `max77705-fuelgauge`: capacity_max 1000, margin 300,
capacity_min 0, capacity_calculation_type 28 = DYNAMIC_SCALE | ATOMIC |
SKIP_ABNORMAL, fg_resistor 2, capacity 3753.

1. SOCREP → 0.1 % (`max77705_fg_read_soc`, fuelgauge.c:392); 0 if VCELL < 3400 mV,
   AvgCurrent < 0 and SOC ≤ 1.0 % (`max77705_get_fuelgauge_soc`, :1141).
2. Scale `(soc − min) × 1000 / (capacity_max − min)`, including the CV-phase
   capacity_max walk-down (`max77705_fg_get_scaled_capacity`, :1384); clamp 0–1000.
3. Integer %, first value taken as is, then ±1 % per poll (ATOMIC) and no
   increase while not charging (SKIP_ABNORMAL) (`max77705_fg_get_atomic_capacity`, :1503).
4. Dynamic scale (`max77705_fg_calculate_dynamic_scale`, :1534) when the charger
   enters FULL and when it leaves FULL (sec_battery.c:2601 and :1156).
5. `battery/capacity` = 100 while status is FULL (sec_battery.c `sec_bat_get_property`).

With capacity_max 1000 and min 0 a fresh boot gives the same % as a fresh 4.9
boot. After a full charge, 4.9 lowers capacity_max (at most to 700); that
state is RAM-only on both kernels.

## I2C transactions (all on HSI2C 0x14360000)

Every access goes through `mx_read()`: one SMBus read = [S addr+W, register
pointer, Sr addr+R, data, P]. The register pointer byte is not a register
write. `i2c_smbus_write_*`, `i2c_master_send*`, `i2c_transfer*`, `regmap_*write*`,
`regmap_update_bits` and `I2C_SMBUS_WRITE` are `#pragma GCC poison`ed in the
driver (a test call fails with "attempt to use a poisoned identifier").

| addr | registers | when |
|---|---|---|
| 0x66 PMIC | PMICID1 0x00, PMICREV 0x01 (byte) | probe |
| 0x36 FG | DesignCap 0x18, TEMP 0x08, RepSOC 0x06 | probe |
| 0x36 FG | STATUS 0x00, VCELL 0x09, AvgVCELL 0x19, VFOCV 0xFB, Current 0x0A, AvgCurrent 0x0B, TEMP 0x08, RepSOC 0x06, VFSOC 0xFF, FullCapRep 0x35, FullCapNom 0x23, RepCap 0x05, Cycles 0x17, VBYP 0xB3, CONFIG 0x1D (word) | every poll (10 s with a source, 30 s without) |
| 0x69 CHG | CHG_INT_OK 0xB2, DTLS_00..02 0xB3..B5, CNFG_00 0xB7, CNFG_02 0xB9, CNFG_04 0xBB, CNFG_09 0xC0, CNFG_10 0xC1 (byte) | every poll |
| 0x25 USBC | BC_STATUS 0x08 (byte, through the adapter: 0x25 is bound to the MUIC driver) | every poll |
| 0x36 FG | VALRT 0x01, TALRT 0x02, SALRT 0x03, FullSOCThr 0x13, DesignCap 0x18, CONFIG 0x1D, IChgTerm 0x1E, LearnCfg 0x28, FilterCfg 0x29, MiscCfg 0x2B, VEmpty 0x3A, DischgTh 0x40, TALRTTH2 0xB2, CONFIG2 0xBB; QRTable20 0x32, QRTable30 0x42, RComp0 0x38, FullCapNom 0x23, FullCapRep 0x35, Cycles 0x17 | probe and self-test (snapshot compare) |
| 0x69 CHG | CNFG_00..CNFG_14 0xB7..0xC5 | probe and self-test (snapshot compare) |

Not read: CHG_INT 0xB0 and the USBC interrupt registers 0x02..0x05
(clear-on-read).

### Writes anywhere in the active paths

The only MAX77705 writer in this kernel is the existing diagnostic MUIC
driver (`drivers/soc/samsung/exynos9810-max77705-muic.c`, slave 0x25 = USB-C MCU):

* at probe, and again on every VBUS rising edge seen by its 2 s poll:
  opcode CONTROL1_WRITE (0x06) with 0x09 (D+/D− to the AP): one I2C block
  write of 2 bytes to AP_DATAOUT0 (0x21) plus one byte write 0x00 to
  AP_DATAOUT32 (0x41). This changes the USB switch only; it does not touch the
  charger, the fuel gauge or PD.
* reads with side effects: at probe and after the switch, `muic_log_status`
  reads UIC_INT/CC_INT/PD_INT/VDM_INT (0x02..0x05), which clears pending MCU
  interrupt flags. Its 2 s poll reads BC_STATUS 0x08 only.
* module parameter `path` (0644) can change the value written or disable it (-1).

The MUIC driver now counts its write transactions
(`exynos9810_max77705_muic_write_count()`); the battery self-test prints the
count (expect 2 after probe, +2 per VBUS reconnect).

Nothing else in the tree binds to 0x36/0x66/0x69 (`grep -rn max77705 drivers/`;
mainline `maxim,max77705-battery` in simple-mfd-i2c is not used by any DT).

## What 4.9 wrote and this stage leaves out

Fuel gauge (drivers/battery_v2/max77705_fuelgauge.c):
* `max77705_fg_init` :1182: CONFIG.TEX set (thermal_source ADC), CONFIG2 / DISCHARGE_THRESHOLD (auto discharge, not enabled on crownlte).
* `max77705_fg_set_vempty` :219: VEMPTY, VALRT threshold (probe and on every capacity read).
* `max77705_fg_write_temp` :268, called from set_property TEMP :2160, fed by sec_battery.c:2742 every poll: FG TEMPERATURE register = ADC battery temperature.
* `max77705_fg_reset_soc` :804 (quick-start: Cycles, MiscCfg, FullCap, RepCap; JIG only), `max77705_fg_reset_capacity_by_jig_connection` :885 (DesignCap).
* `max77705_fg_check_qrtable` :1563, `max77705_fg_check_status_reg` :902 (STATUS clear), fuel alert init, `max77705_set_full_value` (CONFIG_EN_OOPS).
* There is no battery-model load in this 4.9 tree; the model lives in the gauge.

Charger (drivers/battery_v2/max77705_charger.c): `max77705_charger_initialize`
(CNFG_00..12, PMIC MAINCTRL1, watchdog enable at :1078), watchdog kick :227
(called from health :290 and probe), float voltage, charge/input current, OTG,
and the shutdown hook :2800 (CNFG_00=0x04, CNFG_09=0x0F, CNFG_10=0x10, CNFG_12=0x60).

## Self-test (diag DTB, ~107 s)

```
max77705-ro-battery <bus>-0036: MAX77705 PMIC id .. rev ..; FG DesignCap 0ea9 (DT 0ea9, model present), CONFIG xxxx (TEX=1: temperature is host-written ...), ... read-only, no writes
max77705-ro-battery <bus>-0036: first sample: soc N% (repsoc ..) vcell ....mV inow ..mA temp ..C cable . status . chg_dtls .(...) cnfg00 ..
max77705-ro-battery <bus>-0036: battery selftest armed for 107000 ms after boot
...
battery selftest: soc=..% raw=..% repsoc=..% vfsoc=..% cap_max=1000 vcell=..mV vavg=..mV vocv=..mV inow=..mA iavg=..mA
battery selftest: temp=..C (FG TEMP, CONFIG=.... TEX=., changed|UNCHANGED since probe) present=1 health=1 cycles=.. fullcaprep=..mAh fullcapnom=..mAh repcap=..mAh design=3753mAh
battery selftest: charger int_ok=.. dtls=../../.. (chg=. <name> bat=. chgin=. wcin=. autoibus=.) cnfg00=..(mode=. wdten=.) cc=..mA cv=..mV ilim=..mA vbyp=..mV bc=..
battery selftest: online cable=. ac=. usb=. wireless=. status=.
battery selftest: i2c reads=N read_errors=0 writes=0 (no write path compiled) fg_cfg_changed=0 chg_cnfg_changed=0 learned_changed=. muic_opcode_writes=2 (at probe 2) sane=1 -> PASS
```

PASS = all reads OK, battery present, 3000 ≤ VCELL ≤ 4500 mV, 0 ≤ capacity ≤ 100,
−20 < temp < 80 °C, no FG config or charger CNFG register changed since
probe. "learned_changed" lists gauge-internal learning registers and is
informational. If CNFG_09/10 change, check `autoibus` (the USB-C MCU can set
the input limit itself).

## Risks and open points

1. **Charging may be off.** On 4.9 shutdown/reboot the charger hook writes
   CNFG_00=0x04 (buck only, charger off, watchdog off) and CNFG_09=0x0F
   (400 mA input). If SBoot does not rewrite these, a 6.12 boot entered via
   reboot does not charge, and this stage cannot turn charging on. Expect
   `cnfg00=04`, `chg=8 off` and status "Not charging" with a cable connected.
   That is safe for the battery; the battery drains slowly under load.
2. **Battery temperature (resolved by the SoC ADC stage, see soc-adc.md:
   `battery/temp` is now the ch4 NTC; the FG TEMP register is still never
   written and stays stale).** 4.9 used the SoC ADC (adc@14230000 channel 4,
   `battery,thermal_source = <2>`) and wrote it into the FG TEMP register with
   CONFIG.TEX=1. Under TEX=1 the gauge does not measure temperature, so the
   6.12 value is the last one 4.9 (or SBoot) wrote: plausible but stale. The
   self-test prints TEX and whether TEMP changed. The `battery` thermal zone
   and `battery/temp` therefore do not detect overheating in this stage.
   Health OVERHEAT/COLD thresholds (50/0 °C, 4.9 *_normal) apply to that value.
   Fix: port the Exynos ADC (exynos-adc-v3, 4.9 drivers/iio/adc) and read ch4
   (battery) and ch1/ch3 (USB/charger, for the `ac` zone).
3. Units follow 4.9 (current in mA). If the health HAL expects µA, Android
   shows currents 1000× too small; same as on 4.9.
4. Health "Under voltage" (4.9) does not exist in 6.12; it is reported as
   Unknown, which is what healthd made of it on 4.9.
5. Charger watchdog: if something left WDTEN=1, nobody kicks it; the charger
   then stops charging (`chg=b wdt-expired`). Safe, reported by the self-test.
6. Lineage `batt_slate_mode`, `/sys/class/sec/switch/afc_disable`, CISD and
   the 238 `battery/*` rc nodes are absent; init only logs chown failures.
7. No IRQ: cable changes are seen within one poll (10–30 s).

## Next stages

* Charger watchdog kick (CNFG_06 WDTCLR only, supervised, DT `samsung,charger-wdt-kick`,
  off by default): implemented, see `power-gates.md`. With it enabled the driver is no
  longer write-free; the self-test then counts the kicks as its only writes.

* ADC thermistors (battery, USB, charger) → real `battery`/`ac` temperatures: done (soc-adc.md).
* Charger RW only after register-by-register parity with 4.9 (ROADMAP risk 6),
  starting with re-enabling charging after a 4.9 reboot.
