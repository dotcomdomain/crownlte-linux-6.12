# crownlte power gates: touch rails and charger watchdog kick

Status: build-only, not yet run on the phone. Both features are off unless the
DT opts in. Owner decisions: `roadmap/decisions-20261105/2-touch-power.md` and
`3-charger.md` on gekko (both approved 2026-11-05).

## 1. Touch rails (S2MPS18 LDO35/LDO43 over ACPM)

SBoot leaves the touch IC unpowered: L35CTRL 0x6C = 0x2C, L43CTRL 0x74 = 0x30
(vsel 1.8 V / 3.0 V, enable bits 7:6 clear). 4.9 turns them on with
`exynos_acpm_update_reg(type 1, reg, val, mask 0xC0)`, a firmware-side SPEEDY
read-modify-write (`s2m_enable()`, `s2mps18_core.c:116-129`). The APM firmware
handles every FUNC_UPDATE other than 0x32A/0xC0 as `(rd & ~mask) | (val & mask)`
(`g3d-power-domain.md`, APM 0x5064), so mask 0xC0 can only change bits 7:6.

DT opt-in, both needed:

| node | property | effect |
|---|---|---|
| `s2mps18-pmic` (`samsung,exynos9810-s2mps18-acpm`) | `samsung,touch-rail-update` | sealed policy bit `touch_rails`, logged by `write policy sealed: ... touch_rails=1` |
| `touchscreen@48` (`samsung,sec-ts-y771`) | `samsung,acpm-touch-power` | probe powers the IC; without it stage 1 is unchanged |

### What the ACPM transport allows (`acpm_mfd.c`, `acpm_mfd.h`)

Only with `touch_rails` sealed, only FUNC_UPDATE, type 1, mask 0xC0:

| request | precondition (PMIC read in the same locked section) |
|---|---|
| 0x6C val 0x80 (LDO35 on, mode 0b10 as 4.9) | 0x6C vsel 0x2C, mode 00 or 10; at most 2 per boot |
| 0x74 val 0xC0 (LDO43 on, mode 0b11 as 4.9) | 0x6C reads exactly 0xAC; 0x74 vsel 0x30, mode 00 or 11; at most 2 per boot |
| 0x74 val 0x00 (LDO43 off) | none |
| 0x6C val 0x00 (LDO35 off) | 0x74 mode 00 (LDO43 already off) |

Everything else stays refused: FUNC_WRITE and FUNC_BULK_WRITE to the PMIC or TOP,
any other register, value or mask. The G3D rail exception is unchanged.

### Sequence (`sec_ts_y771.c`)

1. Wait (`-EPROBE_DEFER`) until the S2MPS18 driver has sealed the policy. If the
   policy refuses touch rails, log it and continue unpowered (stage-1 "absent").
2. Read 0x6C and 0x74. Refuse unless vsel is 0x2C / 0x30 and modes are 00 or the
   4.9 mode; refuse LDO43-on-while-LDO35-off. Both already 0xAC/0xF0: no write.
3. LDO35 on, 1-2 ms, read back 0xAC.
4. Re-read 0x6C == 0xAC, LDO43 on, read back 0xF0.
5. 70 ms, then 4.9 `sec_ts_wait_for_ready()`: while the IRQ line is low, read
   events (READ_ONE_EVENT) until the boot-complete status event, 100 x 20 ms.
6. The existing stage-1 identify/start. If the IC does not ACK right after our
   power-on, one power cycle (off, 30 ms as 4.9 `sec_ts_reset_work`, on).
7. Off (LDO43 0x00, 4 ms, LDO35 0x00 only if 0x74 reads off) on any probe failure
   and in `.shutdown` (4.9 shutdown -> remove -> `sec_ts_power(off)`).
   Unbind/rebind via sysfs is disabled (`suppress_bind_attrs`). The self-test
   cleanup is registered before the rails go on, so no early probe return can
   leave them on.
8. At most 2 power-on sequences per boot (driver counter and transport counter).

The power path, including the policy query, is compiled only for a built-in
driver with `CONFIG_EXYNOS_ACPM` (the ACPM accessors are not exported). Otherwise
a stub logs an error and the IC stays unpowered; that branch has no ACPM
references, so a module build links.

### Expected log

```
ACPM-MFD: write policy sealed: ... g3d_rail=1 touch_rails=1 ...
sec_ts <bus>-0048: power: before 0x6c=0x2c 0x74=0x30, power-ons so far 0
ACPM-MFD: touch rail update reg=0x6c val=0x80 mask=0xc0 from ...: 0
sec_ts <bus>-0048: power: LDO35 tsp_io on: 0, 0x6c=0xac
ACPM-MFD: touch rail update reg=0x74 val=0xc0 mask=0xc0 from ...: 0
sec_ts <bus>-0048: power: LDO43 tsp_avdd on: 0, 0x74=0xf0
sec_ts <bus>-0048: boot complete after N polls: ...
sec_ts <bus>-0048: device id ...
touch selftest: power control on, power-on result 0, rails on, now 0x6c=0xac 0x74=0xf0 (read 0)
```

## 2. MAX77705 charger watchdog kick

SBoot leaves fast charge on with CNFG_00 = 0x15 (WDTEN=1). Without a kick the
charger stops at ~80 s (CHG_DTLS 0xB, seen in q34/q48). The watchdog is never
disabled; this only kicks it, like 4.9 `max77705_chg_set_wdtmr_kick()`
(`max77705_charger.c:227-235`, from every health poll, `:300`).

DT opt-in: `samsung,charger-wdt-kick` on `battery-monitor@36`
(`samsung,exynos9810-max77705-battery-ro`).

### The only write

`mx_chg_wdt_kick_write()` in `max77705-ro-battery.c`, defined before the
`#pragma GCC poison` block, with fixed slave 0x69, register CNFG_06 0xBD and value
`(cnfg06 & ~0x03) | 0x01`. Before it, CNFG_06 is read; CHGPROT (bits 3:2) must read
11 = unlocked, as 4.9 init leaves it (`max77705_charger.c:1008-1012`; no unlock
write is ever sent), the sample must still be younger than 20 s, and bits 7:2 must equal the
probe value; after it, read back and bits 7:2 must still match. A mismatch stops
kicking for the rest of the boot. Every other write API stays poisoned.

### Supervisor (every 10 s, on the sample taken in the same poll)

A kick is sent only if all of these hold; otherwise no kick, and the first stop
(and every later start/stop transition, max 16 messages) is logged:

| check | limit |
|---|---|
| FG + charger reads, both SoC ADC NTCs | all read in this poll, no ADC "previous value" fallback; the sample's first read < 20 s ago, checked again right before the write |
| battery NTC (ch4), charger NTC (ch3) | ADC code inside its 4.9 table (not clamped) |
| battery present | FG STATUS.BST = 0 and charger INT_OK.BATP_OK or !DTLS_00.BATP_DTLS (4.9 `max77705_check_battery()`) |
| battery temperature | 10.0 <= T < 41.0 C. Hot: stop at 41.0 C, resume below 39.0 C (4.9 `swelling_high_temp_block/recov`; above 41 C 4.9 drops CV to 4.15 V). Cold: stop below 10.0 C, resume at 10.0 C. 4.9 blocks charging at <= 5 C and up to 10 C (`swelling_low_temp_recov_2nd`) only allows a restricted recharge below 4000 mV with CV 4150 mV (`sec_battery.c:1556-1563, 1596-1634`). A kick cannot lower CV, so there is no restricted band |
| charger NTC | T < 44.0 C (resume below 42.0 C) |
| VCELL and AvgVCELL | 3000..4400 mV (CV 4350 + 50) |
| health | GOOD only: input under- or overvoltage, dead, overheat or cold stop the kick (4.9 `sec_bat_ovp_uvlo_result()` turns charging off on UV/OV) |
| charger state | CHG_DTLS a known code; no charging state (0x0-0x3) without CHGIN/WCIN |
| input (raw DETAILS_00, every sample, independent of the cable type) | CHGIN_DTLS and WCIN_DTLS not 01 (UV) or 10 (OV); CHGIN_OK with CHGIN_DTLS 11, or WCIN_OK with WCIN_DTLS 11 (4.9 `max77705_get_vbus_state()`, `max77705_get_charging_health()`). No valid input = no kick |
| CNFG_00 | WDTEN set, value unchanged since probe |
| CNFG_02 / CNFG_04 | unchanged since probe; CC <= 1500 mA (<= 950 mA below 20.0 C: 4.9 1st cold-swelling tier, block 15 C / recover 20 C), CV <= 4350 mV |
| CNFG_06 | CHGPROT = 11; bits 7:2 unchanged since probe (a mismatch or locked CHGPROT stops kicking for the boot) |

Shutdown (`.shutdown`) and unbind stop the kicks and all three works
synchronously; no charger register is written then.

The first poll (and kick) runs at probe. If the watchdog had already expired
(CHG_DTLS 0xB) the driver says so once: a kick does not restart charging, and the
4.9 CNFG_00 restart toggle (decision 3, C2) is not implemented.

### Expected log

```
max77705-ro-battery ...-0036: charger wdt kick enabled: CNFG_06 xx at probe, every 10000 ms while batt 10.0..41.0 C, chg < 44.0 C, 3000..4400 mV, CV <= 4350 mV, CC <= 1500 mA
max77705-ro-battery ...-0036: charger wdt kick: started (batt 30.1C chg 31.0C vcell 3950mV cv 4350mV cc 450mA chg_dtls 1 cc)
battery selftest: charger wdt kick on: kicking, kicks=11 stops=0 last kick 3000 ms ago
battery selftest: ... writes=11 (CNFG_06 WDTCLR kicks only) ... chg_cnfg_changed=0 ... -> PASS
```

The self-test compares CNFG_06 with bits 1:0 masked while kicking (WDTCLR is a strobe).

## Test DTB

`panel-bright-work/dtb/exynos9810-crownlte-romram-power.dtb` = ROM-RAM v18 plus
`samsung,touch-rail-update` on `s2mps18-pmic`, `usi@104b00c0`, `i2c@104b0000` and
`touchscreen@48` enabled with `samsung,acpm-touch-power` and
`samsung,touch-selftest`, and `samsung,charger-wdt-kick` on `battery-monitor@36`.

## Open points

* The charger watchdog period (~80 s) is inferred from logs, not from a datasheet.
* Option A only keeps the charging state; on a 500 mA SDP port the battery still
  drains (decision 3, "net charge").
* With no valid input there is no kick. Whether the charger watchdog keeps
  running while unplugged is unknown; if it expires then, charging stays off
  after a replug (the CNFG_00 restart is not implemented).
* If SBoot leaves CHGPROT locked, there is no kick at all (and no charging past
  the watchdog); the first trial log shows CNFG_06 at probe.
* LDO35 mode 0b10 is "on while PWREN_MIF" (4.9 parity); harmless while 6.12 has
  no suspend.
