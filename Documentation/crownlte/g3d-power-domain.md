# crownlte G3D power domain on/off (Mali stage 2, power part)

Branch `crownlte-next`. Build-only: nothing here has run on the phone yet.

## 1. What 4.9 does

`cal_pd_control(0xb1380008, on)` is in `K49/drivers/soc/samsung/cal-if/cal-if.c:168`.
It runs for genpd `pd-embedded_g3d` from `exynos_pd_power_on/off` (`K49/drivers/soc/samsung/exynos-pd.c:107,145`).
The 4.9 DT node is `pd-embedded_g3d@14064040`, `cal_id = <0xb1380008>`, `need_smc = <0x17410204>` (`exynos9810-crownlte_eur_open_26.dts:3755-3762`).

| Step | On | Off |
|---|---|---|
| a | `exynos_acpm_update_reg(0x3, 0x2a, 0xc0, 0xc0)` (cal-if.c:180) | `pmucal_local_disable`: save seq, then DTZPC save SMC, then `embedded_g3d_off` (`pmucal_local.c:72`) |
| b | `pmucal_local_enable` (`pmucal_local.c:16`): `embedded_g3d_on`, then DTZPC restore SMC, then restore seq | `exynos_acpm_update_reg(0x3, 0x2a, 0x00, 0xc0)` (cal-if.c:185) |

4.9 ignores the return value of the ACPM update.

`K` = `/root/crownlte-hid-kernel`, `T` = this tree.

### 1a. ACPM "type 3, reg 0x2a, 0xc0"

**Kernel encoding.** `exynos_acpm_update_reg(type, reg, value, mask)` (`K49/drivers/soc/samsung/acpm/acpm_mfd.c:196`) sends on ACPM channel 2:

- `cmd[0] = reg | (type & 0xf) << 8`
- `cmd[1] = FUNC_UPDATE(2) | value << 8 | mask << 16`

The shifts are in `K49/include/soc/samsung/acpm_mfd.h:6-26`.
For S2MPS18 accesses, "type" is the SPEEDY slave: TOP 0, PMIC 1, RTC 2, DEBUG 0xf (`K49/drivers/mfd/s2mps18_core.c:40-43`).
The request is therefore SPEEDY address 0x32a.

**What else could be on the bus.** There is only one SPEEDY bus with one device: `speedy@141C0000/s2mps18mfd@00`, `acpm-ipc-channel = <2>` (dts:8723-8725).
S2MPB02, S2DOS05 and the other PMICs sit on normal I2C and are not reachable through ACPM.
No 4.9 driver uses slave 3. `0x2a` is `S2MPS18_PMIC_REG_B6CTRL` (BUCK6, `vdd_g3d`) in the PMIC slave (`s2mps18-private.h:74`). In the type-3 slave the address means nothing.

**What the firmware does with it.** The APM firmware handles the request itself. It never forwards it to SPEEDY.

The APM firmware is in the sboot LUN `sdb` (sha256 `09d0dd96…`) at offset 0x2fc00, loaded at APM address 0.
The base was found from literal pools: 62 of 66 MFD log tags and 106 of 108 PMU log tags resolve with that delta.
In APM space the AP PMU appears at 0xa4060000 (for example 0xa4060404 = RST_STAT) and SPEEDY at 0xa41c0000.
Evidence is on gekko in `/mnt/ssd/host-builds/kernel-upgrade/next-research/` (`apm.bin`, `apm-mfd-g3d-0x4e20-0x5230.dis`, `SHA256SUMS`, `README.txt`).

| APM addr | Function |
|---|---|
| 0x512c | MFD IPC dispatcher. Jump table at 0x597c, indexed by FUNC: 0 read 0x4eac, 1 write 0x4f98, 2 **update 0x5064**, 3 bulk read 0x4eec, 4 bulk write 0x4fd0 |
| 0x5064 | FUNC_UPDATE. `addr = cmd[0] & 0xfff`, `val = cmd[1] >> 8 & 0xff`, `mask = cmd[1] >> 16 & 0xff`. **`if (addr == 0x32a && mask == 0xc0)` → `g3d_rail(val)`** (`movw r3,#0x32a; cmp r5,r3` at 0x5086-0x508e, `cmp r2,#0xc0` at 0x50c6). Otherwise it does a generic SPEEDY RMW: `(rd & ~mask) \| (val & mask)` |
| 0x4e4c | `g3d_rail(v)`. `v == 0xc0`: write 1 to **PMU 0x14064678**, poll **0x1406467c bit 0** until set (at most 1999 polls), then `b6ctrl_bit3(1)`. `v == 0`: first `b6ctrl_bit3(0)`, then PMU 0x14064678 = 0, poll bit 0 clear. Any other value: no action, returns 0x7d0 |
| 0x4e20 | `b6ctrl_bit3(on)`. SPEEDY read of 0x12a (PMIC slave 1, **B6CTRL**), `(x & ~0x0c) \| on << 3`, write back. So bits 3:2 become 0b10 or 0b00 |

A poll timeout only logs the tag `g3d_cnt` (APM 0x58b8). The reply still carries RETURN = 0 (0x50d0-0x50e6 branch to the success store at 0x50ba). **The kernel cannot tell from the reply whether the rail came up.**

**Semantics of 0xc0.** It is not a register value. `mask = 0xc0` selects the special path, and `value` is an op code (0xc0 = on, 0x00 = off).

**PMU 0x4678/0x467c.** These belong to the PWR_EMBEDDED_G3D block:

- `PWR_EMBEDDED_G3D_OPTION` 0x4680 and `_DURATION` 0x4684 (= 0x363) are programmed by `pmucal_lpm_init` (`flexpmu_cal_system_exynos9810.h:97-98`). 6.12 runs the same table in `pmucal_system_init`.
- The names `CONFIGURATION` for 0x4678 and `STATUS` for 0x467c come from the PMU block layout. They are inferred, not taken from a header.

**Is vdd_g3d (S2MPS18 BUCK6) affected? Yes, indirectly.**

- The 4.9 DT has `g3d_en = <1>` (dts:8740).
- The 4.9 regulator probe writes `B6CTRL[7:6] = 0b10` ("for buck6 gpio control, disable i2c control", `K49/drivers/regulator/s2mps18.c:1199-1209`). Mode 2 means on while the PWREN pin is high (`s2mps18_of_map_mode`, :99-111).
- With `g3d_en`, enable/disable/is_enabled of BUCK6 are no-ops (:158, :173, :192).

So in 4.9 the G3D rail is switched by the PMU PWR_EMBEDDED_G3D sequencer through the BUCK6 enable pin. The firmware also toggles B6CTRL bits 3:2 around it.

The datasheet is not available, so the meaning of bits 3:2 is unknown. They are set only after the rail is up and cleared before it goes down, which is consistent with a mode bit such as forced PWM (GUESS).

6.12 has no S2MPS18 regulator driver, so `B6CTRL[7:6]` stays as SBoot left it:

- `0b10`: the rail follows the PMU sequencer, as on 4.9.
- `0b11`: the rail stays on, which wastes power but is safe.

The probe log prints B6CTRL, so the first trial answers which case applies.

### 1b. TrustZone SMC

`exynos_pd_tz_restore/save(need_smc)` (`K49/drivers/soc/samsung/exynos-el3_mon.c:327-337`) calls:

- `exynos_smc(SMC_CMD_PREAPRE_PD_ONOFF = 0x82000410, 1 = EXYNOS_WAKEUP_PD_DOWN (restore), 0x17410204, 0)` on power-on, after `embedded_g3d_on`.
- The same call with x1 = 0 (EXYNOS_GET_IN_PD_DOWN, save) on power-off, before `embedded_g3d_off`.

0x82000410 is an SMC32 SiP fast call.
0x17410204 is offset 0x204 in BLK_G3D's SYSREG page (0x17410000; `BUS_COMPONENT_DRCG_EN` is at 0x17410104). Every 4.9 `need_smc` value follows the same `<block>+0x10204` pattern.

The EL3 monitor saves and restores the block's DTZPC (TrustZone protection) settings across the power cycle. Without the restore, the GPU registers would come back secure-only, and non-secure access would fault.

**6.12.** `T/drivers/soc/samsung/exynos9810-cal-smc.c:32-38` issues the identical SMC through `arm_smccc_smc(0x82000410, 0|1, addr, 0)`. The monitor is the unchanged SBoot one, so the interface is the same.

The 0x82000410 constant is not visible in `sdb`, so the monitor binary was not disassembled. The evidence that it works is that the running 4.9 kernel issues this SMC on every runtime-PM cycle of the GPU.

**Gap in 6.12, now handled.** 4.9 sets `need_smc` from DT in `exynos-pd.c:334-340`. 6.12 has no caller of `cal_pd_set_smc_id()`, so without the Mali code below, `pmucal_local_enable` would skip the SMC.

### 1c. PMU sequence

`embedded_g3d_on` (`flexpmu_cal_local_exynos9810.h:475-481`):

1. `PLL_CON2_PLL_G3D = 0x30000003`
2. Set `PLL_CON0_PLL_G3D` bit 4
3. Set `MUX_CLKCMU_EMBEDDED_G3D_USER` bit 4
4. `EMBEDDED_G3D_CONFIGURATION (0x14064040) = 0xf`
5. Wait for `STATUS (0x4044) == 0xf`

`embedded_g3d_off` runs the reverse.

CMU_G3D (0x17400000) is written before CONFIGURATION, so it lies outside the switched domain and keeps PLL_G3D's PMS. The code still reads the rate back after power-on.

## 2. Implementation (DT-gated, default off)

| Piece | File | Gate |
|---|---|---|
| Transport: exactly `FUNC_UPDATE type 3, reg 0x2a, mask 0xc0, value 0xc0 / 0x00`. Every other update stays `-EPERM`. Success means "handled", not "rail up" | `drivers/soc/samsung/acpm/acpm_mfd.c`, `include/soc/samsung/acpm_mfd.h` | sealed policy field `g3d_rail` |
| Policy owner | `drivers/mfd/s2mps18-acpm.c` | S2MPS18 node `samsung,g3d-rail-update` |
| Domain control | `drivers/gpu/arm/exynos9810/mali_exynos9810_pd.c` (+ `_platform.c`, `_selftest.c`) | gpu node `samsung,g3d-power-control = "boot-on" \| "runtime"` |
| 4.9 constants (inert data) | `exynos9810.dtsi` gpu: `samsung,g3d-pd-cal-id = <0xb1380008>`, `samsung,g3d-need-smc = <0x17410204>`. Other values are refused | — |
| Trial DTBs | `exynos9810-crownlte-delayed-ufs-g3dpd-diag.dts` ("boot-on"), `…-g3dpd-runtime-diag.dts` ("runtime", run only after boot-on passes) | — |

`exynos9810-crownlte-delayed-ufs-diag.dtb` does not set either gate, so its behaviour is unchanged from `crownlte-mali`. It only gains the two inert properties.

### Behaviour

**Probe.**

1. Log PMU G3D, PWR_EMBEDDED_G3D, CMU mux and B6CTRL.
2. If a mode is set: check `cal_pd_status` against the PMU, then call `cal_pd_set_smc_id(0xb1380008, 0x17410204)`.
3. If SBoot left the domain off, call `cal_pd_control(on)` and verify.

**Every transition is verified.**

- On: G3D STATUS = 0xf, PWR status bit 0 = 1, CMU user mux = 1, and the CAL rate readback is a table level.
- Off: G3D STATUS = 0 and PWR bit 0 = 0.

A failed transition latches `pd_failed`. After that no power-off is attempted.

**"boot-on".** The domain is never switched off, apart from the one self-test cycle.

**"runtime".** This follows 4.9 runtime PM:

- kbase power_off schedules a domain off after `samsung,g3d-off-delay-ms`. The default is 150 ms, longer than the 100 ms DVFS idle drop. 4.9 used 50 ms.
- kbase power_on cancels the pending off, switches the domain on under `pd_lock`, and returns 1, so kbase resets the GPU.
- Lock order is `pd_lock` then `set_lock`. DVFS never runs during a transition.

**Unchanged.** System suspend is still vetoed while the GPU is bound. IFPO and the 4.9 BTS/BCM/idle-ip hooks are not ported. (4.9's `exynos_pd_power_off_pre` CL-DVFS notification only applies to "pd-g3d", not "pd-embedded_g3d".)

### Expected log (g3dpd trial)

```
ACPM-MFD: write policy sealed: rtc_reg_mask=0x00000000 update_reg=0x02 forbidden=0x00 g3d_rail=1 (...)
s2mps18-acpm ...: S2MPS18 chip id ... G3D rail requests allowed; TOP/PMIC writes refused ...
mali exynos9810: G3D at probe: PMU cfg 0xf status 0xf, PWR cfg 0x1 status 0x1, CMU mux 0x..1., B6CTRL 0x8? (read 0)
mali exynos9810: G3D power control "boot-on": cal_id 0xb1380008, DTZPC SMC target 0x17410204, off delay 0 ms
[only if SBoot left it off:] mali exynos9810: G3D domain on (probe): N us, PMU cfg 0xf status 0xf, PWR ... status 0x1 ...
mali exynos9810: G3D domain on (...), ..., 6 DVFS levels, power control boot-on
... ~109 s ...
mali selftest: power down: kbase GPU off after N ms, ... (G3D power control boot-on)
mali exynos9810: G3D domain off (selftest): N us, PMU cfg 0x0 status 0x0, PWR cfg 0x0 status 0x0, ..., B6CTRL 0x.(bits 3:2 = 00)
mali selftest: pd cycle: off 0, N us, PMU G3D cfg 0x0 status 0x0
mali exynos9810: G3D domain on (selftest): N us, PMU cfg 0xf status 0xf, PWR cfg 0x1 status 0x1, ..., B6CTRL 0x.(bits 3:2 = 10)
mali selftest: pd cycle: on 1, N us, ..., CAL pd status 1, pd on/off/errors 1/1/0
mali selftest: pd cycle: kbase power up 0, N us, GPU_ID 0x6221...., L2 present 0x1 ready 0x1, clock ... kHz
mali selftest: pd cycle: cache clean+invalidate 0, N us
mali selftest: pd cycle: PASS
mali selftest: PASS (0 failures, N ms)
```

These log lines mean failure:

- `G3D domain on|off (...): FAIL ret …`
- `G3D domain did not power on (…); GPU register access will fault`
- `ACPM-MFD: refused update type=0x3 …` (the PMIC node is missing `samsung,g3d-rail-update`)

### Risks

- **Rail timeout is hidden.** If PWR bit 0 never sets, the firmware still replies OK. The PMU checks catch this, but the domain is then half on. kbase can only be stopped at probe; in "runtime" mode a later failure means the GPU register access faults (an SError is likely). That is why "boot-on" with a single supervised cycle comes first.
- **DTZPC.** If the SMC were rejected, `pmucal_local_enable` returns an error and we latch `pd_failed`. If it silently did nothing, the first GPU register read after the cycle would fault. That read is in the self-test, at a known time.
- **B6CTRL[7:6] = 0b11 from SBoot.** "Off" does not remove vdd_g3d. This is safe but saves less power. The B6CTRL value printed at probe shows which case applies.
- **Trial order.** Run the q27 Mali trial first; it shows whether SBoot leaves the domain on. Then run g3dpd (boot-on), then g3dpd-runtime. Hang risk is moderate: have Odin ready as for the other trials.
