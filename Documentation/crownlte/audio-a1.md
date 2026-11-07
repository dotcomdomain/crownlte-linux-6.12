# crownlte 6.12: audio stage A1, read-only hardware probe

Design: gekko `roadmap/audio-design.md`, section 5 "A1" (also sections 1, 4.1,
4.3, 7). Branch `crownlte-audio-a1` on `crownlte-suspend-s0` 60774ac87.
Build only; not run on the phone yet.

A1 answers the design's open questions 1-3 (is pd-aud on at boot, amp
silicon revision, is LDO34 on) and checks every line the later stages
depend on. It reads only. Nothing writes to the codec, the amps, the PMIC,
ABOX, SYSREG_AUD, the SysMMU or any power domain.

## 1. Why this base

`crownlte-suspend-s0` 60774ac87 rather than `crownlte-integration-6`
4148bf525:

- it has the CL0 watchdog deadline API (`include/soc/samsung/crownlte-wdt.h`,
  `s3c2410wdt_crownlte_set_deadline()` in the ping path, commits 766bcf035
  and 7b58093af). A1 uses it to cover a stalled bus read. int6 has no
  deadline API;
- its delayed-UFS vehicle passed on hardware with this exact config
  (`config-suspend-s0-ee215e307`, pack 416fb7f3, S0a-S0d PASS);
- int6 adds perf/DPU SysMMU/Wi-Fi/TEE, none of which A1 needs. A1 is three
  small commits that merge into int6 without conflicts (one new driver, a
  disabled bus node, one new DTS).

## 2. What A1 reads

Self-test `audio selftest A1:` from DT `samsung,audio-a1-selftest-ms`
(30000), run on CPU1 (the first online CPU after CPU0), so that a CPU wedged
on a read leaves the CPU0-pinned PID1 restart timer alone. Order:

| # | Block | Registers | Precedent / gate |
|---|---|---|---|
| 1 | CL0 watchdog | deadline 145000 ms installed, then state checked: enabled, reset on, still petted, deadline <= ours, WTCNT moving | same check as `exynos9810-pm.c` S0 |
| 2 | PMU 0x14060000 (always on, syscon) | AUD_CONFIGURATION 0x4000, **AUD_STATUS 0x4004**, ABOX_CPU_CONFIGURATION 0x415c, ABOX_CPU_STATUS 0x4160, ABOX_CPU_OPTION 0x4164, ABOX_CPU_STANDBY 0x3804, PAD_RETENTION_ABOX_OPTION 0x4170, GPIO_MODE_ABOX_SYS_PWR_REG 0x1424 | 4.9 pmucal `aud_status`, `abox.c` reads/updates of each |
| 3 | GPIO (pinctrl alive, peric0, peric1, cmgp) | CON/DAT/PUD of codec IRQ gpa2-2, codec reset gpg1-0, codec SPI gpp0-4..7, amp IRQs gpm2-0/gpm3-0, amp I2C gpp4-4/5 | 6.12 gpiolib reads every bank CON at boot; 4.9 |
| 4 | S2MPS18 over ACPM ch2 | PMIC L33CTRL 0x6a, L34CTRL 0x6b (`s2mps18_acpm_read`) | read-only ACPM path; the transport's write policy is sealed empty in this DTB |
| 5 | MAX98512 0x39, 0x3a on HSI2C18 (0x108c0000) | 0x0402 rev ID; 0x0600 only if 0x0402 < 0x40 (rev B); GLOBAL_SHDN 0x0400 (rev A) or 0x0500 (rev B), bit 0 = GLOBAL_EN; INT_RAW1-3 0x0001-0x0003 | 4.9 `max98512_i2c_probe` order; each read is a 2-byte register-address message + repeated start + 1-byte read; no data byte is sent. Anything off this whitelist is refused in code |
| 6 | Codec CS47L93 | no SPI transfer. Reads PERIC0 SYSREG USI01 SW_CONF 0x1041100c and evaluates the SPI ID-read preconditions | see 2.1 |
| 7 | **pd-aud**, only if AUD_STATUS[3:0] == 0xf **and** the deadline is armed | pinctrl_aud gpb0..gpb2 CON/DAT/PUD (0x17c60000); SYSREG_AUD MISC_CON 0x17c1032c; ABOX SFR IP_INDEX 0x0, VERSION 0x4, SYSPOWER_CTRL 0x10, SYSPOWER_STATUS 0x14 (0x17c50000); ABOX SysMMU CTRL 0x0, VERSION 0x34 (0x17e00000); then AUD_STATUS again | 4.9 abox regmap readable list, 4.9 sysmmu version read, 6.12 gpiolib on pinctrl_aud; each block is announced on its own line before it is read |

The jack has no AP GPIO: detection is the codec's own JD2/MICDET
(`cirrus,accdet`). The codec IRQ line gpa2-2 is the only AP-visible signal;
its level is logged. Jack state itself needs the codec (A3).

### 2.1 Why no codec SPI read

The silicon ID (0x6371 at register 0) needs USI01 in SPI mode, the
gpp0-4..7 pins muxed to SPI, the codec reset gpg1-0 released and DCVDD
(LDO34) on. The 6.12 tree has no SPI-mode USI or 9810 SPI controller
support, and bringing any of these up is a write (SYSREG, pin mux, GPIO,
PMIC = gate G1). A plain-protocol mistake would also be costly: a madera
SPI frame without the read bit on register 0 is a software-reset write.
So A1 only reports which preconditions hold (`codec_spi=skip(<first
missing>)`); the ID read moves to A3 with mainline madera.

### 2.2 Writes that do happen in an A1 boot (not by this driver)

- `exynos-usi` for the new `usi_amp` node: PERIC1 SYSREG USI07 SW_CONF
  (0x10811024) := I2C, USI07 wrapper USI_CON/USI_OPTION, the USI07 clocks
  (`GATE_USI07`, `USI07`), exactly as for the touch bus (USI03).
- `i2c-exynos5` for `hsi2c_amp`: HSI2C18 controller reset/timing registers.
- pinctrl "default" of the bus: gpp4-4/gpp4-5 to function 2, no pull
  (the 4.9 `hsi2c18-bus` values).
- Everything the delayed-UFS diagnostic always does.

No amp child node exists, so no amp driver binds and the i2c core scans
nothing (no adapter class, no I2C_CHARDEV).

## 3. Code

- `drivers/soc/samsung/exynos9810-audio-a1.c` (CONFIG_EXYNOS9810_AUDIO_A1,
  bool): platform driver `samsung,exynos9810-audio-a1`. Idle without
  `samsung,audio-a1-selftest-ms` (allowed 20000..100000);
  `samsung,audio-a1-deadline-ms` is mandatory (121000..180000). The DT
  windows are mapped with `devm_ioremap()` without claiming them.
- `exynos9810-crownlte-clocks.dtsi`: `usi_peric1_07_sysreg`, `usi_amp`
  (`usi@108c00c0`) and `hsi2c_amp` (`i2c@108c0000`, SPI 430, 400 kHz,
  `hsi2c18_bus` pins), all **disabled** and without children.
- `exynos9810-crownlte-delayed-ufs-audio-a1-diag.dts`: delayed-UFS
  diagnostic + bus enabled + `audio-a1@14050000` node; the other
  self-tests removed as in the suspend S0 DTB (cpuidle, MCT report,
  devfreq, TMU, cpufreq, display, battery, ADC; Mali disabled). The
  read-only PMIC snapshot at ~106 s stays.

Config: `config-suspend-s0-ee215e307` (1ec1bef6) +
`CONFIG_EXYNOS9810_AUDIO_A1=y`; no other difference after olddefconfig.

## 4. Artifacts

| Item | Value |
|---|---|
| Commits | d53dfd5ac driver, 10161fa8f DTS, this doc |
| Image | `panel-bright-work/images/Image-audio-a1-d53dfd5ac` sha256 `490e3ab87a4b1473a67b3dfc3548487eb34713b543cb9f5ba4c8cbe5d95c72f0` |
| DTB | `panel-bright-work/dtb/exynos9810-crownlte-delayed-ufs-audio-a1-diag-10161fa8f.dtb` sha256 `35c9c7aa3cce4ad2c91aa43dfad881b526f0848823400a0061ed47409f69c0d1` |
| Pack | gekko `acpm-audio-a1-s2/recovery.img` sha256 `c760102783fdfb99fcc769b384aeca0d73a4934ee552e85fbdfc5f4020f88eb2` (68149248 bytes); loader `b0d88e6b...`; shim bbea959; ramdisk 506232b1; parent a8198be0; see `PROVENANCE.txt` |

## 5. Running it

Same procedure as the other delayed-UFS trials: copy `acpm-audio-a1-s2`
to the PC (it includes `ready-capture.ps1`), start only with
`adb reboot recovery`, then `run-trial.sh <dir>` (the default 180 s
capture is enough). The host sends `CROWNLTE-UFS-READY-1` once. **Never
send `CROWNLTE-UFS-PROBE-1`.** No physical help needed (no audio output,
no headset).

| Boot time | What |
|---|---|
| boot | `exynos-usi 108c00c0.usi: USIv2 configured: sw_conf=00000004 ...`; `exynos5-hsi2c 108c0000.i2c: 9810 timing: clock=... bus=400000 ...`; `exynos9810-audio-a1 14050000.audio-a1: audio A1 read-only probe scheduled at ~30000 ms on CPU1, watchdog deadline 145000 ms` |
| 30 s | `s3c2410-wdt ...: crownlte deadline: no reload after 145000 ms boot time`; the A1 lines (section 6) |
| ~106 s | `pmic selftest:` (read-only) |
| 120 s | PID1 restart (150 s fallback); shim -> Lineage |
| 145 s | only if still up: watchdog refuses reloads, reset within 30 s |

After return, `run-trial.sh` saves `/sys/fs/pstore/pmsg-ramoops-0` and
restores bright RECOVERY `8763860c`.

## 6. Expected lines

```
audio selftest A1: start uptime=30xxx ms on CPU1, read-only (no MMIO/PMIC/I2C data writes, no SPI)
audio selftest A1: wdt: deadline 145000 ms armed (WTCON=0x.. timeout 30 s, counter moving)
audio selftest A1: pmu: AUD_CONFIGURATION=0x.. AUD_STATUS=0x.. (pd-aud on|off|partial) ABOX_CPU_CONFIGURATION=.. ABOX_CPU_STATUS=.. (cpu off) ...
audio selftest A1: gpio: codec-irq gpa2-2 con=0x.. (...) dat=. pud=0x.     (10 lines)
audio selftest A1: jack: detection is the codec's own JD2/MICDET (cirrus,accdet), no AP GPIO; ... (dat=.)
audio selftest A1: pmic: L33CTRL[0x6a]=0xec en=3 vsel=0x2c -> 1800000 uV, on (...)
audio selftest A1: pmic: L34CTRL[0x6b]=0x.. en=. vsel=0x28 -> 1200000 uV, off|on (...)
audio selftest A1: amp 0x39: rev 0x4. (rev A|B, ID at 0x0402|0x0600) GLOBAL_SHDN[0x0400|0x0500]=0x00 GLOBAL_EN=0 INT_RAW1-3=0x.. 0x.. 0x..
audio selftest A1: amp 0x3a: (same)
audio selftest A1: codec: USI01 SW_CONF[0x1041100c]=0x. (...) spi-pins=... reset gpg1-0 ... L33=1.8V on L34=...
audio selftest A1: codec: no SPI transfer in A1 (skip(...)); bringing any missing precondition up is a write: A3 [G1]
audio selftest A1: partial: pd_aud=... (pd-aud reads next)
audio selftest A1: pd-aud: reading pinctrl_aud gpb0..gpb2 (0x17c60000)      (only if pd-aud on)
audio selftest A1: pd-aud: gpb0 CON=... DAT=... PUD=...                       (x3)
audio selftest A1: pd-aud: reading SYSREG_AUD MISC_CON (0x17c1032c)
audio selftest A1: pd-aud: SYSREG_AUD MISC_CON=0x.
audio selftest A1: pd-aud: reading ABOX SFR (0x17c50000)
audio selftest A1: pd-aud: ABOX IP_INDEX=0x.. VERSION=0x.. SYSPOWER_CTRL=0x.. SYSPOWER_STATUS=0x..
audio selftest A1: pd-aud: reading ABOX SysMMU (0x17e00000)
audio selftest A1: pd-aud: SysMMU CTRL=0x.. VERSION=0x7......
audio selftest A1: pd-aud: AUD_STATUS after=0x.f (still on)
```

**PASS line** (one of):

```
audio selftest A1: PASS pd_aud=on(read) abox_cpu=off l33=0xec (on) l34=0x28 (off) amp39 rev=0x4.(A) en=0 amp3a rev=0x4.(A) en=0 codec_spi=skip(l34-off) jack=codec-jd2 wdt=armed writes=0
audio selftest A1: PASS pd_aud=off(skipped) abox_cpu=off l33=0xec (on) l34=0x28 (off) amp39 rev=... en=0 amp3a rev=... en=0 codec_spi=skip(l34-off) jack=codec-jd2 wdt=armed writes=0
```

PASS = watchdog armed, PMU read, PMIC driver bound and both LDO reads OK,
L33 enabled, both amps ACK every read with a known revision, no amp
enabled, pd-aud unchanged across its reads, no PMIC write counted. pd-aud
off is a valid PASS (reads skipped by design). LDO34 off, the codec
preconditions and the rev letter are information, not failures.
`writes=` is the S2MPS18 driver's write counter delta over A1 (must be 0);
this driver has no write call.

## 7. Failure signatures

| Line / symptom | Meaning | Next |
|---|---|---|
| `FAIL ... fails: wdt` with `wdt: NOT armed (...)` | no guarded CL0 watchdog or it refused the deadline; pd-aud reads were skipped | check the s3c2410-wdt probe lines; nothing else is wrong |
| `pmu: read FAILED` | PMU syscon not reachable | should not happen (same syscon as the SysMMU pd-status check) |
| `pmic: S2MPS18 ACPM driver not bound` / `L3xCTRL read FAILED -110` | ACPM ch2 not up or timeout | see the `s2mps18-acpm` probe line and the 106 s `pmic selftest:` |
| `fails: l33-off` | codec has no I/O supply; codec stages cannot start | stop audio work, report |
| `amp: HSI2C18 (0x108c0000) adapter not registered` | USI07/HSI2C18 did not probe (clock, SW_CONF readback `USIv2 configuration readback failed`, IRQ) | check `108c00c0.usi` / `108c0000.i2c` lines |
| `amp 0x3.: rev ID read FAILED -6 (NACK)` (-ENXIO) | no ACK: wrong bus/USI mode/pins, or the amp has no supply | record; no write retry. Compare gpp4-4/5 `con=0x2` |
| `amp 0x3.: ... (bus timeout)` / `9810 timeout:` from i2c-exynos5 | SCL held or clocking wrong | record the `9810 timing:` line |
| `amp 0x3.: rev ID 0x.. below 0x40 at 0x0402 and 0x0600` | not a MAX98512 as 4.9 knows it | stop, report |
| `STOP ... en=1` and the `STOP:` line | an amp has GLOBAL_EN=1 left from before this kernel | owner decides (design A1); A1 wrote nothing |
| `pd-aud: reading <block>` is the last line, then nothing; reset at <= 145+30 s or PID1 restart at 120 s | that block stalled the bus (pd-aud on but its clock gated, or an access not allowed from the non-secure side) | the named block must not be read again without a clock/QCH check; rerun without it |
| `SError Interrupt on CPU1` right after `pd-aud: reading <block>` | that block answered with a bus error; panic=30 reboots | as above |
| `pd-aud: AUD_STATUS after=... (CHANGED)` / `fails: pd-aud-changed` | someone switched pd-aud while A1 read it | find who; 6.12 has no pd-aud genpd |
| `fails: pmic-writes` | the PMIC write counter moved during A1 | must not happen with the sealed policy; report |

## 8. Recovery path

As for the other delayed-UFS trials: PID1 reboot at 120 s (CPU0-pinned
timer child, parent fallback 150 s), shim bbea959 bounce to Lineage. A
CPU wedged on a pd-aud read is CPU1, not CPU0, so the restart timer still
runs; if the restart itself blocks (e.g. on RCU waiting for the wedged
CPU), the watchdog's ping path refuses every reload after 145 s and the CL0
watchdog resets the SoC within 30 s. If the phone does not return: owner
force restart (Power + Vol Down ~7-10 s), shim bounces to Lineage, restore
bright RECOVERY `8763860c` by hand and wait for AshLooper `boot=booted`.

## 9. Risks

- **pd-aud reads with the AUD bus clock unknown.** AUD_STATUS = 0xf
  proves the power, not the ABOX/SysMMU clocks. Mitigation: these reads
  come last (all other results are printed first, plus a `partial:` line),
  each block is announced, they run on CPU1, and the deadline is armed.
  The pinctrl_aud block goes first: 6.12 already reads its CON registers
  at every boot (gpiolib), so a stall there would be new.
- **HSI2C18 bring-up writes** (2.2): same pattern as the touch bus, but
  USI07 and its clock are new to 6.12. A failure there only loses the amp
  reads.
- **I2C register-address message:** a MAX98512 treats a 2-byte write
  followed by a repeated start as a pointer set. If the controller sent a
  STOP instead, the pointer is still all that changes (no data byte).
- **Codec SPI:** not attempted (2.1).
- Not reviewed by Sol yet.
