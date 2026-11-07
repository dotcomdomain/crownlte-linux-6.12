# crownlte 6.12: audio stage A1, read-only hardware probe

Design: gekko `roadmap/audio-design.md`, section 5 "A1" (also sections 1, 4.1,
4.3, 7). Branch `crownlte-audio-a1` on `crownlte-suspend-s0` 60774ac87.
Build only; not run on the phone yet. Fix round 1 (Sol R1: 4 findings)
is applied; see section 10.

A1 answers the design's open questions 1 and 2 (amp silicon revision, is
LDO34 on) and reports the PMU's pd-aud power status for question 3. It
checks every line the later stages depend on. It reads only. Nothing
writes to the codec, the amps, the PMIC, ABOX, SYSREG_AUD, the SysMMU or
any power domain, and nothing inside pd-aud is read (since fix round 1).

## 1. Why this base

`crownlte-suspend-s0` 60774ac87 rather than `crownlte-integration-6`
4148bf525:

- it has the CL0 watchdog deadline API (`include/soc/samsung/crownlte-wdt.h`,
  `s3c2410wdt_crownlte_set_deadline()` in the ping path, commits 766bcf035
  and 7b58093af). A1 uses it to cover a stalled bus access. int6 has no
  deadline API;
- its delayed-UFS vehicle passed on hardware with this exact config
  (`config-suspend-s0-ee215e307`, pack 416fb7f3, S0a-S0d PASS);
- int6 adds perf/DPU SysMMU/Wi-Fi/TEE, none of which A1 needs. A1 is a
  few small commits (one new driver and header, an opt-in mode in
  i2c-exynos5, a guard hook in exynos-usi, a disabled bus node, one new
  DTS).

## 2. What A1 does

### 2.1 At probe (device initcall, ~1-3 s): deadline first, then the bus

1. Refuse if no CPU other than CPU0 is online (no self-test, bus never
   comes up).
2. Install the CL0 watchdog deadline 145000 ms
   (`samsung,audio-a1-deadline-ms`) and verify it: watchdog enabled,
   reset on, still petted, deadline <= ours, WTCNT moving (same check as
   `exynos9810-pm.c` S0). If the guarded watchdog is not bound yet, A1
   defers (`waiting for the guarded CL0 watchdog (amp bus held back)`).
3. Only then open the guard `exynos9810_audio_a1_guard()`.

The amp bus devices `usi_amp` (USI07 wrapper) and `hsi2c_amp` (HSI2C18)
carry `samsung,audio-a1-guard = <&audio_a1>` in the A1 DTB. Both call the
guard as the first thing in their probe, before any SYSREG, clock or
register access:

| A1 state | guard returns | bus device |
|---|---|---|
| not probed yet / waiting for the watchdog | -EPROBE_DEFER | deferred; re-probed when A1 binds |
| deadline armed and verified | 0, after a live re-check of the watchdog state and >= 20 s left to the deadline | probes |
| refused (no deadline, verify failed, no nonzero CPU, bad DT) or A1 not built | -EPERM / -EINVAL / -ENODEV | fails for this boot |

`hsi2c_amp` only exists once `usi_amp` probed (exynos-usi populates its
children), so the HSI2C18 pin mux ("default" state, applied by the driver
core just before the HSI2C probe) also comes after the guard opened.

### 2.2 Self-test at ~30 s, on a nonzero CPU

Self-test `audio selftest A1:` from DT `samsung,audio-a1-selftest-ms`
(30000), queued on the first online CPU after CPU0 (CPU1), so that a CPU
wedged on a read leaves the CPU0-pinned PID1 restart timer alone. The
work item pins itself (`migrate_disable()`; the scheduler's hotplug path
waits for pinned tasks, so that CPU cannot go offline under A1) and
checks `smp_processor_id() != 0` before any access. If its CPU went
offline before the work ran, the workqueue runs it unbound; if that puts
it on CPU0, A1 refuses (`fails: cpu0`) without reading anything.

| # | Block | Registers | Precedent / gate |
|---|---|---|---|
| 1 | CL0 watchdog | deadline from probe still installed, enabled, reset on, still petted, WTCNT moving | if not: `wdt: NOT armed`, amp reads skipped |
| 2 | PMU 0x14060000 (always on, syscon) | AUD_CONFIGURATION 0x4000, **AUD_STATUS 0x4004**, ABOX_CPU_CONFIGURATION 0x415c, ABOX_CPU_STATUS 0x4160, ABOX_CPU_OPTION 0x4164, ABOX_CPU_STANDBY 0x3804, PAD_RETENTION_ABOX_OPTION 0x4170, GPIO_MODE_ABOX_SYS_PWR_REG 0x1424 | 4.9 pmucal `aud_status`, `abox.c` reads/updates of each. Status only: nothing inside pd-aud is read |
| 3 | GPIO (pinctrl alive, peric0, peric1, cmgp) | CON/DAT/PUD of codec IRQ gpa2-2, codec reset gpg1-0, codec SPI gpp0-4..7, amp IRQs gpm2-0/gpm3-0, amp I2C gpp4-4/5 | 6.12 gpiolib reads every bank CON at boot; 4.9 |
| 4 | S2MPS18 over ACPM ch2 | PMIC L33CTRL 0x6a, L34CTRL 0x6b (`s2mps18_acpm_read`) | read-only ACPM path; the transport's write policy is sealed empty in this DTB |
| 5 | MAX98512 0x39, 0x3a on HSI2C18 (0x108c0000) | 0x0402 rev ID; 0x0600 only if 0x0402 < 0x40 (rev B); GLOBAL_SHDN 0x0400 (rev A) or 0x0500 (rev B), bit 0 = GLOBAL_EN; INT_RAW1-3 0x0001-0x0003 | 4.9 `max98512_i2c_probe` order; each read is a 2-byte register-address message + repeated start + 1-byte read; no data byte is sent. Off-whitelist registers are refused in A1; any other message shape is refused by the adapter (2.4). The first failed transfer ends all amp traffic for the boot: the other amp is not read |
| 6 | Codec CS47L93 | no SPI transfer. Reads PERIC0 SYSREG USI01 SW_CONF 0x1041100c and evaluates the SPI ID-read preconditions | see 2.3 |

Removed in fix round 1: pinctrl_aud gpb0..2, SYSREG_AUD MISC_CON, ABOX
SFR and ABOX SysMMU reads. AUD_STATUS = 0xf proves pd-aud power, not its
clocks or that the non-secure side may access those blocks.

The jack has no AP GPIO: detection is the codec's own JD2/MICDET
(`cirrus,accdet`). The codec IRQ line gpa2-2 is the only AP-visible signal;
its level is logged. Jack state itself needs the codec (A3).

### 2.3 Why no codec SPI read

The silicon ID (0x6371 at register 0) needs USI01 in SPI mode, the
gpp0-4..7 pins muxed to SPI, the codec reset gpg1-0 released and DCVDD
(LDO34) on. The 6.12 tree has no SPI-mode USI or 9810 SPI controller
support, and bringing any of these up is a write (SYSREG, pin mux, GPIO,
PMIC = gate G1). A plain-protocol mistake would also be costly: a madera
SPI frame without the read bit on register 0 is a software-reset write.
So A1 only reports which preconditions hold (`codec_spi=skip(<first
missing>)`); the ID read moves to A3 with mainline madera.

### 2.4 HSI2C18 in A1 read-only mode

`hsi2c_amp` has `samsung,crownlte-a1-read-only` (accepted only on
`samsung,exynos9810-hsi2c`; it also makes the A1 guard mandatory). In
that mode i2c-exynos5:

- accepts only `[write, 2 bytes][read, 1 byte]` to one 7-bit address
  (flags other than `I2C_M_RD`, ten-bit, `NOSTART`, `IGNORE_NAK` etc. are
  refused); a refusal touches no register (it is checked before the
  clocks are enabled);
- sets `adap.retries = 0`: no automatic retry, also not after
  arbitration loss (`-EAGAIN`);
- never runs the stuck-bus recovery (nine manual read clocks + STOP). If
  TRANS_STATUS shows LOSE (SDA held low) before a message, the transfer
  is refused before MASTER_RUN;
- closes itself for the boot after the first failure of any kind (NACK,
  timeout, arbitration loss, bus busy, idle wait, refused shape, clock
  enable error): IRQ disabled, INT_ENABLE cleared, controller put in
  software reset (HSI2C_CTL SW_RST) and left there. Later transfers
  return -EIO without touching the hardware; resume skips the re-init.

What the amps can see on a failure path:

- The controller's SCL and SDA pads are open drain. SW_RST releases both;
  the bus pull-ups take them high. The controller issues no clock pulse,
  no START and no STOP command after the failure, now or later in the
  boot.
- If the controller was holding SCL low in the middle of a byte, the
  release is one SCL rising edge. A slave can at most sample one more bit
  of the byte in progress. In an A1 transfer the only bytes the master
  ever sends are the slave address and the two register-address bytes;
  a register write would need a complete third (data) byte with 8 more
  clocks and an ACK clock, which never come. At worst the amp's register
  pointer is wrong, which is not a register change.
- Releasing lines can never produce a START (SDA falling while SCL is
  high needs SDA driven low). If SDA rises while SCL is high, the amp sees
  a STOP, which ends its transaction without a write. If the amp itself
  holds SDA low (ACK or a read bit), it keeps holding it until it sees a
  further SCL falling edge; the bus stays like that until the next boot,
  whose first START/STOP (4.9 max98512 probe) resets the amp's interface.
- A NACK in auto mode may let the controller finish with its own STOP
  before the driver sees the error; that is part of the normal transfer,
  not an extra transaction.

### 2.5 Writes that do happen in an A1 boot (not by A1), all after the guard opened

- `exynos-usi` for `usi_amp`: PERIC1 SYSREG USI07 SW_CONF (0x10811024)
  := I2C, USI07 wrapper USI_CON/USI_OPTION, the USI07 clocks
  (`GATE_USI07`, `USI07`), exactly as for the touch bus (USI03).
- `i2c-exynos5` for `hsi2c_amp`: HSI2C18 controller reset/timing
  registers; on a failure, SW_RST (2.4).
- pinctrl "default" of the bus: gpp4-4/gpp4-5 to function 2, no pull
  (the 4.9 `hsi2c18-bus` values).
- Everything the delayed-UFS diagnostic always does.

No amp child node exists, so no amp driver binds and the i2c core scans
nothing (no adapter class, no I2C_CHARDEV, no bus recovery info).

## 3. Code

- `drivers/soc/samsung/exynos9810-audio-a1.c` (CONFIG_EXYNOS9810_AUDIO_A1,
  bool): platform driver `samsung,exynos9810-audio-a1`. Refused (bound,
  idle, guard closed) without `samsung,audio-a1-selftest-ms` (allowed
  20000..100000), without a valid `samsung,audio-a1-deadline-ms`
  (121000..180000) or without an online nonzero CPU. The DT windows are
  mapped with `devm_ioremap()` without claiming them. Exports
  `exynos9810_audio_a1_guard()`.
- `include/soc/samsung/exynos9810-audio-a1.h`: the guard; stub returns
  -ENODEV (guarded devices never come up without A1).
- `drivers/i2c/busses/i2c-exynos5.c`: `samsung,crownlte-a1-read-only`
  (2.4) and the guard call (A1 mode or `samsung,audio-a1-guard`). Other
  adapters unchanged.
- `drivers/soc/samsung/exynos-usi.c`: guard call when the node has
  `samsung,audio-a1-guard`. Other USIs unchanged.
- `exynos9810-crownlte-clocks.dtsi`: `usi_peric1_07_sysreg`, `usi_amp`
  (`usi@108c00c0`) and `hsi2c_amp` (`i2c@108c0000`, SPI 430, 400 kHz,
  `hsi2c18_bus` pins), all **disabled** and without children.
- `exynos9810-crownlte-delayed-ufs-audio-a1-diag.dts`: delayed-UFS
  diagnostic + bus enabled with the guard and A1 mode + `audio-a1@14050000`
  node (five always-on windows); the other self-tests removed as in the
  suspend S0 DTB (cpuidle, MCT report, devfreq, TMU, cpufreq, display,
  battery, ADC; Mali disabled). The read-only PMIC snapshot at ~106 s
  stays.

Config: `config-suspend-s0-ee215e307` (1ec1bef6) +
`CONFIG_EXYNOS9810_AUDIO_A1=y`; no other difference after olddefconfig.

## 4. Artifacts

Current (fix round 1):

| Item | Value |
|---|---|
| Commits | 6d2c7fdfc A1 driver + guard, c350c9cf3 i2c-exynos5 A1 mode, f76aa18de exynos-usi guard, 0e90c992d DTS, this doc |
| Image | `panel-bright-work/images/Image-audio-a1-r1-0e90c992d` sha256 `33fb884522f085caabd40e027034fb4a0d63a3874c994e22b0da7d98b44b0e6e` |
| DTB | `panel-bright-work/dtb/exynos9810-crownlte-delayed-ufs-audio-a1-diag-0e90c992d.dtb` sha256 `ec5ab02d67d4a2d20d76dbd94090f0bb922ba844f00158e0b2863d6037f7c21c` |
| Pack | gekko `acpm-audio-a1-r1-s2/recovery.img` sha256 `1abac03f4c29e28189a3ba903a0b8417b592aaffeae99f141d5711a965442a23` (68149248 bytes); loader `be816f09...`; shim bbea959; ramdisk 506232b1; parent a8198be0; see `PROVENANCE.txt` |

Superseded (757cbf76f, before fix round 1, must not be run):
`acpm-audio-a1-s2` recovery `c7601027...`, Image `490e3ab8...`, DTB
`35c9c7aa...`. The pack directory was deleted from gekko; a copy may
still be on the PC.

## 5. Running it

Same procedure as the other delayed-UFS trials: copy `acpm-audio-a1-r1-s2`
to the PC (it includes `ready-capture.ps1`), start only with
`adb reboot recovery`, then `run-trial.sh <dir>` (the default 180 s
capture is enough). The host sends `CROWNLTE-UFS-READY-1` once. **Never
send `CROWNLTE-UFS-PROBE-1`.** No physical help needed (no audio output,
no headset).

| Boot time | What |
|---|---|
| boot | `s3c2410-wdt ...: crownlte deadline: no reload after 145000 ms boot time`; `audio selftest A1: wdt (probe): deadline 145000 ms armed (...)`; `exynos9810-audio-a1 14050000.audio-a1: audio A1 read-only probe scheduled at ~30000 ms on CPU1, watchdog deadline 145000 ms armed, amp bus guard open` |
| boot, after that | `exynos-usi 108c00c0.usi: audio A1 guard: CL0 watchdog deadline 145000 ms armed, bring-up allowed at ... ms`; `exynos-usi 108c00c0.usi: USIv2 configured: sw_conf=00000004 ...`; `exynos5-hsi2c 108c0000.i2c: audio A1 guard: ...`; `exynos5-hsi2c 108c0000.i2c: 9810 timing: clock=... bus=400000 ...`; `exynos5-hsi2c 108c0000.i2c: A1 read-only mode: ...` |
| 30 s | the A1 lines (section 6) |
| ~106 s | `pmic selftest:` (read-only) |
| 120 s | PID1 restart (150 s fallback); shim -> Lineage |
| 145 s | only if still up: watchdog refuses reloads, reset within 30 s |

The guard lines must come **after** the `deadline ... armed` lines; a
`USIv2 configured` or `9810 timing` line for 108c0000/108c00c0 before
them is a failure of the fix.

After return, `run-trial.sh` saves `/sys/fs/pstore/pmsg-ramoops-0` and
restores bright RECOVERY `8763860c`.

## 6. Expected lines

```
audio selftest A1: start uptime=30xxx ms on CPU1 (pinned until the end of A1), read-only (no MMIO/PMIC/I2C data writes, no SPI, nothing inside pd-aud)
audio selftest A1: wdt (self-test): deadline 145000 ms armed (WTCON=0x.. timeout 30 s, counter moving)
audio selftest A1: pmu: AUD_CONFIGURATION=0x.. AUD_STATUS=0x.. (pd-aud on|off|partial, status only: nothing inside pd-aud is read) ABOX_CPU_CONFIGURATION=.. ABOX_CPU_STATUS=.. (cpu off) ...
audio selftest A1: gpio: codec-irq gpa2-2 con=0x.. (...) dat=. pud=0x.     (10 lines)
audio selftest A1: jack: detection is the codec's own JD2/MICDET (cirrus,accdet), no AP GPIO; ... (dat=.)
audio selftest A1: pmic: L33CTRL[0x6a]=0xec en=3 vsel=0x2c -> 1800000 uV, on (...)
audio selftest A1: pmic: L34CTRL[0x6b]=0x.. en=. vsel=0x28 -> 1200000 uV, off|on (...)
audio selftest A1: amp 0x39: rev 0x4. (rev A|B, ID at 0x0402|0x0600) GLOBAL_SHDN[0x0400|0x0500]=0x00 GLOBAL_EN=0 INT_RAW1-3=0x.. 0x.. 0x..
audio selftest A1: amp 0x3a: (same)
audio selftest A1: codec: USI01 SW_CONF[0x1041100c]=0x. (...) spi-pins=... reset gpg1-0 ... L33=1.8V on L34=...
audio selftest A1: codec: no SPI transfer in A1 (skip(...)); bringing any missing precondition up is a write: A3 [G1]
```

**PASS line** (pd_aud is the PMU status; on, off and partial all PASS):

```
audio selftest A1: PASS pd_aud=on(status) abox_cpu=off l33=0xec (on) l34=0x28 (off) amp39 rev=0x4.(A) en=0 amp3a rev=0x4.(A) en=0 codec_spi=skip(l34-off) jack=codec-jd2 wdt=armed cpu=1 writes=0
```

PASS = watchdog deadline armed at probe and still armed, PMU read, PMIC
driver bound and both LDO reads OK, L33 enabled, both amps ACK every read
with a known revision, no amp enabled, no PMIC write counted. LDO34 off,
pd-aud state, the codec preconditions and the rev letter are
information, not failures. `writes=` is the S2MPS18 driver's write
counter delta over A1 (must be 0); this driver has no write call.

## 7. Failure signatures

| Line / symptom | Meaning | Next |
|---|---|---|
| `deferred probe pending: ... waiting for the guarded CL0 watchdog (amp bus held back)`, no A1 lines | the guarded watchdog never bound: A1 and the amp bus stay deferred (fail closed) | check the s3c2410-wdt probe lines |
| `watchdog deadline NOT armed (...)`: amp bus bring-up refused, then `exynos-usi 108c00c0.usi: audio A1 guard: A1 refused (...)` and later `FAIL ... fails: wdt amp-skipped` | watchdog refused the deadline or did not verify | check the s3c2410-wdt lines; nothing else is wrong |
| `audio A1 refused (no CPU other than CPU0 online)` | secondaries did not come up | not an audio problem |
| `FAIL: self-test found itself on CPU0 ... fails: cpu0` | CPU1 went offline before 30 s, the work ran unbound on CPU0; nothing was read | find who offlined CPU1 |
| `USIv2 configured` / `9810 timing` for 108c00c0/108c0000 **before** the A1 `deadline ... armed` lines | guard bypassed | stop, report (fix failure) |
| `pmu: read FAILED` | PMU syscon not reachable | should not happen (same syscon as the SysMMU pd-status check) |
| `pmic: S2MPS18 ACPM driver not bound` / `L3xCTRL read FAILED -110` | ACPM ch2 not up or timeout | see the `s2mps18-acpm` probe line and the 106 s `pmic selftest:` |
| `fails: l33-off` | codec has no I/O supply; codec stages cannot start | stop audio work, report |
| `amp: HSI2C18 (0x108c0000) adapter not registered` | USI07/HSI2C18 did not probe (guard, clock, SW_CONF readback `USIv2 configuration readback failed`, IRQ) | check `108c00c0.usi` / `108c0000.i2c` lines |
| `amp 0x3.: rev ID read FAILED -6 (NACK)` (-ENXIO), then `A1 read-only mode: transfer failed (-6): adapter closed for this boot (IRQ off, controller held in reset, ...)` and `amp 0x3a: not read: amp bus closed ...` | no ACK: wrong bus/USI mode/pins, or the amp has no supply | record; no retry by design. Compare gpp4-4/5 `con=0x2` |
| `... (bus timeout)` + `9810 timeout:` + adapter closed | SCL held or clocking wrong | record the `9810 timing:` and `9810 timeout:` lines |
| `... (arbitration lost)` / `... (bus busy or stuck)` / `A1 read-only mode: bus stuck before a transfer (TRANS_STATUS ...)` | another master or a slave holding SDA; no recovery clocks were sent | record; the next boot's START/STOP resets the amps' interface |
| `A1 read-only mode: transfer refused: only [2-byte ...` | a non-A1 client tried the bus | must not happen (no child, no i2c-dev); report |
| `amp 0x3.: rev ID 0x.. below 0x40 at 0x0402 and 0x0600` | not a MAX98512 as 4.9 knows it | stop, report |
| `STOP ... en=1` and the `STOP:` line | an amp has GLOBAL_EN=1 left from before this kernel | owner decides (design A1); A1 wrote nothing |
| `fails: pmic-writes` | the PMIC write counter moved during A1 | must not happen with the sealed policy; report |

## 8. Recovery path

As for the other delayed-UFS trials: PID1 reboot at 120 s (CPU0-pinned
timer child, parent fallback 150 s), shim bbea959 bounce to Lineage. The
deadline is armed at probe, so it covers the bus bring-up as well as the
30 s reads. A CPU wedged on an A1 read is CPU1, not CPU0, so the restart
timer still runs; a wedged bring-up runs in whichever CPU probes it, so
the restart timer may be blocked too, but the watchdog's ping path
refuses every reload after 145 s and the CL0 watchdog resets the SoC
within 30 s. If the phone does not return: owner force restart (Power +
Vol Down ~7-10 s), shim bounces to Lineage, restore bright RECOVERY
`8763860c` by hand and wait for AshLooper `boot=booted`.

## 9. Risks (after fix round 1)

- **HSI2C18 bring-up writes** (2.5): same pattern as the touch bus, but
  USI07 and its clock are new to 6.12. Now behind the deadline. A failure
  there only loses the amp reads; a stall is a watchdog reset.
- **Pin mux switch of gpp4-4/5 at bring-up**: switching the pads from
  their boot state to the HSI2C function can glitch the lines once.
  Before probe the controller drives nothing; a glitch cannot form a
  complete address+register+data sequence, so the worst case is a
  spurious START/STOP seen by the amps, no register change. Unchanged
  from the original A1 and from the touch bus.
- **One release edge on a failure** (2.4): bounded to one SCL edge and
  one possible STOP; no data byte can complete.
- **NACK on the first amp ends the run**: by design (no bus traffic after
  any error); 0x3a is then not read, A1 reports FAIL and a second trial
  would be needed.
- **pd-aud blocks are no longer probed**: question 3 is answered only by
  the PMU power status; the ABOX/SysMMU version reads move to a later
  stage with a proper clock/QCH check.
- **Bring-up CPU**: the USI/HSI2C probes run on whatever CPU the driver
  core uses (possibly CPU0); only the A1 reads are pinned off CPU0. The
  deadline covers both.
- **Watchdog accesses**: arming at probe reads WTCON/WTCNT through the
  watchdog driver's API from the probing CPU, as its own ping path does.
- Not reviewed by Sol after fix round 1 yet.

## 10. Fix log

### Fix round 1 (Sol R1 on 757cbf76f: CHANGES, not safe unattended)

| # | Sol finding | Fix | Commit |
|---|---|---|---|
| 1 | HIGH: stuck-bus recovery (`exynos5_i2c_bus_check()` -> `exynos5_i2c_bus_recover()`: nine manual read clocks + STOP outside the messages) escapes the amp whitelist; arbitration loss allows three retries | i2c-exynos5 `samsung,crownlte-a1-read-only` mode (2.4): message-shape whitelist in the adapter, retries 0, no recovery clocks (stuck bus refused before MASTER_RUN), first failure closes the adapter for the boot (IRQ off, controller held in SW_RST, no re-init); slave view documented in 2.4. A1 itself stops all amp reads after the first failure | c350c9cf3, 6d2c7fdfc |
| 2 | HIGH: AUD_STATUS = 0xf proves power, not clocks or accessibility | pinctrl_aud, SYSREG_AUD, ABOX SFR, ABOX SysMMU windows and reads dropped (driver and DT); PMU status snapshot kept | 6d2c7fdfc, 0e90c992d |
| 3 | HIGH: the 30 s deadline does not cover the usi_amp/hsi2c_amp bring-up at boot | deadline installed and verified in A1 probe; `exynos9810_audio_a1_guard()` called first in the exynos-usi and i2c-exynos5 probes via `samsung,audio-a1-guard` (deferred probe until armed, permanent refusal if A1 refused or is not built) | 6d2c7fdfc, c350c9cf3, f76aa18de, 0e90c992d |
| 4 | MEDIUM: falls back to CPU0 if only CPU0 is online | refused at probe without a nonzero CPU; the work pins itself with `migrate_disable()` (offlining waits) and refuses on CPU0 before any access | 6d2c7fdfc |

Build: `config-suspend-s0-ee215e307` + A1, clang 18, no warnings in the
changed files (W=1). Pack: same packer, shim, ramdisk and parent as
before (section 4).
