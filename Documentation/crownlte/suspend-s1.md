# crownlte 6.12: suspend stage S1a/S1b (AP RTC probe, alarm while awake)

Branch `crownlte-suspend-s1`, on `crownlte-suspend-s0` 60774ac87 (S0a-S0d passed on hardware 2026-11-07, pack 416fb7f3).
Design: gekko `roadmap/suspend-s1-design.md` ("the design"), sections 3.1 and 5 (S1a, S1b).
BUILD-ONLY: nothing here has run on the phone yet.

## 1. What S1a/S1b do, and what they never do

S1a and S1b are the first half of stage S1. They prove, without sleeping, that the SoC's own RTC (the "AP RTC" at `0x141E0000`, SPI 31 alarm) can be the wake source for the first real SYS_SLEEP:

- **S1a** reads it (non-secure? counting? free?).
- **S1b** arms a seconds-only alarm while awake and checks that it fires, then disarms it and restores the block.

Both stages are self-recovering. They never:

- sleep, call PSCI or any SMC (the S0 dry finisher is not even reached: S0c/S0d are off, so no suspend or syscore ops are registered);
- arm the APM hand-off (INTR_GEN GRP2);
- write the PMIC (no `samsung,rtc-write-enable`, no `samsung,rtc-read-latch`: the ACPM write policy stays sealed empty);
- write UFS (stays unbound; never send `CROWNLTE-UFS-PROBE-1`) or the charger;
- write the PMU, the GIC or anything outside the AP RTC.

The only new writes are AP RTC `INTP`, `ALMSEC`, `RTCALM` and, only if a write does not read back, a `RTCCON.RTCEN` set that is restored right after.

## 2. Commits

| Commit | What |
|---|---|
| 4abfd3d05 | `exynos9810-pm`: S1a/S1b (DT `samsung,suspend-s1`), Kconfig help |
| 8b1f3132b | `s2mps18-acpm`: optional read-only PWREN_SEL1/2 dump (DT `samsung,pmic-selftest-pwren-sel`) |
| c8544e54c | DTS `exynos9810-crownlte-delayed-ufs-suspend-s1-diag` |
| 4b70afda9 | watchdog early re-arm after a firmware sleep, **unused** (section 7) |
| this file | documentation |

## 3. Artifacts (gekko)

| Item | Path | sha256 |
|---|---|---|
| Image (4b70afda9) | `panel-bright-work/images/Image-suspend-s1-4b70afda9` | `9271a32a33512856cde43c667febc4df1b3ae2edbd02b39df6e9e9f3a4d46351` |
| config | `panel-bright-work/images/config-suspend-s1-4b70afda9` | `1ec1bef6...` (identical to the S0 r3 config) |
| DTB | `panel-bright-work/dtb/exynos9810-crownlte-delayed-ufs-suspend-s1-diag-4b70afda9.dtb` | `ac505b350e316d5a0331f19e26809aa7afd04601b66a286153e534327ed5f675` |
| RECOVERY pack | `acpm-suspend-s1ab-s2/recovery.img` | `45342bb89d43e6ea14d13c7db0a951a68584a0dd1846c1350d306796ddbafa64` |

Pack: `pack-acpm-q-paths.py`, shim `bbea9590ba9bf57c673cee723e4e6817c08cdea7`, ramdisk `506232b1` (CPU0-pinned PID1, restart at 120 s, fallback at 150 s), parent `a8198be0`. The rebuilt S0 DTB from this tree is byte-identical to `61e670ec`.

The S1 DTB differs from the S0 DTB in exactly these places (decompiled diff):

- `model`;
- `system-suspend@14070000`: two more `reg` windows, `"ap-rtc"` `<0x0 0x141e0000 0x100>` and `"apm-qch-rtc"` `<0x0 0x14003028 0x4>`;
- `samsung,suspend-s0 = "a", "b"` (was a..d) and `samsung,suspend-s1 = "a", "b"`;
- `s2mps18_pmic`: `samsung,pmic-selftest-pwren-sel`.

## 4. Trial procedure

Same vehicle as S0 r3:

1. Start only via `adb reboot recovery` with the pack in RECOVERY.
2. Host sends `CROWNLTE-UFS-READY-1` once, **never** `PROBE`. Use the r3 `ready-capture.ps1` (copied into the pack directory). Capture **≥ 180 s**.
3. Expected timeline (boot time):
   - probe: deadline 145000 ms installed and verified before any diagnostic read;
   - 20 s: S0a (snapshot), then **S1a** (~2.2 s), then **S1b** (two alarms, ≤ ~25 s worst case, ~12 s typical), then S0b (hotplug regression, ~11 s);
   - SUMMARY lines 4 times, 5 s apart;
   - ~106 s: PMIC read-only self-test (now with PWREN_SEL1/2);
   - 120 s: PID1 restart to Lineage (shim bounce).
4. After return: save `/sys/fs/pstore/pmsg-ramoops-0` and `dmesg-ramoops-*`, restore bright RECOVERY `8763860c`, wait for AshLooper `boot=booted` before the next reboot.

## 5. Expected lines (PASS)

Prefix `suspend s1:` (S0 lines keep `suspend s0:`). Values in `<>` vary.

### S1a (read only)

```
suspend s1: probe: stages ab (no sleep, no SMC, no PMIC write; AP RTC at 0x00000000141e0000 mapped, not yet read)
suspend s1: S1a start: read-only AP RTC probe at 0x00000000141e0000 (+0x30..+0x90), CMU_APM QCH_CON_APBIF_RTC, PMU 0x404/0x408/0x40c, GICD SPI31; first read follows
suspend s1: S1a ap-rtc first read survived (no abort, no stall)
suspend s1: S1a #1 ap-rtc RTCCON=<x> (rtcen=<0|1> cntsel=0 clkrst=0 ticen=0) INTP=0x0 RTCALM=<x, bit6 0> TICCNT=<x> CURTICCNT=<x>
suspend s1: S1a #1 ap-rtc ALMSEC..ALMYEAR=<6 values> BCDSEC..BCDYEAR=<7 values>
suspend s1: S1a #2 ...                                   (same two lines, second read)
suspend s1: S1a ap-rtc RTCCON=<x> (rtcen=<0|1> clkrst=0 ticen=0) INTP=0x0 RTCALM=<x> TICCNT=<x> QCH_APBIF_RTC=<x>
suspend s1: S1a ap-rtc t1=<yy-mm-dd hh:mm:ss> t2=<..> advance <2|3> s over <~2100> ms (bcd ok) -> counting
suspend s1: S1a pmu RST_STAT=<x> WDT_AUTO_DISABLE=0xfeffffff WDT_MASK_RESET=0xfeffffff (bit24 0/0, both 0 = WDT reset reaches the SoC)
suspend s1: S1a gic SPI31 (INTID 63) ISENABLER1 bit31=0 ISPENDR1 bit31=<0|1> (ISENABLER1=<x> ISPENDR1=<x>)
suspend s1: S1a PASS (read-only)
pmic selftest: pmic[0x1c..0x2b]: ...        (B1..B4 OUT2 at 0x1e 0x21 0x24 0x27; 4.9 writes 0x20)
pmic selftest: pmic[0xc8..0xc9]: <PWREN_SEL1> <PWREN_SEL2>   (4.9 writes PWREN_SEL2 = 0xff)
```

Notes:

- The RTC time is the AP RTC's own (BCD, 2-digit year). Whether it matches wall time does not matter; only that it is valid BCD and counts.
- The advance is 2 or 3 s: the reads are 2100 ms apart, so either count of whole-second steps is correct.
- The PMIC lines are for the read-only comparison with live 4.9 (design S1a "also"); a difference is recorded, not a gate.

### S1b (alarm while awake)

```
suspend s1: S1b before ap-rtc RTCCON=<x> ... INTP=0x0 RTCALM=<x> ...
suspend s1: S1b before ap-rtc ALMSEC..ALMYEAR=<..> BCDSEC..BCDYEAR=<..>
suspend s1: S1b arm +3 s: BCDSEC=<s> ALMSEC=<s+3 bcd> RTCALM=0x41 readback ok rtcen-needed=<0|1>
suspend s1: S1b alarm #1 INTP.ALM set after <1900..3999> ms (expected 1900..3999) at BCDSEC=<s+3>, GICD SPI31 pending=<0|1> (before arm <0|1>, after disarm <0|1>), WAKEUP_STAT=<x>/<x> (before <x>/<x>) -> disarmed RTCALM=0x0, INTP=0x0
suspend s1: S1b arm +7 s: ... RTCALM=0x41 readback ok rtcen-needed=<0|1>
suspend s1: S1b alarm #2 INTP.ALM set after <5900..7999> ms (expected 5900..7999) ... -> disarmed RTCALM=0x0, INTP=0x0
suspend s1: S1b after ap-rtc ...            (both register lines again)
suspend s1: S1b restore: INTP 0x0 -> 0x0, RTCCON <x> -> <x>, RTCALM <x> -> <x>, ALMSEC <x> -> <x>, ALMMIN..ALMYEAR unchanged, rtcen-needed=<0|1> -> restored
suspend s1: S1b PASS 2/2
suspend s1: SUMMARY S1a=PASS [counting +<2|3> s/<ms> ms rtcen=<0|1> qch=<x>] S1b=PASS [2/2 alarms, rtcen-needed=<0|1>]
suspend s0: SUMMARY S0a=PASS [...] S0b=PASS [single 140/140 cluster 5/5 group 25/25 ...] S0c=not-run [] S0d=not-run [] lost=
```

What each S1b value tells S1c/S1d:

- `rtcen-needed=1`: alarm writes need `RTCCON.RTCEN`; the S1c arm path must set it the same way.
- `GICD SPI31 pending=1` after the alarm: the alarm line reaches the GIC (SPI 31 = INTID 63). It stays disabled at the distributor, so no interrupt is taken. `0` does not fail the stage: a secure-group SPI reads 0 from the non-secure side. `after disarm 1` means the GIC latched an edge; harmless while the SPI is disabled, but S1c must clear it (`ICPENDR`) before enabling the SPI.
- `WAKEUP_STAT`: expected unchanged while awake (it latches during SLEEP). A new bit 1 here would mean the PMU sees the alarm even while running.

## 6. Failure signatures

| Signature | Meaning | Next |
|---|---|---|
| Log ends at `S1a start ... first read follows`, `synchronous external abort` in `aprtc_snapshot`/`s1a_run` in pstore `dmesg-ramoops`, phone back via panic (panic=30) + shim bounce | AP RTC is secure-only | **AP RTC route dead → S1f (PMIC RTC, owner D1).** |
| Log ends at `first read follows`, no abort, phone back via WDT reset (RST_STAT WDT bit) after the 145 s deadline plus the run-time watchdog timeout | APB stall on the AP RTC (or its Q-channel) | Same: route dead → S1f |
| `S1a FAIL: not counting` / `bcd invalid` / `time read torn 3 times` | No 32 kHz, held, or not a S3C RTC layout | Route dead. Do not "fix" it with RTCCON writes without a new review. |
| `S1a FAIL: CLKRST set` | Counter held in reset | Route dead as is (clearing CLKRST is a new write needing review) |
| `S1a FAIL: RTCALM.ALMEN=1 (another owner)` / `TICEN=1 (another owner)` / `INTP not 0` / `control registers changed between reads` | Someone else (EL3/APM/CHUB/VTS) uses the block | Stop; find the owner before any write |
| `S1b refused: SPI31 enabled at the GIC ...` | Somebody enabled the alarm SPI: an alarm would be an unhandled IRQ | Nothing written. Find who enables it. |
| `S1b refused (no AP RTC write): ... moved since S1a` | Block state changed between S1a and S1b | Another owner; stop |
| `S1b arm ... readback wrong even with RTCEN` | Alarm registers write-protected (secure or gated) | Route dead → S1f |
| `S1b alarm #n INTP.ALM NOT set ... FAIL: INTP.ALM not set within the window` | Alarm logic does not fire (or counts differently) | Route dead → S1f |
| `... FAIL: INTP.ALM set outside the expected time` | Fires, but not on the seconds match | Investigate (time base) before S1c |
| `... FAIL: RTCALM not disarmed` / `INTP not cleared` | Cannot disarm or clear: the alarm would repeat every minute | Route dead. A stuck INTP with the SPI disabled is harmless until the 120 s restart. |
| `S1b restore: ... NOT RESTORED` | A register this stage wrote is not back at its value | Recorded; FAIL. Compare the before/after lines. |
| `S0b ... FAIL` / `CPU-LOST` | S0 regression (not S1) | As in suspend-s0.md |
| No `suspend s1:` line at all, `probe FAIL: bad DT properties` | DT mismatch | Build problem |

Any FAIL leaves the phone to the normal 120 s restart (no petting change), except an abort (panic) or a stall (deadline).

## 7. Unused in S1a/S1b: early watchdog re-arm (commit 4b70afda9)

For S1c..S1e (design §4.1 and change C4). In real SLEEP the CL0 watchdog is unpowered. If PERIS is reset, it comes back at its reset values `WTCON 0x8021`, `WTDAT 0x8000` (enabled, ~2.6 s), and 6.12 re-arms it only in device resume.

- `s3c2410wdt_crownlte_resume_early()` (`include/soc/samsung/crownlte-wdt.h`):
  - records WTCON/WTDAT/WTCNT and PMU `0x408`/`0x40c` as found;
  - then, in the 4.9 syscore order: `0x408` bit 24 off, `WTDAT = WTCNT =` kept-armed count, `WTCON =` the kept-armed WTCON (saved at kept-armed suspend), `0x40c` bit 24 off.
  - It is a reload and obeys the crownlte rules: after `stop_petting` or the deadline, a counter found running with the reset enabled is left alone (`-ETIME`); one found stopped is started with the full count, so the due reset still happens.
  - Only after a kept-armed suspend (`-ENOENT` otherwise, nothing written). Interrupts off, one CPU.
- `exynos9810-pm`: `e9810_s1_post_return()` (store only) and `e9810_s1_post_return_print()`, both `__maybe_unused`. S1c must call the first right after `cpu_suspend()` returns, before syscore resume, and the second after syscore resume. In this build the compiler drops both; `exynos9810-pm.o` has no reference to the watchdog function.

## 8. Deviations from the design, and why

1. **S0 stages kept as `"a", "b"` instead of `/delete-property/ samsung,suspend-s0`.** The design wants S0a/S0b as the regression base inside S1a, and the probe needs S0 stages for the deadline/topology path. Leaving out c/d means no suspend or syscore ops are registered, so nothing of the sleep path can run.
2. **Order S0a → S1a → S1b → S0b.** The new reads come first, so an abort is not preceded by 11 s of hotplug.
3. **32-bit accesses.** 4.9/upstream use `readb`/`writew` on this layout. The block is on APB, where narrow writes have no byte strobes; all defined fields are ≤ 16 bits and the upper bits are reserved.
4. **Advance 2 or 3 s** (design: "2"): two reads 2100 ms apart can see either count of whole-second steps.
5. **Alarm window** `[(d-1) s - 100 ms, d s + 999 ms]` (design "2.0..3.99" for +3). The 100 ms margin covers a seconds tick between reading BCDSEC and enabling RTCALM.
6. **Extra S1b guard:** S1b refuses if SPI 31 is enabled at the GIC (read-only check). With no handler, a fired alarm would be an unhandled level IRQ.
7. **`GICD SPI31 pending` and `WAKEUP_STAT` are logged, not gated.** A secure-group SPI reads 0 from the non-secure side, and WAKEUP_STAT may only latch in SLEEP.
8. **Restore.** Beyond the disarm (`RTCALM = 0`, `INTP = 0x2`), S1b writes `ALMSEC` back to its value from before the stage, and also `RTCALM` if that was not 0. Every register is printed before → after.
9. **PMIC:** the design lists `0x1E 0x21 0x24 0x27 0xC8 0xC9`. The first four are already in the existing buck dump (`0x1c..0x48`); only `0xC8..0xC9` is new, behind a DT flag. In K49, `PWREN_SEL1 = 0xff` is commented out and only `PWREN_SEL2 = 0xff` is written (`drivers/regulator/s2mps18.c:1282-1283`).

## 9. Register footprint (new in S1a/S1b)

| Register | Access | 4.9 precedent |
|---|---|---|
| AP RTC `0x141E0030..0x141E0090` (INTP, RTCCON, TICCNT, RTCALM, ALMSEC..ALMYEAR, BCDSEC..BCDYEAR, CURTICCNT) | read (S1a, S1b) | **none**: 4.9 never maps it (this is what S1a proves) |
| AP RTC `INTP` (`0x2` W1C), `ALMSEC`, `RTCALM` (`0x41`, `0`, original) | write (S1b) | none |
| AP RTC `RTCCON` (`|= RTCEN`, then original) | write, only if a write does not read back | upstream/4.9 `s3c24xx_rtc_enable` |
| CMU_APM `0x14003028` QCH_CON_APBIF_RTC | read | CAL `save_sleep` list (S0c read it on hardware) |
| PMU `0x404` RST_STAT, `0x408`, `0x40c` | read | watchdog driver (already read on 6.12) |
| PMU `0x600`, `0x640` WAKEUP_STAT/2 | read | S0 (hardware) |
| GICD `0x104` ISENABLER1, `0x204` ISPENDR1 | read | 4.9 GIC save path / S0 |
| PMIC `0xC8..0xC9` | read over ACPM | 4.9 writes `0xC9` |

## 10. Risks (for the reviewer)

- **First non-secure access to the AP RTC.** A secure-only block aborts → panic → reboot → bounce (proven path); a stall → deadline → WDT reset (proven). Both recover.
- **APB stall with no abort, and petting continues on other CPUs until the 145 s deadline.** The CPU0-pinned PID1 restart cannot run then, so the reset comes at the deadline plus the run-time watchdog timeout. Still self-recovering.
- **S1b firing a PMU wakeup event while awake.** The PMU/APM is not in a low-power mode (no SICD on 6.12, no C2 armed), so no sequencer should act on it. Unproven; the WAKEUP_STAT lines show whether anything latched.
- **An edge-latched SPI 31 pending bit left in the GIC** (if the line is edge-triggered). Harmless while the SPI is disabled. S1c must clear it before enabling.
- **RTCEN toggle.** If `RTCEN` gates writes, S1b sets and restores it around each write. It is the only RTCCON write, and the restore is checked.
