crownlte-integration-2
======================

Branch crownlte-integration-2 = crownlte-integration (d52ab57b6) plus every
feature branch that passed on the phone by 2026-11-04, merged with
`git merge --no-ff` so that each feature stays one merge commit. Combined
trial: q45-combined (DTB exynos9810-crownlte-delayed-ufs-combined-diag).

What is in
----------

Already in crownlte-integration: ACPM IPC/DVFS, cpufreq (big capped at
1794 MHz), MIF/INT devfreq, ACPM TMU + thermal zones, S2MPS18 PMIC/RTC
(read-only, sealed write policy), CL0 watchdog with the restart guard,
display stage 1 (simplefb on the SBoot buffer).

| Merge | Branch (head) | Content | Trial |
|---|---|---|---|
| 1 | crownlte-panel-bright 0282e1596 (on crownlte-display-s2) | DECON-F adopt-mode fbdev (fb0, TE vsync, WIN_CONFIG, A/B flip self-test), IDMA/DPP bank fix, DSIM0 polled command path, 4.9 fault-display brightness 44700 | q39 PASS (20/20 flips), q42 PASS (42 packets, read-back MATCH) |
| 2 | crownlte-next c8128e3d5 (on crownlte-mali) | Mali kbase r38p1 + IRQ-by-name, DT-gated G3D power domain ("boot-on"/"runtime") with the exact 4.9 G3D rail request, self-test runtime-PM fix, cooling names thermal-cpufreq-0/1 | q27b, q30b, q31c PASS |
| 3 | crownlte-adc f629c9cbe (on crownlte-battery) | CMGP SoC ADC, MAX77705 read-only battery/ac/usb/wireless supplies with ADC thermistors | q34 PASS |
| 4 | crownlte-integration-idle-c 54f243e93 | MCT G0 tick broadcast, C2 cpuidle with per-CPU hook reset, BIG-only C2 (LITTLE refused unless samsung,c2-little-enable) | q21b, q44 PASS |
| 5 | crownlte-tee 4e81ce065 | Trustonic driver, start in a kthread; node disabled | q29 PASS (kernel start only) |
| 6 | crownlte-touch dc29860e5 | sec_ts Y771 driver + USI03 HSI2C nodes; disabled | q32: IC unpowered |

Commits on top of the merges:
- exynos_adc: the self-test samples its counters under the conversion lock.
  The q34 "FAIL" was not the unwired channels (they never failed the test)
  but the 9-writes-per-conversion check: the battery monitor converted at
  the same time and the first snapshot fell mid-conversion (804 writes for
  90 conversions). Unwired (all-rail) channels are now labelled as such.
- cpuidle-exynos9810: DT samsung,cpuidle-selftest-soak-ms (1..10 s,
  default 10 s as in q44).
- defconfig/fragment: MFD_S2MPS18_ACPM, RTC_DRV_S2MPS18_ACPM, CPU_IDLE,
  ARM_EXYNOS9810_CPUIDLE, EXYNOS9810_MCT_BROADCAST (previously only set in
  trial build directories). NO_HZ_IDLE was already the defconfig tick mode.
- the combined DTB and this file.

What is off, and why
--------------------

- Trustonic TEE: q29 only verified the kernel-side start. The enable moved
  out of the base diag DTS into exynos9810-crownlte-delayed-ufs-tee-diag;
  no other DTB starts the secure world. TEE hard rules:
  Documentation/crownlte/trustonic-tee.md.
- Touchscreen: q32 showed LDO35/LDO43 off (bootloader leaves the IC
  unpowered); needs a narrow PMIC LDO-enable policy first. The enable moved
  into exynos9810-crownlte-delayed-ufs-touch-diag.
- DPU SysMMU (crownlte-dpu-iommu): not merged; it needs a rebase on the
  display stage 2 DPP/IDMA fix (c3b589012) first. sysmmu_dpu0..2 stay
  disabled.
- LITTLE C2: cores were lost in EL3 in q22b/q40/q43.
- Watchdog expiry self-tests (crownlte-watchdog selftest DTBs): not merged;
  the watchdog is armed and pinged normally, the restart guard is in.
- Brightness stays opt-in: only DTBs with samsung,fault-display-brightness
  (display-s2-bright, combined) map DSIM0 and send panel commands. The
  owner has not yet confirmed the q42 brightness visually.

Merge conflicts
---------------

All conflicts were DT files where two branches added a block at the same
place; no C conflicts. Both sides were kept. Where git aligned the two new
blocks on shared lines (`/*`, `status = "disabled"; };`), our side was
taken and the incoming branch's own diff re-applied with patch(1), so both
blocks stay whole (reapply-theirs.sh on gekko):
- exynos9810.dtsi: sysmmu_dpu0..2, gpu@17500000 and adc@14230000 all
  added before pinctrl_aud.
- exynos9810-crownlte-delayed-ufs-diag.dts: Mali, battery/ADC enables
  appended at the end; the TEE and touch enables appended at the same
  place went into their own DTBs instead (see above).
- exynos9810-crownlte-clocks.dtsi: TEE node and USI03/HSI2C/touch nodes.
The watchdog restart guard came in three times (d52ab57b6 and the
identical cherry-picks 2dddf197d, d3e358457) and merged cleanly once.

q45: the combined DTB
---------------------

exynos9810-crownlte-delayed-ufs-combined-diag.dts includes the
display-s2-bright DTB (and through it the stage 2 and base diag DTBs) and
adds the RTC read latch, the G3D rail request + "runtime" G3D power
control, a 3 s cpuidle soak and the brightness at 99 s. TEE, touch,
SysMMU and the stage 1 helper are forced off.

Self-test schedule (scheduled uptime; printk time runs ~1-1.5 s later):

| s | Self-test | PASS looks like |
|---|---|---|
| 86 | MCT broadcast | `exynos9810-mct-bc: report: ready=1 why=ok ... test_fired=3/3` |
| 90 | cpuidle (LITTLE refusal, big-1..4, 3 s soak; done ~98.5 s) | `cpuidle selftest: LITTLE arm check: refused`, `cpuidle selftest: done result=PASS phases_passed=5/5 ... lost=` |
| 99 | brightness 44700 | `panel-bright: read-back MATCH: ...`, `panel-bright: RESULT PASS: 0, 42 packets sent, ... 0 DSIM errors` |
| 100 | cpufreq | every `cpufreq selftest: policyN request=... PASS`, then `cpufreq selftest: done` |
| 104 / 104.5 | ADC raw / thermistors | `adc selftest: ... -> PASS`; `adc selftest: thermistors wired=3 ok=3 ... -> PASS` |
| 105 | devfreq + TMU | `devfreq selftest: PASS (0 mismatches, ...)`, `acpm-tmu selftest: done sensors=4` |
| 106 | PMIC + RTC latch | `pmic selftest: rtc advance 1 s over 1 s: PASS`, `pmic selftest: done errors=0` |
| 107 | battery | `battery selftest: ... writes=0 ... -> PASS` |
| 108 | DECON flips (~10 s) | `decon selftest: PASS: 20/20 flips latched ... latch timeouts 0` |
| 109 | Mali (runtime G3D) | `mali selftest: pd cycle: ...` lines, `mali selftest: PASS (0 failures, ...)` |
| 120 | restart | `s3c2410-wdt ...: restart: armed for 30 s and no longer petted`, `reboot: Restarting system` |

The cpuidle soak is shortened and the brightness moved from 96 s to 99 s
so that C2 arming and the DSIM command window never overlap; every other
time is unchanged. Known overlaps, each already seen in earlier trials:
ADC raw with the battery thermistor test (the fixed counter race), devfreq
with the TMU self-test, Mali with the DECON flips (q31c had Mali next to
the stage 1 display self-test).

How to read q45:
1. Serial (COM4 after READY) and, back in Lineage, the tail of
   /sys/fs/pstore/pmsg-ramoops-0 (the 6.12 console zone) must show every
   line of the table and no `FAIL`, `CPU-LOST`, `lost=<cpus>`, `deep idle
   auto-disabled`, `latch timeout` or DSIM error.
2. The owner should see the screen become readable at ~100 s, then the
   green 'A' / magenta 'B' flips from ~109 s to ~119 s.
3. Normal end: the restart lines at ~120 s and Lineage by itself. A hang
   ends in a watchdog reset (~30 s later) and the shim v2 bounce to
   Lineage; then only the pmsg tail tells where it stopped.
4. If a single feature fails, rerun its own DTB (q39/q42 display, q31c
   Mali, q34 battery, q44 cpuidle, q24 RTC) on this Image before blaming
   the combination.
