# crownlte 6.12: system suspend stage S0

Stage S0 of the suspend design (gekko `roadmap/suspend-design.md` §5):
everything a suspend-to-RAM does, up to but never including the firmware
sleep call. Four self-recovering self-tests, S0a to S0d, on the delayed-UFS
RECOVERY vehicle. Log prefix: `suspend s0:`.

Build-only so far. Nothing in this document has run on the phone yet.

## 1. What S0 can and cannot do

S0 runs the kernel's real suspend path: freezer, device suspend, noirq,
CPU hotplug, syscore and the platform `.enter`. It stops before the SoC
could go to sleep. That is guaranteed by construction, not by timing:

1. **No CPU_SUSPEND call exists.** `.enter` runs `cpu_suspend()` with a
   dry finisher. The finisher records what 4.9 would have passed to PSCI
   (fn `0xc4000001`, power_state `0x03000000`, entry `__pa(cpu_resume)`)
   and returns without an SMC. `cpu_suspend()` reports that as
   `-EOPNOTSUPP`, which is the not-slept path. The built
   `exynos9810-pm.o` has no `smc`/`hvc` instruction and no reference to
   PSCI or to `cal_pm_enter()`; it calls only `cal_pm_enter_unarmed()`.
2. **The APM hand-off is withheld.** Syscore suspend runs
   `cal_pm_enter_unarmed(8)`: the 4.9 SYS_SLEEP CPU_INFORM hint, register
   save and enter sequence, minus the INTR_GEN `GRP2_INTR_BID_ENABLE`
   write (`0x14070200` bit 4, CPU0). That bit is how a CPU0 "core down"
   event reaches the APM firmware, which runs the SoC and MIF sleep
   sequencers. The driver reads it back as 0, or undoes the step and
   fails it.
3. **No PMIC, charger or UFS command.** This DT has no regulator driver,
   no PMIC write permission (no `samsung,rtc-write-enable` or
   `rtc-read-latch`), no charger watchdog kick, and UFS stays unbound.
   S0c/S0d refuse to start if a UFS host is bound, a charger kick is
   enabled, a CPU was lost, or the CL0 watchdog is not kept armed.
   The only PMIC traffic is the APM's own DVFS voltage change for the
   cpufreq/devfreq suspend frequencies, the same as any runtime frequency
   change.

As a result, S0d cannot prove the real firmware early-wake path (APM
`EarlyRet`). It proves everything up to that SMC: the parameters, the
pending wake IPI, the `cal_pm_earlywakeup()` undo, and the counters
running. The APM log ring shows no SLEEP or EarlyRet events in S0, and
none are expected.

## 2. Code

| Commit | What |
|---|---|
| `soc: samsung: cal-if: system power mode entry without the APM hand-off` | `pmucal_system_enter_unarmed()` / `cal_pm_enter_unarmed()`. Refuses (writes nothing) if the enter table has no GRP2 arming write. |
| `cpu/hotplug: add a STARTING state for the Exynos 9810 core power hooks` | `CPUHP_AP_EXYNOS9810_CPU_POWER_STARTING`, right after `CPUHP_AP_OFFLINE`: the 4.9 `CPUHP_AP_EXYNOS_CPU_DOWN_POWER_CONTROL` slot. |
| `PM: suspend: add pm_suspend_test_level() for in-kernel self-tests` | pm_test from a kernel thread. `CONFIG_PM_DEBUG` only. |
| `watchdog: s3c2410_wdt: crownlte keep-armed suspend and suspend-driver hooks` | DT `samsung,keep-armed-in-suspend`; `include/soc/samsung/crownlte-wdt.h` (ping, state, stop_petting). Guarded crownlte instance only. |
| `soc: samsung: exynos9810-pm: system suspend stage S0` | `drivers/soc/samsung/exynos9810-pm.c`, `CONFIG_EXYNOS9810_PM`. |
| `arm64: dts: exynos9810: crownlte suspend S0 diagnostic DTBs` | Disabled `system-suspend` node in `exynos9810.dtsi`; the S0 and S0ab DTBs. |

### Why the generic files change

- **cpuhotplug.h:** the core power hooks must run on the dying CPU with
  interrupts off, after `sched_cpu_dying()` and before the idle loop's
  PSCI CPU_OFF. Only static STARTING states do that; there is no dynamic
  STARTING range. One enum entry, unused unless the DT node is enabled.
- **suspend.c / suspend.h:** the vehicle's PID1 is fixed and never
  touches `/sys/power`, and `pm_test_level` is private to
  `kernel/power`. The helper takes the same lock as the sysfs store,
  restores both values afterwards, and refuses `none`. Known limit,
  stated in its kernel-doc: a concurrent `/sys/power/pm_test` write
  between unlock and `pm_suspend()` wins. In S0 that cannot reach
  firmware sleep (§1).
- **s3c2410_wdt.c:** upstream stops the watchdog in the device phase.
  That leaves noirq, syscore and `.enter` unguarded, and those are where
  a suspend hangs. The stop-petting hook exists because of trial
  c2diag-b: a core lost while counted online kept the system "alive",
  the other CPUs kept petting, and the phone needed a manual reset.

### CONFIG

Base: `panel-bright-work/images/config-romram-22e97b5d2` (the
integration-4 / TEE-2 config), plus:

```
CONFIG_PM_DEBUG=y          # pm_test; PM_SLEEP_DEBUG follows
CONFIG_EXYNOS9810_PM=y
```

`CONFIG_PM_TEST_SUSPEND` is not used: it is the RTC-alarm test and would
really suspend.

### DT (`samsung,exynos9810-pm`)

| Property | S0 DTB | Meaning |
|---|---|---|
| `samsung,suspend-s0` | `"a","b","c","d"` (`"a","b"` in s0ab) | Stages to run; S0a always runs. Without it, probe fails: this build is S0 only. |
| `samsung,suspend-s0-start-ms` | 20000 | First stage at this boot time. |
| `samsung,suspend-s0-end-by-ms` | 100000 | A stage that would not finish by then is SKIPPED. |
| `samsung,suspend-s0-deadline-ms` | 145000 | Still up at this time: stop watchdog petting (forced ≥ end-by + 10 s). |
| `samsung,suspend-s0-hotplug-cycles` | 20 | S0b single-core cycles per core (1..100). |
| `samsung,suspend-s0-cluster-cycles` | 5 | S0b BIG-cluster cycles (0..20). |
| `samsung,suspend-s0-group-cycles` | 5 | S0b cycles per LITTLE pair, LITTLE triple and all-nonboot group (0..20). |
| `samsung,suspend-s0-pm-test-delay-s` | 5 | pm_test wait per level (1..5). |
| `samsung,sleep-with-cp-off` | set | CP_STAT 0 does not cancel (4.9 refuses; the CP never boots here). |
| `samsung,sleep-mode-idx`, `samsung,sleep-psci-state`, `samsung,wakeup-mask-offsets`, `samsung,wakeup-masks` | dtsi | 4.9 values: 8, `0x03000000`, `0x610 0x614 0x618 0x644` = `0xd00f7e7e 0xffff00ff 0xffff00ff 0`. |

The watchdog node gets `samsung,keep-armed-in-suspend`. Mali is
disabled: kbase vetoes suspend on `PM_SUSPEND_PREPARE`. Every other
self-test is removed, except the read-only PMIC snapshot at ~106 s. It
dumps INTxM, the rail CTRL registers and the RTC registers (design R9),
after S0 has ended.

## 3. Artifacts

| Item | Path (gekko `/mnt/ssd/host-builds/kernel-upgrade/`) | sha256 |
|---|---|---|
| Image (code at `4877cf53b`) | `panel-bright-work/images/Image-suspend-s0-4877cf53b` | `1fa72477800c00ab4aa0f8d6a4e241ee2cdb4bac3745776606f484884c253763` |
| config | `panel-bright-work/images/config-suspend-s0-4877cf53b` | `1ec1bef6e867e455921af6dd6f57ace26a543d9257bf56ff71fde350b6e4fa6a` |
| S0 DTB (a..d) | `panel-bright-work/dtb/exynos9810-crownlte-delayed-ufs-suspend-s0-diag-4877cf53b.dtb` | `61e670ec5078b8e9f8c1a07b520beb102a7476c545e69cd9de1165c4c7259050` |
| S0ab DTB (a+b) | `panel-bright-work/dtb/exynos9810-crownlte-delayed-ufs-suspend-s0ab-diag-4877cf53b.dtb` | `362e56d366f53a87394f24969452c254991362824819b9ae358fa18c69a8b4de` |
| Trial 1: S0ab RECOVERY | `acpm-suspend-s0ab-s2/recovery.img` | `7b48380bfda1fdfe07ef3d6a1a0abc3d226e4086f9de39afb7e4d5a1d0234d01` |
| Trial 2: S0 RECOVERY | `acpm-suspend-s0-s2/recovery.img` | `f787654d07c81caf8539fd86d6ff925bc7916bad25af6068b13208f3adcf2bde` |

Both packs: `pack-acpm-q-paths.py`, shim `bbea959`, ramdisk `506232b1`
(CPU0-pinned PID1, restart at 120 s, fallback at 150 s), parent
`a8198be0`. `manifest.json` sits next to each image. Its
`kernel_source` field is the packer's stale default; `PROVENANCE.txt`
has the real source.

## 4. Running it

The order is S0ab first, then S0. A LITTLE hotplug loss in S0ab ends the
series: S0c/S0d need all CPUs.

Same procedure as the other delayed-UFS trials: copy the work dir to the
PC, add `ready-capture.ps1`, then `run-trial.sh <dir>` (default capture
180 s). Start it only with `adb reboot recovery`. The host sends
`CROWNLTE-UFS-READY-1` once. **Never send `CROWNLTE-UFS-PROBE-1`**: a
bound UFS host makes S0c/S0d refuse, and the UFS guard is not extended
for suspend.

Timeline:

| Boot time | What |
|---|---|
| ~2 s | `suspend s0: probe: stages abcd ...`; `s3c2410-wdt ... system suspend keeps it armed (timeout 60 s while suspended)` |
| 20 s | deadline armed; S0a |
| ~20-35 s | S0b (170 hotplug cycles) |
| ~35-75 s | S0c (5 levels × 5 s + overhead). **COM4 drops** at the `devices` level (dwc3 suspends) and comes back on resume. `ready-capture.ps1` reopens the port, and PID1 resends its cached log. |
| ~75-80 s | S0d (COM4 drops again) |
| then | `SUMMARY` four times, 5 s apart |
| ~106 s | `pmic selftest:` (read-only) |
| 120 s | PID1 restart (150 s fallback); shim → Lineage |
| 145 s | only if still up: watchdog petting stops, WDT reset within 30 s |

After return, `run-trial.sh` saves `/sys/fs/pstore/pmsg-ramoops-0`
(the 6.12 console tail; this DT's ramoops console is 16 KB, so only the
last ~150 lines) and restores bright RECOVERY `8763860c`.

## 5. Expected PASS lines

`<...>` marks values that are not known yet. Kernel lines without the
prefix are shown where they matter.

### S0a

```
suspend s0: S0a CP_STAT=<cp> CP_CTRL_NS=<..> CP_CTRL_S=<..> SEQ_CP cfg=<..> status=<..> DFD=<..> G3D=0xf
suspend s0: S0a INFORM0..7=0 0 0 0 0 0 0 0 FLEXPMU_INFORM=<..>
suspend s0: S0a WAKEUP_MASK 0x60c=<..> 0x610=<..> 0x614=<..> 0x618=<..> 0x644=<..> WAKEUP_STAT=<..>/<..>
suspend s0: S0a PAD_RET top=<..> ... IDLE_IP=... IDLE_IP_MASK=...
suspend s0: S0a INTR_GEN en/pend grp0=... grp4=... EVENT_INT_EN_GRP2=<..>
suspend s0: S0a flexpmu sw_flag=<..> ap_sleep=<n>/<n> mif_sleep=<n>/<n>
suspend s0: S0a apm-ring rear=<..> front=<..> ... newest 8:
suspend s0: S0a PASS (read-only) CP_STAT=<cp>
```

`G3D=0xf` matters for S0c/S0d: the SYS_SLEEP save list reads CMU_G3D
unconditionally, and syscore suspend refuses if G3D is off. The S0a
values are the 6.12 reference for design §7 (CP_STAT with the CP never
booted, and the reset values of 0x60c/0x610/0x614/0x618/0x644).

### S0b

```
suspend s0: S0b start: 20 x cpu7..4, 5 x BIG cluster, 20 x cpu3..1, 5 x LITTLE pairs {1,2} {1,3} {2,3}, 5 x LITTLE {1,2,3}, 5 x cpu1..7 (suspend order)
suspend s0: S0b big cpu7 #1 off: inform=0x1 status=<off> off_us=<..> on: up_ms=<..> ping_us=<..> stale_grp2=0 ok
...
suspend s0: S0b cluster #1 BIG off (NONCPU 0x2044=<..> 0x204c=<..> APM2AP=<..>) and on again (APM2AP=<..>) ok
suspend s0: S0b little cpu3 #1 off: inform=0x1 status=<off> ... ok
suspend s0: S0b little-2 #1 2 cores off (status/inform: cpu1=<off>/0x1 cpu2=<off>/0x1) and on in <..> ms ok
suspend s0: S0b little-3 #1 3 cores off (status/inform: cpu1=... cpu2=... cpu3=...) and on in <..> ms ok
suspend s0: S0b nonboot #1 7 cores off (status/inform: cpu1=... cpu7=...) and on in <..> ms ok
suspend s0: S0b cpu<n> hooks down=<..> up=<..> cluster off/on=<..> stale_grp2=<..> dead=<..> still_on=0 last cal 0/0
suspend s0: S0b PASS single 140/140 cluster 5/5 group 25/25 in <..> ms, 0 CPU_DEAD polls saw the core still powered
```

`inform=0x1` is the 4.9 hotplug hint: `cal_cpu_disable()` writes the C2
mode, and hotplug differs from C2 only in EL3 CPU_OFF and in no wake. "Off" means STATUS
bits 3:0 = 0 (the q40/q43 dumps read `0x880000` for an off core). A `!`
after a core in a group line means its CPU_DEAD poll timed out.

### S0c

Per level, with kernel lines in between (`PM: suspend entry (deep)`,
`PM: suspend debug: Waiting for 5 second(s).`, `PM: suspend exit`):

```
suspend s0: S0c start: pm_test freezer .. core, 5 s each
suspend s0: S0c level=freezer ret=0 <ms> ms begin/prepare_late/enter +0/+0/+0 syscore +0/+0 cpus 8/8 wdt 30s en=1 -> PASS
suspend s0: S0c level=devices ret=0 <ms> ms begin/prepare_late/enter +1/+0/+0 syscore +0/+0 cpus 8/8 wdt 30s en=1 -> PASS
suspend s0: S0c level=platform ret=0 <ms> ms begin/prepare_late/enter +1/+1/+0 syscore +0/+0 cpus 8/8 wdt 30s en=1 -> PASS
suspend s0: S0c level=processors ret=0 <ms> ms begin/prepare_late/enter +1/+1/+0 syscore +0/+0 cpus 8/8 wdt 30s en=1 -> PASS
suspend s0: step 20 syscore suspend mode 8 masks 0x60c=<..>(not written in S0) 0x610=0xd00f7e7e 0x614=0xffff00ff 0x618=0xffff00ff 0x644=0x0 IDLE_IP_MASK=0xffffffff CP_STAT=<cp> (CP off, allowed by DT)
suspend s0: step 20 cal_pm_enter_unarmed(8) ok: INFORM0=0x4 EVENT_INT_EN_GRP2=<bit4 set> GRP2 en=<bit4 clear> (cpu0 bit 0, APM hand-off withheld: 1 write skipped)
suspend s0: step 50 syscore resume exit ok (ret 0/0): INFORM0=0x0 GRP2 en=<..> EVENT_INT_EN_GRP2=<bit4 clear> cpu0 GRP2 latched in window=0 PAD_RET top=1 spi=1 bootldo0=1 bootldo1=1 pcie=1 uart=1 WAKEUP_STAT=<..>
suspend s0: S0c level=core ret=0 <ms> ms begin/prepare_late/enter +1/+1/+0 syscore +1/+1 no-cpu0-WFI cpus 8/8 wdt 30s en=1 -> PASS
suspend s0: S0c PASS 5/5
```

Also expected: `s3c2410-wdt ... suspend: kept armed, timeout 60 s` and
`resume: armed, timeout 30 s` at the levels from `devices` up. The
`core` level runs `cal_pm_exit(8)` as 4.9 did under pm_test. That
includes the 4.9 DRCG enables, the PAD_RETENTION release (bit 0 = 1) and
the restore of the ~1100 saved CMU registers, all to the values saved a
few seconds earlier.

### S0d

```
suspend s0: S0d start: full suspend-to-RAM, .enter = dry run with a pending self-IPI (no SMC)
suspend s0: step 0 begin state=3 (mode 8, psci_state 0x3000000)
suspend s0: step 5 prepare_late (noirq): 8 CPUs online
suspend s0: step 20 syscore suspend mode 8 masks ... (as S0c)
suspend s0: step 20 cal_pm_enter_unarmed(8) ok: INFORM0=0x4 ... 1 write skipped)
suspend s0: step 30 enter (S0 dry run, no SMC): fn=0xc4000001 state=0x3000000 entry=<__pa(cpu_resume)> cpu0 self-IPI pending=1 (ISPENDR0=<bit1> SPENDSGIR0=<..>) wdt=60s en=1 rst=1 cntpct=<a> frc=<b>
suspend s0: step 40 CPU_SUSPEND returned -95 (early) dry calls=1 fn=0xc4000001 state=0x3000000 entry=<..> cntpct <a>-><a'> frc <b>-><b'> (frc running) mif_sleep <m>-><m> ap_sleep <s>-><s> (MIF blocked: no firmware call)
suspend s0: step 50 syscore resume earlywakeup ok (ret 0/0): INFORM0=0x0 ... cpu0 GRP2 latched in window=0 ...
suspend s0: step 50 early wakeup: EINT_PEND gpa0..4=<..>
suspend s0: step 50 early wakeup: GICD_ISPENDR0=<bit 1: the self-IPI, still pending with IRQs off>
suspend s0: step 60 wake: 8 CPUs online
suspend s0: step 70 end
suspend s0: S0d pm_suspend=-95 in <ms> ms: enter=-95 (early) ipi_pending=1 fn=0xc4000001 state=0x3000000 entry=<..> earlywakeup seq ok cpu0-WFI-in-window=0 frc <b>-><b'> cpus 8/8 -> PASS
```

`pm_suspend=-95` (`-EOPNOTSUPP`) is the PASS value. The kernel also logs
the suspend as failed (`PM: suspend exit`, the failure counted in
`suspend_stats`); that is expected. The FlexPMU sleep counters do not
move in S0.

### Final

```
suspend s0: SUMMARY S0a=PASS [CP_STAT=<cp> INFORM0=0x0 0x610=<..> mapped all] S0b=PASS [single 140/140 cluster 5/5 group 25/25 still-on 0] S0c=PASS [5/5 levels] S0d=PASS [early return -95, earlywakeup ok] lost=
```

With the S0ab DTB: `S0c=not-run [] S0d=not-run [] lost=`.

## 6. Failure signatures

| Signature | Meaning | What happens next |
|---|---|---|
| `suspend s0: probe FAIL: <why>` and, at ~22 s, `SUMMARY ... probe failed: <why>` | The driver did not start (bad DT, CAL, topology, or the DFD dump is enabled so `big_reset_control()` would be needed). Nothing ran. | Normal 120 s restart. |
| `CPU-LOST cpuN: PSCI CPU_ON bring-up failed` with `LOST cpuN phyN status=... inform=... grp1 en/pend=... grp2 en/pend=...` and an `apm-ring` dump. Kernel: `CPUN: failed to come online`. | CPU_OFF worked but CPU_ON did not bring the core back. This is the hotplug form of the EL3 LITTLE hang. The kernel counts it offline, so nothing waits on it. | S0b FAIL, S0c/S0d SKIPPED, `CPU-LOST N: offline; the normal 120 s restart applies`. |
| `CPU-LOST cpuN: online but not answering IPIs` / `bring-up never completed` / `tear-down never completed`, then `s3c2410-wdt: petting stopped for good (...): SoC reset in about 30 s` | The core is counted online but does not run, or a hotplug call is stuck holding the hotplug locks, which the restart path needs. | WDT reset ≤ 30 s → sboot RECOVERY → shim bounce → Lineage. The pstore tail ends with these lines. |
| `cpuN still powered <ms> ms after CPU_DEAD ... (4.9 would BUG here)`; S0b `still-on N` > 0 | PSCI CPU_OFF returned, but the PMU still reports the core powered. Recorded, not fatal in S0. | S0b can still PASS, but it **blocks S1**: SYS_SLEEP needs all cores off. |
| `S0b ... hooks: ... cal=<err> FAIL` / `cluster_disable not run by cpu4` | A CAL core or cluster sequence failed, or a hook did not run. | S0b FAIL. |
| `S0b end: changed INFORM[n]: a -> b` (or `EVENT_INT_EN_GRP2`, `INTR_GEN GRP2 enable`, `APM2AP bit24`, `EINT mask gpa`) | The hotplug hooks left PMU state behind. | S0b FAIL (`regs not restored`). |
| `S0c refused: <why>` / `S0d refused: <why>` | A precondition failed (UFS bound, charger kick, CPU lost, watchdog not kept armed, ops not registered, no PM_DEBUG). | SKIPPED. Nothing was suspended. |
| `S0c level=<l> ... -> FAIL: pm_suspend returned an error`, with kernel `Freezing ... failed`, `PM: Some devices failed to suspend`, `PM: Wakeup pending, aborting suspend` / `active wakeup source: <name>` | A task or device refused, or a wakeup source is active. | S0c FAIL at that level; later levels and S0d do not run. |
| `step 20 refused: EMBEDDED_G3D_STATUS=<x> (not on)` | G3D is powered off; the CAL save would read a dead CMU. Refused before any write. | `core` level FAIL (`unexpected syscore callbacks`), S0d not run. Fix before S1: power G3D on, or make the save conditional. |
| `step 20 cal_pm_enter_unarmed(8) failed <err> ... undone` | The CAL refused (table without the arming write: `-ENOENT`) or a sequence failed. The step undid itself. | `core` level FAIL. |
| `step 50 syscore resume exit FAILED` / `earlywakeup FAILED` | The CAL exit or earlywakeup sequence returned an error. | S0c `core` / S0d FAIL. |
| `S0c ... cpu0-WFI-in-window` / `S0d ... cpu0-WFI-in-window=1` | CPU0's GRP2 event latched while the system was entered. The APM was not told (enable withheld). | Logged. Investigate before S1. |
| `S0d ... FAIL: self-IPI not seen pending before the call` | The 4.9 `test_early_wakeup` precondition does not hold on 6.12 (GIC-400 SGI1 pending bits not set). | A future firmware early-wake test cannot rely on a self-IPI. |
| `S0d ... FAIL: FRC/arch counter not counting` | MCT FRC or CNTPCT did not advance across `.enter`. | Must be understood before S1 (design R3). |
| `deadline 145000 ms reached and the system is still up` | The PID1 restart and its 150 s fallback did not happen by 145 s. | Petting stops → WDT reset ≤ 30 s → shim bounce. |
| Log stops with no FAIL line | Hang. The last `suspend s0:` / `step NN` line in pstore names the place. From the devices level up, the watchdog is kept armed at 60 s. | WDT reset → shim bounce → Lineage. |
| Serial stops at S0c `devices` and never resumes | dwc3 did not come back after resume. | Read pstore after return. The 120 s restart still runs. |

## 7. Recovery path

1. **Normal.** PID1 reboots at 120 s (CPU0-pinned timer child; parent
   fallback at 150 s). sboot boots RECOVERY again, the shim (`bbea959`)
   sees a SWRESET-free start and bounces to Lineage. `run-trial.sh`
   waits for Lineage, saves pstore and the reset reason, and restores
   bright RECOVERY `8763860c`.
2. **Kernel hang with the watchdog running.** The CL0 watchdog resets
   the SoC: 30 s at run time, 60 s from device suspend to resume. Then
   the same shim bounce (proven in q35/q37/q40).
3. **Core lost while counted online, or still up at 145 s.** The driver
   stops the petting, and path 2 follows.
4. **Phone does not return** (`run-trial.sh`: `PHONE DID NOT RETURN`):
   owner force restart, Power + Vol Down ~7-10 s. That is a cold PMIC
   cycle; the shim bounces it to Lineage (BOOT is untouched). Then
   restore bright RECOVERY `8763860c` by hand and wait for AshLooper
   `boot=booted` before the next reboot. S0 has no path that leaves the
   SoC asleep (§1), so the "slept and never woke" case (design R1) does
   not apply to S0.

## 8. Before S1

S0 PASS is necessary for S1, not sufficient. Still missing:

- the real finisher (`psci_ops.cpu_suspend(0x03000000, __pa_symbol(cpu_resume))`);
- the GRP2 arming (`cal_pm_enter`);
- the MCT FRC restart;
- pinctrl retention and the 0x60C EINT mask;
- the PMIC IRQ / RTCA0 (owner decision D1);
- DECON stop;
- TMU suspend IPC.

S1 needs the owner present (design §5, D5).
