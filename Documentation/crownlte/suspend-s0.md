# crownlte 6.12: system suspend stage S0

Stage S0 of the suspend design (gekko `roadmap/suspend-design.md` §5):
everything a suspend-to-RAM does, up to but never including the firmware
sleep call. Four self-recovering self-tests, S0a to S0d, on the delayed-UFS
RECOVERY vehicle. Log prefix: `suspend s0:`.

Build-only so far. The first hardware run (S0ab pack `7b48380b`,
2026-11-06 23:46) panicked in probe on the secure-only PMU register 0x30;
fix round 1 (§10) removes that access and the other unproven ones (§9),
and fixes the Sol R1 review findings. Fix round 1 has not run on the phone.

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
| `watchdog: s3c2410_wdt: crownlte deadline in the ping path, keep-armed suspend never stops it` (fix round 1) | `s3c2410wdt_crownlte_set_deadline()`; every counter reload checks the deadline; the kept-armed suspend fails instead of stopping (§10). |
| `PM: suspend: run pm_suspend_test_level() under one system_transition_mutex ownership` (fix round 1) | `enter_state_check()` / `enter_state_locked()` split (§10). |
| `soc: samsung: exynos9810-pm: S0 fix round 1: ...` (fix round 1) | CP_CTRL reads removed, register audit (§9), bounded cross-CPU calls, recovery gate (§10). |
| `watchdog: s3c2410_wdt: no stretched or late reload when crownlte petting stops` (fix round 1) | Keep-armed refusal puts the run-time timeout back; the restart notifier re-checks the deadline under the lock (§10). |

### Why the generic files change

- **cpuhotplug.h:** the core power hooks must run on the dying CPU with
  interrupts off, after `sched_cpu_dying()` and before the idle loop's
  PSCI CPU_OFF. Only static STARTING states do that; there is no dynamic
  STARTING range. One enum entry, unused unless the DT node is enabled.
- **suspend.c / suspend.h:** the vehicle's PID1 is fixed and never
  touches `/sys/power`, and `pm_test_level` is private to
  `kernel/power`. The helper refuses `none`. It takes
  `system_transition_mutex` once (trylock, like `pm_suspend()`), sets the
  level and delay, runs the same sequence as `pm_suspend(PM_SUSPEND_MEM)`
  through `enter_state_locked()`, restores both values and only then
  unlocks. The `/sys/power/pm_test` store and every system transition
  take the same mutex, so nothing interleaves. `enter_state()` and
  `pm_suspend()` behave as before.
- **s3c2410_wdt.c:** upstream stops the watchdog in the device phase.
  That leaves noirq, syscore and `.enter` unguarded, and those are where
  a suspend hangs. The stop-petting hook exists because of trial
  c2diag-b: a core lost while counted online kept the system "alive",
  the other CPUs kept petting, and the phone needed a manual reset. The
  deadline lives in the ping path for the same reason: no timer on any
  particular CPU has to fire for it to hold. Without a crownlte deadline
  (every DT but the S0 ones) the driver behaves as before.

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
| `samsung,suspend-s0-deadline-ms` | 145000 | Mandatory. Installed at probe in the watchdog ping path: after this boot time no CPU reloads the counter, so the WDT resets the SoC within 30 s (forced ≥ end-by + 10 s). Probe fails without it or without an armed, petted watchdog. |
| `samsung,suspend-s0-hotplug-cycles` | 20 | S0b single-core cycles per core (1..100). |
| `samsung,suspend-s0-cluster-cycles` | 5 | S0b BIG-cluster cycles (0..20). |
| `samsung,suspend-s0-group-cycles` | 5 | S0b cycles per LITTLE pair, LITTLE triple and all-nonboot group (0..20). |
| `samsung,suspend-s0-pm-test-delay-s` | 5 | pm_test wait per level (1..5). |
| `samsung,sleep-with-cp-off` | set | CP_STAT 0 does not cancel (4.9 refuses; the CP never boots here). |
| `samsung,sleep-mode-idx`, `samsung,sleep-psci-state`, `samsung,wakeup-mask-offsets`, `samsung,wakeup-masks` | dtsi | 4.9 values: 8, `0x03000000`, `0x610 0x614 0x618 0x644` = `0xd00f7e7e 0xffff00ff 0xffff00ff 0`. Offsets other than these four are a DT error (§9). |

The watchdog node gets `samsung,keep-armed-in-suspend`. Mali is
disabled: kbase vetoes suspend on `PM_SUSPEND_PREPARE`. Every other
self-test is removed, except the read-only PMIC snapshot at ~106 s. It
dumps INTxM, the rail CTRL registers and the RTC registers (design R9),
after S0 has ended.

## 3. Artifacts

| Item | Path (gekko `/mnt/ssd/host-builds/kernel-upgrade/`) | sha256 |
|---|---|---|
Fix round 1 (code at `7c0493496`):

| Item | Path (gekko `/mnt/ssd/host-builds/kernel-upgrade/`) | sha256 |
|---|---|---|
| Image | `panel-bright-work/images/Image-suspend-s0-7c0493496` | `fea8dbe1913f95e59d93b1cfaf4ae35a7eba0db0a606c72ae29277f769314fc7` |
| config (unchanged) | `panel-bright-work/images/config-suspend-s0-7c0493496` | `1ec1bef6e867e455921af6dd6f57ace26a543d9257bf56ff71fde350b6e4fa6a` |
| S0 DTB (a..d), unchanged | `panel-bright-work/dtb/exynos9810-crownlte-delayed-ufs-suspend-s0-diag-7c0493496.dtb` | `61e670ec5078b8e9f8c1a07b520beb102a7476c545e69cd9de1165c4c7259050` |
| S0ab DTB (a+b), unchanged | `panel-bright-work/dtb/exynos9810-crownlte-delayed-ufs-suspend-s0ab-diag-7c0493496.dtb` | `362e56d366f53a87394f24969452c254991362824819b9ae358fa18c69a8b4de` |
| Trial 1: S0ab RECOVERY | `acpm-suspend-s0ab-r1-s2/recovery.img` | `0b956baad13d1b7cbe5923e1726f8565e14c7b9ca81746f5033380b7c93a72fe` |
| Trial 2: S0 RECOVERY | `acpm-suspend-s0-r1-s2/recovery.img` | `c39885265bb9587db27cba96fc7aacb44190ec129f866c62bf4078e30ae2df74` |

Superseded (code at `4877cf53b`, panics in probe on PMU 0x30; gekko
copies of the packs deleted, PC copies in
`C:\Users\Patrick\tools\trial-acpm-suspend-{s0ab,s0}-20261107`): Image
`1fa72477…`, S0ab RECOVERY `7b48380b…` (ran 2026-11-06 23:46), S0
RECOVERY `f787654d…` (never run).

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
| ~0.3 s (probe) | `s3c2410-wdt ... system suspend keeps it armed (timeout 60 s while suspended)`; `s3c2410-wdt ... crownlte deadline: no reload after 145000 ms boot time`; `suspend s0: probe: watchdog deadline 145000 ms installed before any cross-CPU request`; `suspend s0: probe: stages abcd ...` |
| 20 s | `deadline 145000 ms installed at probe in the watchdog ping path`; S0a; recovery check (WTCNT moving) before S0b |
| ~20-35 s | S0b (170 hotplug cycles) |
| ~35-75 s | S0c (5 levels × 5 s + overhead). **COM4 drops** at the `devices` level (dwc3 suspends) and comes back on resume. `ready-capture.ps1` reopens the port, and PID1 resends its cached log. |
| ~75-80 s | S0d (COM4 drops again) |
| then | `SUMMARY` four times, 5 s apart |
| ~106 s | `pmic selftest:` (read-only) |
| 120 s | PID1 restart (150 s fallback); shim → Lineage |
| 145 s | only if still up: the next ping attempt from any CPU logs `s3c2410-wdt: deadline 145000 ms passed: counter no longer reloaded`, the hrtimer adds `deadline 145000 ms reached ...`; WDT reset within 30 s |

After return, `run-trial.sh` saves `/sys/fs/pstore/pmsg-ramoops-0`
(the 6.12 console tail; this DT's ramoops console is 16 KB, so only the
last ~150 lines) and restores bright RECOVERY `8763860c`.

## 5. Expected PASS lines

`<...>` marks values that are not known yet. Kernel lines without the
prefix are shown where they matter.

### S0a

```
suspend s0: S0a CP_STAT=<cp> SEQ_CP cfg=<..> status=<..> DFD=<..> G3D=0xf
suspend s0: S0a INFORM0..7=0 0 0 0 0 0 0 0 FLEXPMU_INFORM=<..>
suspend s0: S0a WAKEUP_MASK 0x60c=<..> 0x610=<..> 0x614=<..> 0x618=<..> 0x644=<..> WAKEUP_STAT=<..>/<..>
suspend s0: S0a PAD_RET top=<..> ... IDLE_IP=... IDLE_IP_MASK=...
suspend s0: S0a INTR_GEN en/pend grp1=... grp2=... grp4=... EVENT_INT_EN_GRP2=<..>
suspend s0: S0a FRC=<..> G_TCON=<..>(start=1) CNTPCT=<..> (read on cpu0)
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
suspend s0: step 30 enter (S0 dry run, no SMC): fn=0xc4000001 state=0x3000000 entry=<__pa(cpu_resume)> cpu0 self-IPI pending=1 (ISPENDR0=<bit1>) wdt=60s en=1 rst=1 cntpct=<a> frc=<b>
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
| Kernel panic in `e9810_snap` / `regmap_mmio_read32le`, `synchronous external abort` (pstore `dmesg-ramoops-0`) | A register read that the secure world protects (the 4877cf53b build read PMU 0x30). Fix round 1 reads only the §9 set. | Panic → reboot → shim bounce → Lineage (seen 2026-11-06 23:46). |
| `suspend s0: probe FAIL: <why>` and, at ~22 s, `SUMMARY ... probe failed: <why>` | The driver did not start (bad DT, CAL, topology, or the DFD dump is enabled so `big_reset_control()` would be needed). Nothing ran. | Normal 120 s restart. |
| `probe FAIL: no armed, petted CL0 watchdog to carry the 145000 ms deadline` | The guarded watchdog is not running with the reset enabled (INFORM not normal, DFD set, or the PMU still masks the reset). No cross-CPU request was sent, nothing ran. | Normal 120 s restart. |
| `cpuN did not run the <what> request within 200 ms (request left to it)`, then `s3c2410-wdt: petting stopped for good (suspend s0: a CPU does not answer cross-CPU calls)` | A CPU counted online does not take IPIs. At probe (`MPIDR read`, `hook reset`) the probe fails as well. | WDT reset ≤ 30 s → shim bounce. |
| `S0b refused (no hotplug done): <why>` | No live, reset-enabled, still-petted watchdog with the deadline (B2 gate; `counter not moving` means WTCNT did not change in ~50 ms). | S0b SKIPPED; S0c/S0d not run. |
| `S0b <phase> cpuN off#n not started: <why> FAIL` | The watchdog stopped being a recovery path during S0b (deadline passed, petting stopped). No further core goes down. | S0b FAIL. |
| `s3c2410-wdt ... suspend refused: ...` with `PM: Some devices failed to suspend` | Keep-armed suspend could not keep the watchdog armed (reset disabled, petting stopped, suspend timeout refused, WTCON wrong afterwards). The watchdog keeps running; the suspend aborts. | S0c level / S0d FAIL. |
| `CPU-LOST cpuN: PSCI CPU_ON bring-up failed` with `LOST cpuN phyN status=... inform=... grp1 en/pend=... grp2 en/pend=...` and an `apm-ring` dump. Kernel: `CPUN: failed to come online`. | CPU_OFF worked but CPU_ON did not bring the core back. This is the hotplug form of the EL3 LITTLE hang. The kernel counts it offline, so nothing waits on it. | S0b FAIL, S0c/S0d SKIPPED, `CPU-LOST N: offline; the normal 120 s restart applies`. |
| `CPU-LOST cpuN: online but not answering IPIs` / `bring-up never completed` / `tear-down never completed`, then `s3c2410-wdt: petting stopped for good (...): SoC reset in about 30 s` | The core is counted online but does not run, or a hotplug call is stuck holding the hotplug locks, which the restart path needs. | WDT reset ≤ 30 s → sboot RECOVERY → shim bounce → Lineage. The pstore tail ends with these lines. |
| `cpuN still powered <ms> ms after CPU_DEAD ... (4.9 would BUG here)`; S0b `still-on N` > 0 | PSCI CPU_OFF returned, but the PMU still reports the core powered. Recorded, not fatal in S0. | S0b can still PASS, but it **blocks S1**: SYS_SLEEP needs all cores off. |
| `S0b ... hooks: ... cal=<err> FAIL` / `cluster_disable not run by cpu4` | A CAL core or cluster sequence failed, or a hook did not run. | S0b FAIL. |
| `S0b end: changed INFORM[n]: a -> b` (or `EVENT_INT_EN_GRP2`, `INTR_GEN GRP2 enable`, `APM2AP bit24`, `EINT mask gpa`) | The hotplug hooks left PMU state behind. | S0b FAIL (`regs not restored`). |
| `S0c refused: <why>` / `S0d refused: <why>` | A precondition failed (UFS bound, charger kick, CPU lost, the B2 watchdog check or no keep-armed-in-suspend, ops not registered, no PM_DEBUG; `pm_suspend_test_level` returns `-EBUSY` if another system transition holds the lock). | SKIPPED. Nothing was suspended. |
| `S0c level=<l> ... -> FAIL: pm_suspend returned an error`, with kernel `Freezing ... failed`, `PM: Some devices failed to suspend`, `PM: Wakeup pending, aborting suspend` / `active wakeup source: <name>` | A task or device refused, or a wakeup source is active. | S0c FAIL at that level; later levels and S0d do not run. |
| `step 20 refused: EMBEDDED_G3D_STATUS=<x> (not on)` | G3D is powered off; the CAL save would read a dead CMU. Refused before any write. | `core` level FAIL (`unexpected syscore callbacks`), S0d not run. Fix before S1: power G3D on, or make the save conditional. |
| `step 20 cal_pm_enter_unarmed(8) failed <err> ... undone` | The CAL refused (table without the arming write: `-ENOENT`) or a sequence failed. The step undid itself. | `core` level FAIL. |
| `step 50 syscore resume exit FAILED` / `earlywakeup FAILED` | The CAL exit or earlywakeup sequence returned an error. | S0c `core` / S0d FAIL. |
| `S0c ... cpu0-WFI-in-window` / `S0d ... cpu0-WFI-in-window=1` | CPU0's GRP2 event latched while the system was entered. The APM was not told (enable withheld). | Logged. Investigate before S1. |
| `S0d ... FAIL: self-IPI not seen pending before the call` | The 4.9 `test_early_wakeup` precondition does not hold on 6.12 (GIC-400 SGI1 pending bits not set). | A future firmware early-wake test cannot rely on a self-IPI. |
| `S0d ... FAIL: FRC/arch counter not counting` | MCT FRC or CNTPCT did not advance across `.enter`. | Must be understood before S1 (design R3). |
| `s3c2410-wdt: deadline 145000 ms passed: counter no longer reloaded ...` and/or `deadline 145000 ms reached and the system is still up` | The PID1 restart and its 150 s fallback did not happen by 145 s. | No CPU can reload the counter any more → WDT reset ≤ 30 s → shim bounce. |
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
3. **Core lost while counted online, a cross-CPU request unanswered, or
   still up at 145 s.** The driver stops the petting, or the watchdog's
   own ping path refuses every reload after 145 s (any CPU), and path 2
   follows.
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

## 9. Register access audit (fix round 1)

Rule: the driver touches a register directly only if the 4.9 kernel
running on 9810 reaches that register by plain non-secure MMIO. 4.9
tree: gekko `src/close-range-fix-4.9`. "R" = 4.9 reads it, "W" = 4.9
writes it. A CAL `PMUCAL_WRITE` with a partial mask is a read-modify-
write (`pmucal_rae.c:116-125`), so it proves both. 4.9 runs the CAL
tables below with `pmucal_rae_handle_seq()` / `_save_seq()` /
`_restore_seq()` (`pmucal_rae.c:128-284`), which never use SMC. Only
`pmucal_rae_handle_cp_seq()` routes PMU offsets 0x30/0x34 to SMC
(`pmucal_rae.c:296-306`). Paths are under `drivers/soc/samsung/` unless
given in full; `T/` = `cal-if/exynos9810/`; `DTS` =
`arch/arm64/boot/dts/exynos/exynos9810-crownlte_eur_open_26.dts`.

### Removed or never accessed

| Register | Was used for | Why removed | 4.9 evidence |
|---|---|---|---|
| PMU 0x30 CP_CTRL_NS, 0x34 CP_CTRL_S | S0a snapshot | **Secure-only.** The read aborted on HW (S0ab 7b48380b, 0.348 s). Not replaced by an SMC: the driver keeps no smc/hvc. | `cal-if/pmucal_cp.c:206-221` (`pmucal_is_cp_smc_regs()`), `:224-317` (SMC 0x82000700 READ_CTRL/WRITE_CTRL); only users `T/pmucal_cp_exynos9810.h:3-4,20,26,31-40` |
| INTR_GEN GRP0 EN/PEND (0x000/0x008), GRP3 EN/PEND (0x300/0x308) | S0a snapshot | 4.9 never reads PEND; S0 does not need these groups. | only EN writes at `T/flexpmu_cal_system_exynos9810.h:2584,2587` |
| GICD_SPENDSGIRn (0x10101f20..) | S0a snapshot, S0d self-IPI check | 4.9 reads it only in `drivers/irqchip/irq-gic.c:940` (`gic_migrate_target`, big.LITTLE switcher, not built for 9810). S0d now uses GICD_ISPENDR0 bit 1, which shows a pending SGI from any source. | none |
| PMU CPU CONFIGURATION (status − 4: 0x2000..0x2038) | lost-core dump | 4.9 touches it only in `exynos-pmu.c:73` (`pmu_cpu_ctrl`, `exynos_cpu` power ops), not shown to run on 9810. | none |
| Any other PMU wakeup-mask offset | DT `samsung,wakeup-mask-offsets` | The driver now accepts only 0x610, 0x614, 0x618, 0x644. | DTS:4914-4933 |

### Kept

| Block / register | Driver use | 4.9 evidence (non-secure MMIO on 9810) |
|---|---|---|
| PMU 0x0038 CP_STAT | R (snapshot, syscore CP gate) | `exynos-pmu.c:172-177` R |
| PMU 0x0280 CENTRAL_SEQ_CP_CONFIGURATION | R | `T/pmucal_cp_exynos9810.h:21` RMW in the non-SMC branch `cal-if/pmucal_rae.c:307-312` |
| PMU 0x0284 CENTRAL_SEQ_CP_STATUS | R | `T/pmucal_cp_exynos9810.h:23,27` WAIT (R) |
| PMU 0x03e0-0x03ec IDLE_IP0..3 | R | `exynos-powermode.c:137` R, `:305` RMW |
| PMU 0x03f0-0x03fc IDLE_IP_MASK0..3 | R, W (syscore save, all masked, restore) | `exynos-powermode.c:194` W. **Read justified, not proven:** 4.9 only writes it. 4.9 treats the 9810 secure registers per offset and the same for reads and writes (`pmucal_is_cp_smc_regs()` sends reads and writes of the same offsets to SMC), so an offset that 4.9 writes from the non-secure side is not a secure-only one. IDLE_IP in the same block is read by 4.9. |
| PMU 0x0500 RESET_SEQUENCER_CONFIGURATION | R (DFD check) | `exynos-reboot.c:137,144,164,191,293` R |
| PMU 0x0600 WAKEUP_STAT | R | `exynos-pm.c:216` R |
| PMU 0x060c EINT_WAKEUP_MASK | R | `exynos-pm.c:92` R; `exynos-powermode.c:639` W |
| PMU 0x0610 / 0x0614 / 0x0618 / 0x0644 wakeup masks | R, W (syscore) | `exynos-powermode.c:642` W, offsets DTS:4918,4923,4928,4933 (SYS_SLEEP column `0xd00f7e7e 0xffff00ff 0xffff00ff 0`). **Read justified as for IDLE_IP_MASK**; 4.9 reads 0x600, 0x60c and 0x640 of the same group. |
| PMU 0x0640 WAKEUP_STAT2 | R | `exynos-pm.c:131` R (`extra_wakeup_stat`, DTS:4884 = 1600 = 0x640) |
| PMU 0x0860-0x087c CPU_INFORM0..7 | R | `cal-if/pmucal_powermode.c:26,35` W (`T/flexpmu_cal_powermode_exynos9810.h:2-9`). **Read justified as for IDLE_IP_MASK**; the 6.12 `drivers/cpuidle/cpuidle-exynos9810.c:911,1059` C2 diagnostics also read it. |
| PMU 0x0880 FLEXPMU_INFORM | R | `exynos-pm.c:383,644` R |
| PMU 0x2004..0x203c CPU STATUS (LITTLE 0x2024+8n, BIG 0x2004+8n) | R (CPU_DEAD poll, snapshot, dump) | `exynos-pmu.c:83` R (`core_status` sysfs, `:210`); `T/flexpmu_cal_cpu_exynos9810.h:15,67` (`coreNN_status` READ) |
| PMU 0x2044 / 0x204c NONCPU STATUS | R | `exynos-pmu.c:113` R; `T/flexpmu_cal_cpu_exynos9810.h:115,124` READ |
| PMU 0x3060/0x3080/0x30a0/0x30c0/0x3120/0x31a0 PAD_RETENTION | R | `T/flexpmu_cal_system_exynos9810.h:1272-1277` RMW (exit_sleep) |
| PMU 0x4044 EMBEDDED_G3D_STATUS | R (syscore G3D gate) | `T/flexpmu_cal_local_exynos9810.h:480,488,495` WAIT/READ |
| PMU 0x7f08 EVENT_INTERRUPT_ENABLE_GRP2 | R | `T/flexpmu_cal_system_exynos9810.h:128`, `T/flexpmu_cal_cpu_exynos9810.h:7,12` RMW |
| INTR_GEN 0x14070100 / 0x108 GRP1 EN / PEND | R | `T/flexpmu_cal_cpu_exynos9810.h:269` RMW; `:11` COND (R of 0x108) |
| INTR_GEN 0x14070200 / 0x208 / 0x20c GRP2 EN / PEND / CLEAR | R; W CLEAR (dying CPU) | `T/flexpmu_cal_cpu_exynos9810.h:5,6,10`: RMW of 0x200, COND R of 0x208, W of 0x20c |
| INTR_GEN 0x14070400 / 0x408 GRP4 EN / PEND | R | `T/flexpmu_cal_system_exynos9810.h:2588,2622` W/RMW, `:1284` COND (R of 0x408) |
| GPIO_ALIVE 0x14050900-0x910 EINT mask gpa0..4 | R | `drivers/pinctrl/samsung/pinctrl-exynos.c:103,138` RMW of `eint_mask + eint_offset`, `:501` (`exynos7_wkup_irq_chip`, 0x900), banks `:1930-1935` (offsets 0x00-0x10), DTS:316-317 |
| GPIO_ALIVE 0x14050a00-0xa10 EINT pend gpa0..4 | R | `exynos-pm.c:99,137,206` R (DTS:4874 `gpio_alive_base` 0x14050000, `num-eint` 34) |
| GICD 0x10101200-0x23c ISPENDR0..15 | R (snapshot, S0d self-IPI, early-wakeup dump) | `exynos-pm.c:210` R (DTS:4874 `gicd_ispendrn_base` 0x10101200, `num-gic` 16) |
| MCT 0x10040100 / 0x104 G_CNT_L/U, 0x240 G_TCON | R | `drivers/clocksource/exynos_mct.c:184-189` R, `:166,268,280` R (DTS:4996-4998 `mct@10040000`) |
| FlexPMU debug SRAM 0x0204f800, lines 10/22/23, words +8/+12 | R | `exynos-flexpmu-dbg.c:110,118,134,142` R at `base + 16*line + 8 (+4)`; lines `DID_SW_FLAG` (`:72`), `DID_AP_COUNT_SLEEP` (`:84`), `DID_MIF_COUNT_SLEEP` (`:85`); DTS:9216-9221 (`data-base` 0x0204f800, size 1024) |
| APM mailbox 0x14100010 APM2AP | R | `T/flexpmu_cal_cpu_exynos9810.h:109,112,118,121` RMW (bit 24) |
| APM log ring (APM SRAM) | R, through `acpm_ipc_log_snapshot()` | `acpm/acpm_ipc.c:144-165` R; also read on 6.12 hardware in c2diag-a |
| CL0 watchdog WTCON/WTDAT/WTCNT | only through `crownlte-wdt.h` | `drivers/watchdog/s3c2410_wdt.c` (4.9 and 6.12); 6.12 HW q35/q37 |

### CAL tables run by this driver

`cal_cpu_disable/enable`, `cal_cluster_disable/enable`,
`cal_pm_enter_unarmed(8)` (hint + `save_sleep` + `enter_sleep` minus the
GRP2 enable), `cal_pm_exit(8)` (`exit_sleep` + `save_sleep` restore) and
`cal_pm_earlywakeup(8)`. The tables `flexpmu_cal_system_exynos9810.h`,
`flexpmu_cal_cpu_exynos9810.h`, `flexpmu_cal_powermode_exynos9810.h`,
`flexpmu_cal_p2vmap_exynos9810.h`, `flexpmu_cal_define_exynos9810.h` and
the code in `pmucal_cpu.c` / `pmucal_powermode.c` are byte-identical to
4.9 (`diff -q`). None of these tables contains PMU offset 0x30 or 0x34
(grep `0x14060000, 0x00(30|34),` matches only `pmucal_cp_exynos9810.h`).
4.9 runs them all through the plain-MMIO RAE paths above, from the same
suspend flow (`exynos-pm.c` syscore → `cal_pm_enter` →
`pmucal_system_enter`). The 6.12 `pmucal_rae.c` keeps the SMC routing
only in `pmucal_rae_handle_cp_seq()`. No entry of the unarmed/exit path
is SMC-routed in 4.9, so this driver reaches by MMIO nothing that 4.9
did not also reach by MMIO.

Residual: the `save_sleep` list and the `exit_sleep` DRCG writes touch
~1100 CMU/SYSREG registers of many blocks without a power or clock
condition, as 4.9 does. If 6.12 leaves a block's clock gated where 4.9
had it running, such an access can stall the bus (no abort; a hang that
the kept-armed watchdog catches). S0c `core` and S0d are the first runs
of these lists on 6.12.

## 10. Fix log

### Fix round 1 (2026-11-07): commits `766bcf035`, `dca7ce452`, `536117c66`, `7c0493496`

Input: the S0ab hardware run (pack `7b48380b`, 2026-11-06 23:46) and Sol
R1 (verdict CHANGES) on `dfb1db390`.

| Finding | Fix |
|---|---|
| HW: synchronous external abort at 0.348 s, `regmap_mmio_read32le` ← `e9810_snap+0x50` ← `e9810_pm_probe`, PMU+0x30 | CP_CTRL_NS/S reads removed, no SMC added. Every access audited (§9). Unproven ones dropped (INTR_GEN GRP0/3, GICD_SPENDSGIRn, CPU CONFIGURATION) or justified (IDLE_IP_MASK, wakeup-mask and CPU_INFORM reads). Wakeup-mask offsets limited to the four 4.9 ones. The phone came back by itself (panic → reboot → shim bounce). |
| B1: both deadline hrtimers end up on CPU0 after S0b offlines CPU7 | `s3c2410wdt_crownlte_set_deadline()`. Every reload path (`keepalive` from any CPU or context, `start` of a running counter, crownlte ping, `stop_petting`, restart notifier, the last re-checked under the lock in `7c0493496`) checks the deadline under `wdt->lock` and refuses after it. One hrtimer is kept as a supplement, armed locally (no IPI). Once a deadline is set, user space can neither stop the watchdog nor change its timeout. |
| B2: S0b starts without verifying recovery | `s0_recovery_refused()` needs the guarded watchdog enabled, reset enabled, still petted, with the deadline installed and WTCNT seen moving (at most 20 × 2-3 ms). It is required before S0b (else SKIPPED, no hotplug) and checked again, without the liveness wait, before every single CPU-off. S0c/S0d use it plus keep-armed-in-suspend. |
| B3: probe `smp_call_function_single(wait=1)` unbounded | `e9810_xcall()`: `smp_call_function_single_async()` with a heap request that is leaked on a timeout (valid memory for a late runner), a 200 ms bound and `stop_petting` on a timeout. Used for the MPIDR read, the hook reset and all pings. The deadline is installed before the first request, probe fails without an armed watchdog, and the deadline property is mandatory. |
| B4: keep-armed suspend falls through to stopping the watchdog | `s3c2410wdt_suspend_keep_armed()` needs the reset enabled and petting alive. A failed suspend timeout leaves the watchdog running at the old timeout, WTCON is checked afterwards, and every failure returns an error so the suspend aborts. A refusal after the switch to the 60 s count puts the run-time timeout back, so the slower prescaler cannot stretch a due reset (`7c0493496`). Suspend never stops a watchdog that is under a deadline. |
| M5: the `pm_suspend_test_level()` transaction is not serialized | One `mutex_trylock(&system_transition_mutex)` covers set + suspend + restore. `enter_state()` is split into `enter_state_check()` / `enter_state_locked()`; nothing locks recursively. |

Build: `config-suspend-s0-7c0493496` (unchanged), clang 18. `W=1` on the
changed objects gave no warnings. `exynos9810-pm.o` has no
`smc`/`hvc` instruction (`llvm-objdump`), no PSCI symbol and no
`smp_call_function_single`. Artifacts in §3.

Residual risks (fix round 1):

- The IDLE_IP_MASK, wakeup-mask and CPU_INFORM reads rest on the
  per-offset argument in §9, not on a 4.9 read. If one of them aborts,
  the signature is the probe-time panic of §6 and the phone returns by
  itself.
- `ktime_get_boot_fast_ns()` stands still while timekeeping is suspended
  (syscore → `.enter` → syscore resume). A deadline that falls in that
  window takes effect at the first ping after resume. The kept-armed 60 s
  watchdog covers that window anyway.
- A probe that fails after the deadline was installed leaves the
  deadline in place: if the vehicle has not restarted by 145 s, the
  watchdog resets the SoC.
- The S0b liveness check only shows that the counter moves and that
  WTCON has the reset enabled. The PMU reset routing was proven in
  q35/q37 and is not re-checked here.
- The CAL save/exit lists (§9, last paragraph).
