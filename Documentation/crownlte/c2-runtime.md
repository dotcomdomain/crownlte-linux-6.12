# Runtime C2 on crownlte 6.12: S0 (self-recovery) and S1a (big C2)

Branch `crownlte-c2-runtime` on `crownlte-integration-6` 4148bf525. Design:
`roadmap/c2-runtime-design.md` (gekko), sections 4.0 (S0) and 4.1 (S1a, C1).
Build only. Nothing here has run on the phone yet. Fix round 1 (C2 Sol R1) is in
section 7; the record format is in section 6.

| Commit | What |
|---|---|
| d9c0de8a9 | watchdog: `crownlte_wdt_stop_petting()` (reverted in d72797164) |
| 05b293073 | mct-bc: `exynos9810_mct_bc_ready_ms()` |
| f1a2ddbb2 | cpuidle: S0 chain + delayed boot arming (C1) |
| 59b2fab54 | `arch/arm64/configs/crownlte_c2.fragment` (buddy hard-lockup detector, panic) |
| a6bcb13f5 | DT: quarantine page, ROM-RAM `v23a-int6-c2`, two drill DTBs, Wi-Fi self-test dropped from the int6 DT |
| ce5267cee | this document (round 0) |
| d72797164 | revert of d9c0de8a9 (the C2-only watchdog stop) |
| 68234bc90, 89a2a1bc1, e6c1f71a1, f8f6c8146 | cherry-picks of suspend-s0 4800e0e24, 766bcf035, 7c0493496, 7b58093af (watchdog: keep-armed suspend, deadline in the ping path, no stretched or late reload, timeout changes serialized) |
| ea54a6b81 | watchdog: one crownlte final-stop state for suspend and C2 |
| b57b23022 | cpuidle: fail-closed v2 record, armed session, hotplug-safe detector, consumed drills, per-CPU boot arming |
| f3ee75781 | DT: trial tokens (S1a, drill b), comments |
| fbcc1d167 | watchdog: the restart notifier reloads exactly once (the timeout without a reload, then the final stop) |

## 1. What it does

### S0: self-recovery chain (any DTB where the cpuidle driver sets up)

1. **Lost-core detector** (`cpuidle-exynos9810.c`). Every 2 s a work on CPU0 looks at
   every online CPU except CPU0. A CPU that has sat in the firmware stage (PSCI
   CPU_SUSPEND issued) for more than 1 s gets an async csd ping. If it has not answered
   after 200 ms, it gets a reschedule IPI and another 200 ms. It is lost only if:
   - a ping was really enqueued (or an earlier one is still pending). A ping that cannot
     be enqueued (CPU went offline) gives no verdict;
   - it went unanswered both times;
   - after each wait the CPU is still online, still in the firmware stage and still in
     the same firmware entry. Each C2 entry bumps a per-CPU generation (`fw_gen`),
     published before the stage. The scan samples it, so a CPU that left C2 (and maybe
     entered it again) is never declared lost.

   Scans hold `cpus_read_trylock()` and never wait for the hotplug lock. If hotplug holds
   the lock, the scan is skipped. After 5 skipped scans in a row (a hotplug operation
   that hangs, for example on a lost CPU), the scan runs without the lock and relies on
   the revalidation alone (`det_hp_unlocked`).

   A healthy CPU in a long C2 period answers within its exit latency, so the cost is at
   most one extra wake per 2 s per sleeping CPU.
2. **On a loss** (`e9810_c2_lost()`, process context, never from panic):
   - a `C2-LOST cpuN` dump (pr_emerg, so it reaches console-ramoops);
   - the loss written into the quarantine record (section 6);
   - deep idle off, every CPU disarmed;
   - the watchdog's one shared **final stop**, `s3c2410wdt_crownlte_stop_petting()`
     (section 5): at most one last reload, then never reloaded, stopped, slowed or masked
     again. The reset follows within the current timeout (30 s: DT `timeout-sec` of the CL0 watchdog in these DTBs);
   - `orderly_reboot()`;
   - `emergency_restart()` 75 s after the loss as a last resort.

   A drill does not reboot by itself at first: it waits 45 s for the watchdog reset, so
   the trial proves the watchdog path.
3. **Buddy hard-lockup detector** (`crownlte_c2.fragment`): `HARDLOCKUP_DETECTOR=y` (buddy,
   since there is no pseudo-NMI on GICv2) and `BOOTPARAM_HARDLOCKUP_PANIC=y`. A CPU whose
   watchdog hrtimer stops is reported by its buddy after about 12-24 s, and the kernel
   panics (`panic=30`, then `emergency_restart()`). Soft lockups are only reported.
   **The panic path writes nothing.** The record already says "session armed" (step 5),
   so the next boot keeps all C2 off.
4. **Quarantine record**: 4 KB no-map page `c2-quarantine@ff607000`
   (`exynos9810-crownlte-c2.dtsi`, `memory-region` of the cpuidle node), format v2 in
   section 6. RAM only: no UFS or eMMC write, and no other storage.
   - Placement: this is the last page of the 4.9 exynos-snapshot carve-out
     (0xfda00000..0xff608000, already no-map here as `snapshot-after-pstore`, now 4 KB
     shorter). It is **not** right after ramoops: 4.9 writes `log_kevents` at
     0xfed18000..0xff518000. 4.9 places no snapshot item in 0xff518000..0xff608000.
   - **Fail closed.** At probe, before anything can be armed:
     - no valid record (cold boot, another kernel, garbage, a torn page) → **all C2
       off**, and a valid record saying `UNINIT` is written;
     - a record whose session is still `ARMED` → **all C2 off** (`UNCLEAN`): the boot
       that armed C2 ended without a clean restart or power-off;
     - a big loss → all C2 off; a LITTLE loss → LITTLE C2 off, big allowed;
     - only a valid record with none of these flags allows C2.
   - **Explicit clear only:**
     - root: `echo 1 > /sys/bus/platform/devices/14070000.cpuidle/c2_quarantine_clear`
       takes effect on the next boot;
     - or a trial DTB's `samsung,c2-quarantine-trial-token = <N>`: a one-shot clear at
       probe of a **valid** record whose stored token differs from N. N is then stored.
       A missing or invalid record is never cleared this way: that boot stays fail-closed
       and writes `UNINIT`, and the next boot of the same pack clears it. That next boot
       also shows that the page kept its contents across the restart. ROM-RAM has no root
       shell (adb root stays shell), hence the token.
   - **Without the page, boot arming and both drills are refused.** Only the DT-gated
     self-test may still arm (diag DTBs without the page; it watches its own phases).
5. **Armed session (B2).** Before the first CPU is armed, the record's session word is
   set to `ARMED` and read back. Arming fails if that write fails. The reboot notifier
   clears it (`IDLE`) only after `cpuidle_pause()` has seen every CPU leave C2, and never
   after a loss. These paths run the notifier and therefore count as a clean end:
   `kernel_restart()` (reboot(2) `RB_AUTOBOOT`, `orderly_reboot()`, Android `reboot`, the
   diag PID1 restart at 120 s and its 150 s fallback, the ROM-RAM DONE reboot),
   `kernel_power_off()` and `kernel_halt()`. These do not run it, so the next boot is
   `UNCLEAN`: panic (buddy hard-lockup, any oops with `panic_on_oops`),
   `emergency_restart()`, a watchdog reset (CPU0 wedge, C2-LOST), a PMIC reset and power
   loss. Nothing arms once a restart or power-off has begun.
6. **Observability**, root-only files in `/sys/bus/platform/devices/14070000.cpuidle/`:
   - `c2_armed`, `c2_lost`, `c2_stats` (per CPU: stage, fw_gen, enter, resumed, early,
     psci_fail, unarmed, guarded, wfi_fallback, stale_grp2, last_err; then deep_off,
     det_scans, det_pings, det_enq_fail, det_revalidated, det_hp_unlocked, boot_pending),
     `c2_quarantine` (the newest record, both slots' validity and sequence, the flags
     in force);
   - write-only `c2_disarm` (kill switch until reboot: also stops boot-arming retries
     and online re-arming), `c2_quarantine_clear` and `c2_drill_rearm`;
   - `c2 stats:` KERN_INFO lines at the first detector pass and every 10 min.

   adb root does not work on ROM-RAM, so read these from the serial log or a root shell.
   The cpuidle sysfs in section 4 is world-readable.
7. **Drills** (DT, diag DTBs only, both need the page). Each runs **once**: its bit in the
   record's `drills` word is persisted before it fires or arms anything, and a set bit
   skips it on later boots. Root `echo 1 > .../c2_drill_rearm` re-arms both for the next
   boot. If the bit cannot be persisted, the drill is skipped.
   - `samsung,c2-lost-drill = <cpu ms>` (drill a): at `ms` boot time the detector treats
     `cpu` as lost (simulated). **Scope:** it covers the record write → disarm →
     watchdog final stop → watchdog expiry → reset path. It does **not** exercise the
     real candidate selection, the csd/IPI delivery, the revalidation or a real core in
     EL3. Those are covered only by drill (b) and real losses.
   - `samsung,c2-loss-drill-ms = <ms>` (drill b): arms cpu1-3 (LITTLE, no concurrency
     limit), the configuration that lost a core in q22b, q40, q43 and c2diag-a/b. It
     disarms them after `ms` if nothing was lost. A LITTLE (or all-C2) quarantine skips it.
8. CPU0 is refused explicitly. Any self-test phase still works: the detector skips the
   CPUs of a running phase, and an unrecovered phase loss enters the same chain
   (reason `selftest`).

### S1a: big C2 at runtime

- **C1, arming order, per CPU:** `samsung,c2-boot-cpus` is no longer applied in probe. A
  work runs at `samsung,c2-arm-delay-ms` boot time and arms each CPU on its own, so one
  CPU's refusal no longer refuses the others. A CPU refused because the MCT tick
  broadcast is not active yet (the cpuidle probe runs before mct-bc, link order) or
  because it is offline is retried every 1 s for up to 60 s. After that:
  - a CPU still offline stays pending and is armed when it comes online (cpuhp callback
    → the same work);
  - a CPU still waiting for the MCT broadcast is dropped.
  Any other refusal is final for that CPU.
- **ROM-RAM DTB** `exynos9810-crownlte-romram-v23a-int6-c2` = int6 +
  `exynos9810-crownlte-c2.dtsi` + `samsung,c2-boot-cpus = <0xf0>` +
  `samsung,c2-arm-delay-ms = <45000>` (after the 40 s boot QoS) +
  `samsung,c2-quarantine-trial-token = <0xc251a001>`. LITTLE stays WFI.
- **int6 DT change:** `samsung,wifi-selftest` and `-ms` were removed from
  `exynos9810-crownlte-romram-wifi.dtsi` (Sol finding on int6). This changes the int6 DTB
  too: 297eb4f4 → c30c8d04, and the decompiled diff is exactly those two properties.

## 2. Artifacts (gekko `/mnt/ssd/host-builds/kernel-upgrade`)

Fix round 1, `crownlte-c2-runtime` fbcc1d167. Config `panel-bright-work/ko-c2-fbcc/config`
= int6 config (c7b917d1) + `crownlte_c2.fragment`, `olddefconfig`, unchanged from round 0
(668b03c4; the fragment change is a comment). It adds only `HARDLOCKUP_DETECTOR`,
`_BUDDY`, `_COUNTS_HRTIMER`, `BOOTPARAM_HARDLOCKUP_PANIC`, `LOCKUP_DETECTOR` and
`SOFTLOCKUP_DETECTOR`. Banner: `Linux version 6.12.93-crownlte-diag+`. W=1 clean for
`s3c2410_wdt.o` and `cpuidle-exynos9810.o`.

| File (`panel-bright-work/ko-c2-fbcc/`) | sha256 |
|---|---|
| `arch/arm64/boot/Image` | 17544cf3e22adbe8a46202278438ce054f15d7cf848dcd82fcfb41ddfe0a5d92 |
| `.../exynos9810-crownlte-romram-v23a-int6-c2.dtb` (S1a ROM-RAM) | 2bc3c73c15d535202a8c794f18373b5419ffc0ce31947d248834f578ebb29d17 |
| `.../exynos9810-crownlte-romram-v23a-int6.dtb` (int6 minus Wi-Fi self-test, unchanged) | c30c8d048dc10eb2ad841e6748b7063be9dc3a8ccbdb8c29428f0fbe8f4c24ee |
| `.../exynos9810-crownlte-delayed-ufs-c2-drill-lost-diag.dtb` (drill a, unchanged: comments only) | 6488152a30e86eccaf392ae7749bd691ce87e05ef96da2413202860c41ec53c2 |
| `.../exynos9810-crownlte-delayed-ufs-c2-drill-loss-diag.dtb` (drill b, + token 0xc2b10001) | 1775625f0681dc4bde2f6626a2cdda44b185d24e62d06aba008ea4c3dcbab1c7 |
| `config` | 668b03c456c204e0473e883e5a31e0edd1c90349dd52537fd79ba478f16ffa7c |

Packs (shim bbea959, parent a8198be0, restore RECOVERY 8763860c). Each pack dir has a
PROVENANCE.txt:

| Pack | recovery.img sha256 | Inputs |
|---|---|---|
| `c2-drill-lost-r1-20261107` (delayed-UFS, drill a) | aea3534a475e8b034f04dae35d64f3122bfb391a0b506879593a93cd32423583 | `pack-acpm-q-paths.py`, Image 17544cf3, DTB 6488152a, ramdisk 506232b1, loader 32df794d. The manifest `purpose`/`kernel_source` are packer defaults; see PROVENANCE.txt |
| `rom-ram/pack-romram-int6-c2-r1` (ROM-RAM S1a) | f655965cd76d6f91496b96d3daef76565faf9caac7a44c6740a75d62035f606a | `rom-ram/pack-rom-ram.py`, KO `panel-bright-work/ko-c2-fbcc`, DTB 2bc3c73c, ramdisk build-30 e615cc8d, loader 1b9c5014 |

The round-0 packs (`c2-drill-lost-20261107` 57715ff5, `rom-ram/pack-romram-int6-c2`
611cbbb0) were deleted on gekko; copies are on the PC. Do not run them: they fail open
on an invalid record. Drill (b) has a DTB but no pack yet. It loses a real core on
purpose; run it only with the owner present, after drill (a) passed.

## 3. Trials, PASS lines and failure signatures

Shared: at boot the probe prints exactly one of these lines.
- `C2 quarantine marker at 0x00000000ff607000: clear ... (seq N, ...)` (C2 allowed).
- `C2 quarantined (all): no valid record at ... fail closed, a new UNINIT record (seq N) was written`.
  This is expected on the first 6.12 boot after a cold boot, after round 0 (its v1 record
  is not valid in v2), or if 4.9 or sboot overwrote the page.
- `C2 quarantined (<all|LITTLE>): flags ... UNCLEAN ... / UNINIT ...`.

### 3.1 Drill (a), delayed-UFS (`c2-drill-lost-r1-20261107`)

Standard delayed-UFS run (READY only, run-trial.sh default 180 s). The drill DTB has no
trial token. Big C2 is armed at 30 s only if the record is already valid and clear (for
example after an S1a run that ended with a clean reboot); the drill does not need it. The
simulated loss of cpu2 comes at 60 s and the watchdog reset at about 90 s (30 s
timeout). The PID1 restart at 120 s must **not** be what ends the run.

PASS, in this order:

```
exynos9810-cpuidle: C2 quarantine marker at 0x00000000ff607000: clear ...    or    C2 quarantined (all): no valid record ... UNINIT ...
exynos9810-cpuidle: C2-LOST drill ENABLED: cpu2 is treated as lost at 60000 ms boot time (simulated, once: consumed in the record first); ...
exynos9810-cpuidle: C2-LOST detector running: ...
exynos9810-cpuidle: C2 boot arming of 4-7 at 30000 ms boot time            (only if clear)
exynos9810-cpuidle: C2 armed for 4-7 at <t> ms (MCT broadcast ready at <t> ms); ...   (only if clear)
exynos9810-cpuidle: C2-LOST drill: consumed (record seq N); treating cpu2 as lost at 600xx ms (simulated; the CPU is fine)
exynos9810-cpuidle: C2-LOST cpu2 (DRILL, simulated): no answer from C2 (drill): stage=kernel ...
exynos9810-cpuidle: C2-LOST: quarantine record written and read back at 0x00000000ff607000: seq=N+1 flags=... cpus=0x4 reason=drill losses=... session=<armed|idle>; next boot: C2 quarantined (<all|LITTLE>)
exynos9810-cpuidle: deep idle disabled, WFI only: C2-LOST drill
s3c2410-wdt: petting stopped for good (C2-LOST drill): SoC reset in about 30 s
exynos9810-cpuidle: C2-LOST: watchdog final stop: never reloaded again, SoC reset within 30 s
exynos9810-cpuidle: C2-LOST drill: waiting for the watchdog reset; orderly reboot only if none within 45 s
```

Then the log stops at about 90 s, and the phone returns to Lineage without a button
press: a watchdog reset is not a SWRESET, so the shim bounces it. On 4.9, check:
- `/proc/reset_reason`, or `sec_debug.rst_stat` bit 24 (CLUSTER0_WDTRESET) in
  `/proc/cmdline`;
- `/sys/fs/pstore/console-ramoops*` holds the C2-LOST lines.

Then restore RECOVERY 8763860c.

Optional second boot of the same RECOVERY (marker read back after a 4.9 boot): the probe
must log `C2 quarantined (all)` or `C2 quarantined (LITTLE)` with `drill` in the flags,
`UNCLEAN` if big C2 had been armed, and

```
C2-LOST drill: already run on an earlier boot (consumed, record seq N): skipped; rerun: echo 1 > .../c2_drill_rearm (root)
```

and **no** second drill. "no valid record" there means 4.9 or sboot overwrote
0xff607000: report it. A direct 6.12 → 6.12 restart, the real-BOOT case that the record
is for, does not pass through 4.9.

FAIL signatures:
- `C2-LOST drill: FAIL: no watchdog reset 45 s after petting stopped` (the watchdog did not
  fire, then orderly reboot);
- `C2-LOST: no guarded watchdog` or `final stop latched, but the counter does not run`;
- `C2-LOST drill: SKIPPED` or `refused` on the first run;
- `WRITE FAILED` anywhere in a C2 line;
- the drill firing on the second boot;
- the PID1 120/150 s restart line, or `C2-LOST: still running 75 s after the loss`;
- the phone not back in Lineage within about 3 min (manual reset needed).

### 3.2 S1a, ROM-RAM (`rom-ram/pack-romram-int6-c2-r1`)

The standard int6 ROM-RAM trial (integration-6.md section 5) plus the soak in
roadmap/c2-runtime-design.md 4.1: 15 min screen on, 30 min screen off (Wi-Fi associated),
15 min load. Take a cpuidle and /proc/interrupts snapshot every 60 s (section 4). All
int6 checks apply, except that no `wifi selftest: RESULT` line is expected any more; Wi-Fi
is checked via the HAL. End the run with the DONE frame (`done.flag`): that is a clean
restart and closes the session.

Boot 1 may log `no valid record ... UNINIT` (expected after a cold boot or after round
0). Then nothing is armed: end that run with DONE and boot the same pack again. On the
next boot the record is valid, and the trial token clears it once:

```
exynos9810-cpuidle: C2 quarantine: trial token 0xc251a001 (record has 0x0): one-shot clear of flags 0x10 (all) by the DT
exynos9810-cpuidle: C2 quarantine marker at 0x00000000ff607000: clear (trial token) (seq N, ...)
```

PASS:

```
exynos9810-cpuidle: C2 quarantine marker at 0x00000000ff607000: clear ...   (no "C2 quarantined")
exynos9810-cpuidle: C2-LOST detector running: ...
exynos9810-cpuidle: C2 boot arming of 4-7 at 45000 ms boot time
exynos9810-cpuidle: C2 armed for 4-7 at <t> ms (MCT broadcast ready at <t> ms); armed now 4-7, pending
exynos9810-cpuidle: c2 stats: armed=4-7 lost= deep_off=0 quarantine=none ... det_enq_fail=0 det_revalidated=... det_hp_unlocked=0 ...   (every 10 min)
exynos9810-cpuidle: c2 stats: cpu4 enter=... resumed=... early=... psci_fail=0 ...   (cpu4-7)
exynos9810-cpuidle: C2 session closed cleanly (restart)        (at the DONE reboot)
```

- Per CPU at each `c2 stats:`: resumed + early + psci_fail = enter, or enter - 1 for a CPU
  whose stage is `firmware` at that moment.
- psci_fail = 0 on the big cores.
- `cat /proc/sys/kernel/hardlockup_panic` is 1 and `cat /proc/sys/kernel/nmi_watchdog` is 1
  (buddy detector on).
- The final restart is normal and logs `C2 session closed cleanly`.
- Residency at screen off: big ≥ 85 % (4.9: 90-92 %).
- A further boot of the same pack logs `clear` without the token line (token already
  stored) and arms again.

FAIL signatures:
- `C2-LOST`;
- `C2 detector: cpuN silent for 200 ms in the firmware stage` (followed by a reboot);
- `deep idle disabled`;
- `Watchdog detected hard LOCKUP on cpu`;
- `rcu: INFO: rcu_preempt self-detected stall` or `detected stalls`;
- `C2 boot arming of cpuN refused`, `C2 quarantined` on the token boot;
- `the armed-session record failed its read-back`, `WRITE FAILED`;
- big psci_fail > 0;
- the `c2 stats:` lines stop;
- no `C2 session closed cleanly` at the DONE reboot.

## 4. Reading C2 residency over adb (compare with 4.9)

Read-only. Git Bash: `export MSYS_NO_PATHCONV=1`. Do not redirect into files on the device.
The cpuidle sysfs is the same on 4.9 and 6.12: state0 = WFI, state1 = C2. Residency = Δtime
(µs) / Δuptime; entries/s = Δusage / Δuptime.

```sh
adb shell uname -r                                   # 6.12.93-crownlte-diag+ (4.9: 4.9.337-...)
adb shell 'dumpsys power | grep -m1 mWakefulness'     # Asleep = screen off (>= 2 min before sampling)
adb shell 'for c in 0 1 2 3 4 5 6 7; do d=/sys/devices/system/cpu/cpu$c/cpuidle; for s in $d/state*; do echo "cpu$c $(basename $s) $(cat $s/name) disable=$(cat $s/disable) default=$(cat $s/default_status 2>/dev/null)"; done; done'
```

Two snapshots 60 s apart, computed on the PC:

```sh
snap() { adb shell 'cat /proc/uptime; for c in 0 1 2 3 4 5 6 7; do for s in /sys/devices/system/cpu/cpu$c/cpuidle/state*; do echo "cpu$c $(basename $s) $(cat $s/usage) $(cat $s/time)"; done; done' | tr -d '\r'; }
A=$(snap); sleep 60; B=$(snap)
paste <(echo "$A") <(echo "$B") | awk 'NR==1{dt=$3-$1; next} {printf "%s %s usage/s=%.1f residency=%.1f%%\n", $1, $2, ($7-$3)/dt, ($8-$4)/(dt*1e4)}'
```

Expected on S1a at screen off: cpu4-7 state1 residency ≥ 85 %, entries/s of the same order
as 4.9 (40-51/s per big core). cpu0-3 state1 stays at 0 (LITTLE is WFI only).

4.9 reference (design section 3, 4.9.337, screen off, USB powered):

| | C2 % | C2 entries/s |
|---|---|---|
| cpu4 / 5 / 6 / 7 | 90.3 / 90.3 / 91.5 / 91.8 | 51 / 44 / 40 / 46 |
| cpu0-3 | 84.3-84.5 | 79-109 |

Interrupts (6.12): `adb shell cat /proc/interrupts`. Look at the `mct-bc` line
(SPI 466), `IPI` "Timer broadcast interrupts" (one per C2 timer wake,
4.9 has 0) and the rescheduling and function-call IPIs. Twice, 60 s apart.

Root only (serial or a root shell), not needed for the comparison:
`cat /sys/bus/platform/devices/14070000.cpuidle/c2_stats` (and `c2_armed`, `c2_quarantine`).

## 5. Watchdog: one final stop for suspend and C2

The C2 chain and the suspend S0 driver share one API (`include/soc/samsung/crownlte-wdt.h`,
served only by the SBOOT-guarded instance): `s3c2410wdt_crownlte_ping()`,
`_set_deadline()`, `_state()` and the final stop

```c
int s3c2410wdt_crownlte_stop_petting(const char *why);
```

The rest of the API comes from the suspend-s0 watchdog commits (4800e0e24, 766bcf035,
7c0493496, 7b58093af, cherry-picked here unchanged; their document
`Documentation/crownlte/suspend-s0.md` lives on `crownlte-suspend-s0`). ea54a6b81 then
turns the final stop into one state:

- **One latched state**, `pet_stopped` + `stop_why`, set under `wdt->lock` by
  `__s3c2410wdt_final_stop()` (the public API, the restart notifier, the expiry self-test)
  or by `s3c2410wdt_may_pet()` once the deadline has passed. It is never cleared. The old
  `reboot_pending` and `selftest_stopped` stop flags are gone. C2 has no stop flag of
  its own.
- **Final stop:** under `wdt->lock` there is at most one last reload, and only while
  `may_pet()` allows it: a running counter, no deadline passed, no earlier final stop.
  Then the latch. The call is idempotent. It returns the timeout, or `-EIO` if the counter
  is not running with the reset enabled right now (the state is latched anyway), or
  `-ENODEV`.
- **Every path that would reload or weaken the counter checks the latch under
  `wdt->lock`:** keepalive/ping, `.start` (after the latch it never reloads: a counter
  that is off or runs without the reset is switched on with the reset enabled from its
  current WTCNT), `.stop` and `WDIOC_SETTIMEOUT` (refused once a deadline is installed or
  after the latch), the prescaler (never slowed after the latch), the kept-armed suspend
  (refused) and its resume (no reload), shutdown and the unbind actions (never mask or
  stop after the latch).
- **Plain suspend (B4):** the latch check, the register save and switching the counter
  off are one `wdt->lock` section (`pm_stopped`). The PMU reset mask is set only after
  that, while the counter is off. A final stop is therefore either seen there (suspend
  refused, `-EBUSY`) or it lands while the counter is off (`-EIO`, latched). Resume first
  unmasks the PMU reset, then restores under `wdt->lock`. After a final stop it writes
  WTCON only (reset on) and does **not** reload WTCNT, so the counter goes on from where it
  stopped. A failed PMU mask undoes the suspend.

## 6. Quarantine record format (v2)

Page `0xff607000`, 4 KB, no-map, mapped write-combining. Two slots of 14 little-endian
u32 words at byte offsets 0 (slot 0) and 2048 (slot 1).

| Word | Name | Meaning |
|---|---|---|
| 0 | magic | `0x51324339` |
| 1 | version | 2 (round 0 wrote version 1 at offset 0: invalid here) |
| 2 | slot | 0 or 1, must equal the slot the record sits in |
| 3 | seq | sequence number; `seq & 1 == slot` |
| 4 | flags | bit0 LITTLE lost, bit1 big lost, bit2 written by a drill, bit3 UNCLEAN (armed session found at boot), bit4 UNINIT (no valid record since the last clear); other bits invalid |
| 5 | session | `0x49444c45` IDLE or `0x41524d44` ARMED; anything else invalid |
| 6 | boots | probes of this kernel counted |
| 7 | cpus | lost CPU mask (bits 0-7) |
| 8 | uptime_ms | boot time of the last loss |
| 9 | reason | 0 none, 1 detector, 2 selftest, 3 drill, 4 unclean, 5 invalid |
| 10 | losses | losses recorded |
| 11 | drills | consumed drills: bit0 `c2-lost-drill`, bit1 `c2-loss-drill-ms`; other bits invalid |
| 12 | token | last `samsung,c2-quarantine-trial-token` applied (0: none) |
| 13 | crc | CRC32 (crc32_le(~0, words 0-12) ^ ~0) |

**Valid slot:** magic, version, slot field, seq parity, known session word, no unknown
flag or drill bits, and the CRC must all match. **Newest record:** if both slots are
valid, their seq must differ by exactly 1 and the higher one wins; any other difference
counts as no valid record. If one slot is valid, it wins.

**Write (`e9810_qm_commit()`, under `e9810_qm_lock`):**
1. Build the full new record in RAM with seq = newest + 1.
2. Store it word by word into slot `seq & 1`, which is never the slot of the newest record.
3. `dsb(sy)`, then read the slot back and compare it.

A reset in the middle of step 2 leaves a slot that fails its CRC, and the previous
record, in the other slot, is used. A failed read-back sets `write_failed`, and the
caller handles it: the boot write leaves all C2 off, a failed session write refuses
arming, and a failed drill-consumed write skips the drill.

**Transitions** (each one is a single commit):

| Event | Change | If interrupted, the next boot sees |
|---|---|---|
| probe, valid record | session ARMED → flags \|= UNCLEAN; token clear if the DT token differs; session = IDLE; boots++ | the old record (same verdict) |
| probe, no valid record | new record: flags = UNINIT, reason invalid | still no valid record (UNINIT again) |
| first arm of the boot | session = ARMED (read back before any CPU is armed) | old IDLE record, but nothing was armed |
| drill consumed | drills \|= bit, before the drill fires or arms | old record: drill not consumed, but it never fired |
| loss | flags \|= LITTLE/BIG (+DRILL), cpus, uptime, reason, losses++ (session stays ARMED) | the ARMED record: UNCLEAN, all C2 off |
| clean restart/power-off | session = IDLE, after cpuidle_pause() | the ARMED record: UNCLEAN (fail closed) |
| root `c2_quarantine_clear` | flags/cpus/uptime/reason = 0 (session, drills, token kept) | the quarantined record |
| root `c2_drill_rearm` | drills = 0 | drills still consumed |

So whenever any CPU is armed, the newest durable record is ARMED or newer. No
interrupted write can bring back a record that allows more than the one before it.

## 7. Fix round 1 (C2 Sol R1 on ce5267cee)

| # | Finding | Fix |
|---|---|---|
| 1 BLOCKER | `cpuidle-exynos9810.c:1669`: an invalid or corrupt record, or a reset during the in-place rewrite, became zero flags (C2 on); a stale CRC-valid clear record was accepted | v2 record (section 6): two sequenced slots, never written in place. No valid record means all C2 off (UNINIT). Only an explicit clear (root sysfs, or a one-shot DT trial token on a valid record) enables C2. Staleness: every armed boot persists ARMED first, so a record that allows C2 cannot be older than an arming |
| 2 BLOCKER | `:1932` and `crownlte_c2.fragment:9`: a buddy hard-lockup panic or a CPU0 wedge resets without any record, so a reboot loop is possible | armed-session record (section 1, item 5) persisted and read back before the first CPU is armed; cleared only by the reboot notifier after `cpuidle_pause()` on a clean restart or power-off, never after a loss. A leftover ARMED at boot means UNCLEAN, all C2 off. Nothing runs in panic context. The fragment comment is corrected |
| 3 MAJOR | `s3c2410_wdt.c:608`: `WDIOC_SETTIMEOUT` stretches the final countdown; B4 `:1329`: suspend can mask the reset after stop-petting began, and resume reloads | d9c0de8a9 reverted. Suspend-s0 watchdog commits cherry-picked. ea54a6b81 makes one latched final-stop state with the Sol-specified `s3c2410wdt_crownlte_stop_petting()`. Plain suspend/resume is serialized with it under `wdt->lock` (section 5) |
| 4 MAJOR | `:1853`: hotplug can make a healthy CPU look lost | enqueue failure ≠ unanswered delivery; `cpus_read_trylock()`, never an unbounded wait (lock-free scan after 5 busy scans); revalidation of online + firmware-entry generation after each wait |
| 5 MAJOR | `:1916`: drill (a) repeats despite quarantine | consumed bit persisted before the drill fires; root `c2_drill_rearm`. Drill (a) scope documented (section 1, item 7) |
| also | an offline requested CPU refused the whole boot mask and was never retried | per-CPU boot arming; offline/MCT retried for 60 s; an offline CPU is armed when it comes online |

## 8. Limits and open points (residual risks)

- **Trial token.** On a valid record, the token is an explicit clear that reaches past
  UNCLEAN and loss flags without root. It is only in the S1a and drill (b) trial DTBs. It
  fires once per token value, so a loss after it stays quarantined on the next boots of
  the same pack. A record that is wiped between boots (UNINIT) plus a stored token that
  differs gives one more armed boot every second boot. That is bounded by the shim (any
  non-SWRESET reset goes back to Lineage) and does not apply to a real BOOT kernel
  without the property.
- **First boot after a cold boot or after round 0** never arms C2 (UNINIT). That is the
  intended fail-closed cost. A real BOOT kernel needs root `c2_quarantine_clear` once
  per wipe of the page.
- **False UNCLEAN.** Any panic, watchdog reset or power loss while C2 was armed
  quarantines C2, even when C2 was not the cause (the suspend deadline, an unrelated
  oops). Root clear is needed then.
- **Page survival across 4.9 and sboot** is expected from the 4.9 snapshot layout, not
  measured. Drill (a)'s second boot measures it. If the page does not survive, every
  6.12 boot after 4.9 is UNINIT (safe, but C2 never arms in that flow).
- **Detector blind spots:** a CPU stuck in kernel code with IRQs off (stage
  `cal_cpu_disable`, `cal_cpu_enable`, `cpu_pm_exit`) is left to the buddy hard-lockup
  panic (12-24 s). After 5 busy hotplug scans the detector runs without the hotplug lock
  and relies on revalidation alone.
- **Watchdog:**
  - After a final stop latched during a plain suspend, the reset can only come after
    resume, because the counter is off while suspended. The keep-armed suspend
    (`samsung,keep-armed-in-suspend`) avoids that window and is not enabled in these DTBs.
  - `.start` after the latch can enable the reset from a stale WTCNT: the reset then
    comes within at most the full count (≤ max timeout).
  - The final stop keeps the current timeout instead of forcing 30 s as round 0 did.
    These DTBs set `timeout-sec = <30>`, so drill (a) still expects the reset at about
    90 s. A timeout that user space set before the loss (no deadline installed) applies
    as it is, at most the maximum timeout.
- **Selftest-only DTBs without the page** (`delayed-ufs-diag`) may still arm in self-test
  phases with no session record. They are diag only and run on the shim vehicle.
- The detector's ping cadence vs the C2 entry rate: compare the C2 entries/s of S1a with
  4.9 (design open point).
- Drill (b) and S1b (LITTLE cpu1-2) are not packed.
