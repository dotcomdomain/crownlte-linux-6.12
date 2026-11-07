# Runtime C2 on crownlte 6.12: S0 (self-recovery) and S1a (big C2)

Branch `crownlte-c2-runtime` on `crownlte-integration-6` 4148bf525. Design:
`roadmap/c2-runtime-design.md` (gekko), sections 4.0 (S0) and 4.1 (S1a, C1).
Build only. Nothing here has run on the phone yet. Fix rounds 1 and 2 (C2 Sol R1, R2)
are in sections 7 and 8, the record format (v3) in section 6, the recovery procedure
(how to clear a quarantine) in section 9.

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
| 8e5400c3e | this document (round 1) |
| 3bb7ff989 | cpuidle: acknowledged C2 drain at shutdown, irreversible shutdown gate, SHUTDOWN session closed at the last controlled point, record v3 (trusted only with both slots chained), monotonic trial floor |
| 47d657b15 | watchdog: latch-aware guarded `.restart`, one reload in the expiry self-test |
| ae5fc0401 | DT: round-2 trial tokens 0xc2c20001 (S1a), 0xc2c20002 (drill b) |

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
   (`exynos9810-crownlte-c2.dtsi`, `memory-region` of the cpuidle node), format v3 in
   section 6. RAM only: no UFS or eMMC write, and no other storage.
   - Placement: this is the last page of the 4.9 exynos-snapshot carve-out
     (0xfda00000..0xff608000, already no-map here as `snapshot-after-pstore`, now 4 KB
     shorter). It is **not** right after ramoops: 4.9 writes `log_kevents` at
     0xfed18000..0xff518000. 4.9 places no snapshot item in 0xff518000..0xff608000.
   - **Fail closed.** At probe, before anything can be armed:
     - the record is **trusted** only if both slots are valid and exactly one sequence
       number apart (section 6);
     - no valid slot (cold boot, another kernel, garbage, a round 0/1 v1/v2 record) →
       **all C2 off** (`UNINIT`, reason `invalid`), and a fresh record is written into
       both slots;
     - one valid slot, or two that do not chain → **all C2 off** (`UNINIT`, reason
       `ambiguous`): a lone clean slot never allows C2, because its newer successor
       (maybe ARMED or quarantined) may have been lost. Whatever survives is kept: loss
       flags, `UNCLEAN` for a session that is not IDLE, lost CPUs, consumed drills, the
       trial floor. The record is rewritten into both slots;
     - a session that is still `ARMED` or `SHUTDOWN` → **all C2 off** (`UNCLEAN`, reason
       `unclean` or `shutdown`): the boot that armed C2 did not reach the last controlled
       point of a restart, halt or power-off (item 5);
     - a big loss → all C2 off; a LITTLE loss → LITTLE C2 off, big allowed;
     - only a trusted record with none of these flags allows C2.
   - **Explicit clear only** (procedure: section 9):
     - root: `echo 1 > /sys/bus/platform/devices/14070000.cpuidle/c2_quarantine_clear`,
       then a clean `reboot`. It takes effect on the next boot;
     - or a trial DTB's `samsung,c2-quarantine-trial-token = <N>`: one clear at probe of a
       **trusted** record whose trial floor is below N. N becomes the floor. Tokens form
       one increasing sequence over all trial DTBs, so a used or superseded token never
       clears again (A, B, A clears at most twice, never three times). Without a trusted
       record the token clears nothing and only becomes the floor: a new DTB with a higher
       token (or root) is needed then. ROM-RAM has no root shell, hence the token.
   - **Without the page, boot arming and both drills are refused.** Only the DT-gated
     self-test may still arm (diag DTBs without the page; it watches its own phases).
5. **Armed session (B2).** Before the first CPU is armed, the record's session word is
   set to `ARMED` and read back. Arming fails if that write fails. A shutdown closes it
   in two steps:
   1. **Reboot notifier** (priority INT_MAX - 1, right after the watchdog's restart
      guard). Under `e9810_arm_lock` it sets the irreversible `e9810_shutdown` flag,
      disables C2 on every CPU and clears the arm mask. Every arming path, the C2 entry
      gate and the detector check the flag. reboot.c runs the notifiers *before* it sets
      `system_state`, so `system_state` alone does not stop a pending arming work. Then
      the **drain**: each C2 entry sets a per-CPU busy word before it reads the gate,
      with a full barrier in between; the notifier issues the same barrier after closing
      the gate and waits, at most 1 s while kicking the idle CPUs, until it has seen
      busy = 0 on every possible CPU. A 0 is a positive acknowledgement: that CPU is out
      of the C2 path, and every later entry sees the closed gate. `cpuidle_pause()` is no
      longer used: its RCU grace period does not wait for a CPU stuck in firmware (that
      CPU is in an RCU idle section). If any CPU does not acknowledge, or a loss was
      recorded, the record is left as it is (`ARMED`: next boot `UNCLEAN`). Otherwise
      `ARMED` → `SHUTDOWN`, which the next boot still treats as `UNCLEAN`.
   2. **Last controlled point**: `SHUTDOWN` → `IDLE`.
      - restart: a restart handler at priority INT_MAX in `machine_restart()` (IRQs
        off, the other CPUs stopped by `smp_send_stop()`), before every handler that
        resets (the crownlte diagnostic restart guard at 255 writes SWRESET and never
        returns, then the watchdog at 128). It closes only if syscore shutdown of an
        orderly `kernel_restart()` ran first, `system_state` is `SYSTEM_RESTART`, and no
        panic (`panic_cpu`), oops, loss or kexec is in progress. It only try-locks the
        record lock; if busy, the record stays `SHUTDOWN`. It prints its line before the
        write, and after it only on failure;
      - halt and power-off: the syscore shutdown hook (no power-off handler is
        registered: that would change `kernel_can_power_off()`);
      - **kexec is unclean by definition**: `kernel_kexec()` runs the notifier
        (`SHUTDOWN`) and syscore shutdown, but no restart handler, so the next kernel sees
        `SHUTDOWN` (`UNCLEAN`). `CONFIG_KEXEC` is off in this config anyway.

   Paths that reach IDLE: `kernel_restart()` (reboot(2) `RB_AUTOBOOT`, `orderly_reboot()`,
   Android `reboot`, the diag PID1 restart at 120 s and its 150 s fallback, the ROM-RAM
   DONE reboot), `kernel_power_off()`, `kernel_halt()`. The next boot is `UNCLEAN` after:
   panic (buddy hard-lockup, any oops with `panic_on_oops`) at any stage, including in a
   later reboot notifier, `device_shutdown()` or syscore shutdown; `emergency_restart()`;
   a watchdog reset (CPU0 wedge, C2-LOST, a hang anywhere in the shutdown path); a PMIC
   reset; power loss; kexec. Nothing arms once a restart or power-off has begun.

   **Unavoidable final window.** After the IDLE write, still in the restart path: the
   remaining restart handlers (diagnostic SWRESET, PSCI, watchdog) and, if all of them
   fail, the arm64 "Reboot failed -- System halted" loop, which ends in the watchdog
   reset (final stop latched by the watchdog's restart notifier, 30 s). For halt and
   power-off: `kmsg_dump()`, `machine_halt()`/`machine_power_off()` and its handlers. A
   failure there leaves a clean record. Every CPU had acknowledged leaving C2 and could
   not enter it again since the drain, so such a failure cannot come from C2.
6. **Observability**, root-only files in `/sys/bus/platform/devices/14070000.cpuidle/`:
   - `c2_armed`, `c2_lost`, `c2_stats` (per CPU: stage, fw_gen, enter, resumed, early,
     psci_fail, unarmed, guarded, wfi_fallback, stale_grp2, last_err; then deep_off,
     det_scans, det_pings, det_enq_fail, det_revalidated, det_hp_unlocked, boot_pending),
     `c2_quarantine` (the newest record with `session=idle|armed|shutdown`,
     `trial_floor`, both slots' validity and sequence, `trusted_at_boot`, the flags in
     force);
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
  `samsung,c2-quarantine-trial-token = <0xc2c20001>` (round 2; round 1 had 0xc251a001,
  now retired). LITTLE stays WFI.
- **int6 DT change:** `samsung,wifi-selftest` and `-ms` were removed from
  `exynos9810-crownlte-romram-wifi.dtsi` (Sol finding on int6). This changes the int6 DTB
  too: 297eb4f4 → c30c8d04, and the decompiled diff is exactly those two properties.

## 2. Artifacts (gekko `/mnt/ssd/host-builds/kernel-upgrade`)

Fix round 2, `crownlte-c2-runtime` ae5fc0401 (code 3bb7ff989 + 47d657b15, DT ae5fc0401).
Config `panel-bright-work/ko-c2-ae5f/config` = int6 config (c7b917d1) +
`crownlte_c2.fragment`, `olddefconfig`, unchanged since round 0 (668b03c4). It adds only
`HARDLOCKUP_DETECTOR`, `_BUDDY`, `_COUNTS_HRTIMER`, `BOOTPARAM_HARDLOCKUP_PANIC`,
`LOCKUP_DETECTOR` and `SOFTLOCKUP_DETECTOR`. `CONFIG_KEXEC` is not set. Banner:
`Linux version 6.12.93-crownlte-diag+`. W=1 clean for `s3c2410_wdt.o` and
`cpuidle-exynos9810.o`.

| File (`panel-bright-work/ko-c2-ae5f/`) | sha256 |
|---|---|
| `arch/arm64/boot/Image` | f760d082b103238ac5dfd912428922fbb717a774aa43644f37f44f4f4177022f |
| `.../exynos9810-crownlte-romram-v23a-int6-c2.dtb` (S1a ROM-RAM, token 0xc2c20001) | 283b1243639fd1866ff6f4a9a1c3ba7a55f3aade5291904efd93213109772d7c |
| `.../exynos9810-crownlte-romram-v23a-int6.dtb` (int6 minus Wi-Fi self-test, unchanged) | c30c8d048dc10eb2ad841e6748b7063be9dc3a8ccbdb8c29428f0fbe8f4c24ee |
| `.../exynos9810-crownlte-delayed-ufs-c2-drill-lost-diag.dtb` (drill a, unchanged, no token) | 6488152a30e86eccaf392ae7749bd691ce87e05ef96da2413202860c41ec53c2 |
| `.../exynos9810-crownlte-delayed-ufs-c2-drill-loss-diag.dtb` (drill b, token 0xc2c20002) | 83cfd17418bc03ee31830c8d4f29a27b367591dae44ef724e196a3d129d7abf2 |
| `config` | 668b03c456c204e0473e883e5a31e0edd1c90349dd52537fd79ba478f16ffa7c |

The decompiled S1a and drill (b) DTBs differ from round 1 (2bc3c73c, 1775625f) only in
the token value.

Packs (shim bbea959, parent a8198be0, restore RECOVERY 8763860c), built in /dev/shm,
shim worktrees removed. Each pack dir has a PROVENANCE.txt:

| Pack | recovery.img sha256 | Inputs |
|---|---|---|
| `c2-drill-lost-r2-20261107` (delayed-UFS, drill a) | 3df772d07ac2fa006adae287e59c659494a6a6c698c350728948ddc543669c49 | `pack-acpm-q-paths.py`, Image f760d082, DTB 6488152a, ramdisk 506232b1, loader 31084318. The manifest `purpose`/`kernel_source` are packer defaults; see PROVENANCE.txt |
| `rom-ram/pack-romram-int6-c2-r2` (ROM-RAM S1a) | 51a7699dadb3ac31944978f0978b7738bd55247c0f50cf7340150a0992d6f6cd | `rom-ram/pack-rom-ram.py`, KO `panel-bright-work/ko-c2-ae5f`, DTB 283b1243, ramdisk `rom-ram/build-32` 82771399 (romram-adb 46a3670, ADB=1 ADB_ROOT=1, EFS_RO_DIR), loader f85ea602 |

Deleted on gekko (copies on the PC). **Do not run them**: they use record v2 and the
round-1 shutdown and token rules, and v3 treats their records as invalid anyway. Round 1:
`c2-drill-lost-r1-20261107` aea3534a, `rom-ram/pack-romram-int6-c2-r1` f655965c. Round 0:
`c2-drill-lost-20261107` 57715ff5, `rom-ram/pack-romram-int6-c2` 611cbbb0 (round 0 also
fails open on an invalid record). Drill (b) has a DTB but no pack yet. It loses a real
core on purpose; run it only with the owner present, after drill (a) passed.

## 3. Trials, PASS lines and failure signatures

Shared: at boot the probe prints exactly one of these lines.
- `C2 quarantine marker at 0x00000000ff607000: clear ... (seq N, ..., trial floor F)` (C2
  allowed).
- `C2 quarantined (all): no valid record at ... fail closed, a new UNINIT record (seq N, both slots) was written`.
  Expected on the first 6.12 boot after a cold boot, after any round 0/1 kernel (v1/v2
  records are invalid in v3), or if 4.9 or sboot overwrote the page.
- `C2 quarantined (all): ambiguous record at ... (only one valid slot: its successor may be lost | both slots valid but not one sequence number apart)`:
  report it (a torn write or a damaged page).
- `C2 quarantined (<all|LITTLE>): flags ... UNCLEAN ... / UNINIT ...` (reason `unclean`:
  the session was still ARMED; `shutdown`: it was drained but the last controlled point
  was not reached).

At a clean restart of a boot that had armed C2 (S1a DONE reboot, the PID1 restart if
big C2 was armed):

```
exynos9810-cpuidle: C2 session drained (restart): every CPU acknowledged leaving C2; record SHUTDOWN (seq N), closed only at the final restart handler point
exynos9810-cpuidle: C2 session: final restart point: closing the record (SHUTDOWN -> IDLE)
```

The second line comes after `reboot: Restarting system`. Nothing is printed after the
write unless it fails (`C2 session NOT closed at the final restart point`). A boot that
never armed C2 prints neither line; its record is IDLE already.

### 3.1 Drill (a), delayed-UFS (`c2-drill-lost-r2-20261107`)

Standard delayed-UFS run (READY only, run-trial.sh default 180 s). The drill DTB has no
trial token. Big C2 is armed at 30 s only if the record is already trusted and clear (for
example after an S1a run that ended with a clean reboot); the drill does not need it. The
simulated loss of cpu2 comes at 60 s and the watchdog reset at about 90 s (30 s
timeout). The PID1 restart at 120 s must **not** be what ends the run.

PASS, in this order:

```
exynos9810-cpuidle: C2 quarantine marker at 0x00000000ff607000: clear ...    or    C2 quarantined (all): no valid record ... UNINIT ... (seq N, both slots) ...
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
must log `C2 quarantined (all)`, with `drill` in the flags (plus `UNINIT` if the first
boot found no record, the LITTLE flag for cpu2, and `UNCLEAN` if big C2 had been armed),
and

```
C2-LOST drill: already run on an earlier boot (consumed, record seq N): skipped; rerun: echo 1 > .../c2_drill_rearm (root)
```

and **no** second drill. "no valid record" there means 4.9 or sboot overwrote
0xff607000; "ambiguous" means one slot was damaged. Report either. A direct 6.12 → 6.12
restart, the real-BOOT case that the record is for, does not pass through 4.9.

A drill (a) run also leaves the trusted record that the S1a token needs (section 3.2).

FAIL signatures:
- `C2-LOST drill: FAIL: no watchdog reset 45 s after petting stopped` (the watchdog did not
  fire, then orderly reboot);
- `C2-LOST: no guarded watchdog` or `final stop latched, but the counter does not run`;
- `C2-LOST drill: SKIPPED` or `refused` on the first run;
- `WRITE FAILED` anywhere in a C2 line;
- the drill firing on the second boot;
- the PID1 120/150 s restart line, or `C2-LOST: still running 75 s after the loss`;
- the phone not back in Lineage within about 3 min (manual reset needed).

### 3.2 S1a, ROM-RAM (`rom-ram/pack-romram-int6-c2-r2`)

The standard int6 ROM-RAM trial (integration-6.md section 5) plus the soak in
roadmap/c2-runtime-design.md 4.1: 15 min screen on, 30 min screen off (Wi-Fi associated),
15 min load. Take a cpuidle and /proc/interrupts snapshot every 60 s (section 4). All
int6 checks apply, except that no `wifi selftest: RESULT` line is expected any more; Wi-Fi
is checked via the HAL. End the run with the DONE frame (`done.flag`): that is a clean
restart and closes the session.

**Order.** The token 0xc2c20001 clears only a **trusted** record. Boot this pack right
after a 6.12 boot of this round that left one, with no cold boot in between: for example
the drill (a) r2 run. Its record survives the 4.9 bounce only if the page survives,
which the drill's optional second boot measures. If the S1a boot itself finds no
trusted record, it logs

```
exynos9810-cpuidle: C2 quarantine: trial token 0xc2c20001 not applied: no trusted record, so its earlier use cannot be ruled out; it is now the trial floor (...)
exynos9810-cpuidle: C2 quarantined (all): no valid record ...    (or: ambiguous record ...)
```

and never arms. The token is then spent: every further boot of this pack logs
`trial token 0xc2c20001 not above the trial floor 0xc2c20001 (already used or superseded): ignored`.
To go on, you need root (section 9) or a new S1a DTB with a token above 0xc2c20002
(the drill (b) token).

Token boot (trusted record, floor below 0xc2c20001):

```
exynos9810-cpuidle: C2 quarantine: trial token 0xc2c20001 (floor 0x0): one-shot clear of flags 0x... (all) by the DT
exynos9810-cpuidle: C2 quarantine marker at 0x00000000ff607000: clear (trial token) (seq N, ..., trial floor 0xc2c20001)
```

PASS:

```
exynos9810-cpuidle: C2 quarantine marker at 0x00000000ff607000: clear ...   (no "C2 quarantined")
exynos9810-cpuidle: C2-LOST detector running: ...
exynos9810-cpuidle: C2 boot arming of 4-7 at 45000 ms boot time
exynos9810-cpuidle: C2 armed for 4-7 at <t> ms (MCT broadcast ready at <t> ms); armed now 4-7, pending
exynos9810-cpuidle: c2 stats: armed=4-7 lost= deep_off=0 quarantine=none ... det_enq_fail=0 det_revalidated=... det_hp_unlocked=0 ...   (every 10 min)
exynos9810-cpuidle: c2 stats: cpu4 enter=... resumed=... early=... psci_fail=0 ...   (cpu4-7)
exynos9810-cpuidle: C2 session drained (restart): every CPU acknowledged leaving C2; record SHUTDOWN (seq N), closed only at the final restart handler point   (DONE reboot)
exynos9810-cpuidle: C2 session: final restart point: closing the record (SHUTDOWN -> IDLE)   (after "reboot: Restarting system")
```

- Per CPU at each `c2 stats:`: resumed + early + psci_fail = enter, or enter - 1 for a CPU
  whose stage is `firmware` at that moment.
- psci_fail = 0 on the big cores.
- `cat /proc/sys/kernel/hardlockup_panic` is 1 and `cat /proc/sys/kernel/nmi_watchdog` is 1
  (buddy detector on).
- The final restart is normal and logs both session lines above.
- Residency at screen off: big ≥ 85 % (4.9: 90-92 %).
- A further boot of the same pack logs `clear`, after the line `trial token 0xc2c20001 not
  above the trial floor 0xc2c20001 ... ignored` (there is nothing to clear: the DONE
  reboot closed the session), and arms again. This boot is what proves the clean close
  and that the page is kept across a 6.12 → shim → 6.12 restart.

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
- `C2 session NOT closed` at the DONE reboot (a CPU did not acknowledge leaving C2, or
  the final write failed), or no `C2 session drained` line;
- `C2 quarantined ... UNCLEAN` (reason `shutdown` or `unclean`) on the boot after a DONE
  reboot.

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

### 5.1 The shared API (`include/soc/samsung/crownlte-wdt.h`)

The C2 chain and the suspend S0 driver (`drivers/soc/samsung/exynos9810-pm.c` on
`crownlte-suspend-s0`) share one header. Round 2 does not change it, and it differs from
the header on `crownlte-suspend-s0` 60774ac87 only in its comment (the declarations and
the struct are identical). Only the SBOOT-guarded instance (`samsung,exynos9810-wdt`,
`QUIRK_SBOOT_RESET_GUARD`) serves it, and only if it was running at probe. Without it
every call returns `-ENODEV` (or `false`). Every call is safe from any context
(hardirq, syscore, IRQs off) and never sleeps.

```c
struct crownlte_wdt_state {
	u32 wtcon, wtdat, wtcnt;
	unsigned int timeout;		/* seconds, current setting */
	bool enabled, reset;		/* WTCON enable / reset enable */
	bool keep_armed_suspend;	/* DT samsung,keep-armed-in-suspend */
	bool pet_stopped;		/* no longer petted (restart/selftest/stop/deadline) */
	u64 deadline_ns;		/* CLOCK_BOOTTIME reload deadline, 0: none */
};

int  s3c2410wdt_crownlte_ping(void);
int  s3c2410wdt_crownlte_stop_petting(const char *why);
int  s3c2410wdt_crownlte_set_deadline(u64 boot_ns);
bool s3c2410wdt_crownlte_state(struct crownlte_wdt_state *st);
```

| Call | Effect | Returns |
|---|---|---|
| `_ping()` | one reload under `wdt->lock`, only while `may_pet()` allows it | 0; `-ETIME` latched or deadline passed (no reload); `-EIO` counter off; `-ENODEV` |
| `_stop_petting(why)` | **the** final stop: under `wdt->lock` at most one last reload (only while `may_pet()` allows it and the counter runs), then the latch (`pet_stopped`, first `why` kept). Irreversible, idempotent | the timeout (s) if the counter runs with the reset enabled; `-EIO` otherwise (latched anyway; a plain suspend's resume re-arms without a reload); `-ENODEV` |
| `_set_deadline(boot_ns)` | CLOCK_BOOTTIME ns after which `may_pet()` latches by itself, on any CPU; only moves earlier; once set, user space cannot stop the watchdog or change its timeout | 0; `-EINVAL` 0 ns; `-EIO` not running with reset; `-ETIME` already latched; `-ENODEV` |
| `_state(st)` | WTCON/WTDAT/WTCNT snapshot; evaluates the deadline like the ping path | `false` without the instance |

C2 uses `_stop_petting()` only (C2-LOST). Suspend S0 uses all four.

### 5.2 Internal rules (the latch)

The rest comes from the suspend-s0 watchdog commits (4800e0e24, 766bcf035, 7c0493496,
7b58093af, cherry-picked here unchanged; their document
`Documentation/crownlte/suspend-s0.md` lives on `crownlte-suspend-s0`). ea54a6b81,
fbcc1d167 and 47d657b15 come on top, and integrating `crownlte-suspend-s0` means
picking these three:

- **One latched state**, `pet_stopped` + `stop_why`, set under `wdt->lock` by
  `__s3c2410wdt_final_stop()` (the public API, the restart notifier, the expiry
  self-test), by the guarded `.restart` handler, or by `s3c2410wdt_may_pet()` once the
  deadline has passed. It is never cleared. C2 has no stop flag of its own.
- **Every path that would reload or weaken the counter checks the latch under
  `wdt->lock`:** keepalive/ping, `.start` (after the latch it never reloads: a counter
  that is off or runs without the reset is switched on with the reset enabled from its
  current WTCNT), `.stop` and `WDIOC_SETTIMEOUT` (refused once a deadline is installed or
  after the latch), the prescaler (never slowed after the latch), the kept-armed suspend
  (refused) and its resume (no reload), shutdown and the unbind actions (never mask or
  stop after the latch).
- **Restart notifier** (INT_MAX, `SYS_RESTART` only, fbcc1d167): one `wdt->lock`
  section, the 30 s timeout without a reload, then the final stop. Exactly one reload.
- **Expiry self-test** (`samsung,selftest-stop-petting-ms`, 47d657b15): the same way,
  the short timeout without a reload, then the final stop. Exactly one reload (round 1
  reloaded twice).
- **Guarded `.restart`** (47d657b15): the watchdog core calls it from the restart
  handler chain (priority 128), IRQs off. It takes `wdt->lock` with a bounded trylock
  (100 ms; a CPU stopped by `smp_send_stop()` may hold it, and the same writes are made
  without the lock then). It latches the final stop without a reload, lowers WTCNT to at
  most 0x80 ticks (loads 0x80 if the counter is off), never touches the prescaler or
  WTDAT, and switches counter and reset on if they are off. A final countdown that is
  already running can only get shorter. Round 1 (upstream) wrote WTCON = 0, a new count
  and a new prescaler without the lock. Non-guarded variants keep the upstream
  sequence.
- **Plain suspend (B4):** the latch check, the register save and switching the counter
  off are one `wdt->lock` section (`pm_stopped`). The PMU reset mask is set only after
  that, while the counter is off. A final stop is therefore either seen there (suspend
  refused, `-EBUSY`) or it lands while the counter is off (`-EIO`, latched). Resume first
  unmasks the PMU reset, then restores under `wdt->lock`. After a final stop it writes
  WTCON only (reset on) and does **not** reload WTCNT, so the counter goes on from where it
  stopped. A failed PMU mask undoes the suspend.

## 6. Quarantine record format (v3)

Page `0xff607000`, 4 KB, no-map, mapped write-combining. Two slots of 14 little-endian
u32 words at byte offsets 0 (slot 0) and 2048 (slot 1). v3 has the same layout as v2.
The version, the SHUTDOWN session word, the reasons 6/7 and the meaning of word 12 are
new; a v1/v2 record is invalid.

**v4 (crownlte-c2-s2 from ec753284a, c2-s2.md section 8).** Same 14-word layout. A
v4 kernel still reads v3 slots (v3 and v4 slots chain, so the phone's trusted v3
record stays trusted), and every write is v4. A v3 kernel treats a v4 slot as invalid
and fails closed. New in v4: session words `0x4c41524d` LT_ARMED (MCT local timers
may run CPU ticks; covers an armed C2 session too) and `0x4c534844` LT_SHUTDOWN (its
drained form), flags bit5 LTIMER (no local-timer takeover, LITTLE C2 off) and reason
8 `ltimer`. A session found LT_ARMED or LT_SHUTDOWN at boot adds LTIMER to UNCLEAN.
The tables below describe v3; the v4 transitions are in c2-s2.md section 8.

| Word | Name | Meaning |
|---|---|---|
| 0 | magic | `0x51324339` |
| 1 | version | 3 (round 0 wrote 1, round 1 wrote 2: both invalid here) |
| 2 | slot | 0 or 1, must equal the slot the record sits in |
| 3 | seq | sequence number; `seq & 1 == slot` |
| 4 | flags | bit0 LITTLE lost, bit1 big lost, bit2 written by a drill, bit3 UNCLEAN (a session that was not IDLE found at boot), bit4 UNINIT (no trusted record since the last clear); other bits invalid |
| 5 | session | `0x49444c45` IDLE, `0x41524d44` ARMED, `0x5348444e` SHUTDOWN (drained at the start of a shutdown, last controlled point not reached); anything else invalid |
| 6 | boots | probes of this kernel counted |
| 7 | cpus | lost CPU mask (bits 0-7) |
| 8 | uptime_ms | boot time of the last loss |
| 9 | reason | 0 none, 1 detector, 2 selftest, 3 drill, 4 unclean, 5 invalid, 6 shutdown, 7 ambiguous; anything else invalid |
| 10 | losses | losses recorded |
| 11 | drills | consumed drills: bit0 `c2-lost-drill`, bit1 `c2-loss-drill-ms`; other bits invalid |
| 12 | trial_floor | highest trial token used or spent (0: none) |
| 13 | crc | CRC32 (crc32_le(~0, words 0-12) ^ ~0) |

**Valid slot:** magic, version, slot field, seq parity, known session word, no unknown
flag or drill bits, a known reason, and the CRC must all match.

**Trusted record:** both slots valid and their seq exactly 1 apart; the higher one is
the record. Every write keeps both slots valid (the newest and the one before it), and
a fresh or rebuilt record is written twice, so a trusted record stays trusted.
Anything else is not trusted:

| Slots at boot | Verdict | Written |
|---|---|---|
| both valid, 1 apart | trusted: the newer one | one commit |
| one valid | **ambiguous** (its successor may have been ARMED or quarantined) | merge + UNINIT, reason `ambiguous`, two commits |
| both valid, not 1 apart | **ambiguous** | merge + UNINIT, reason `ambiguous`, two commits |
| none valid | no record | zeroes + UNINIT, reason `invalid`, two commits |

The merge keeps everything that can only make the verdict stricter: the OR of the flags,
UNCLEAN for any slot whose session is not IDLE, the OR of lost CPUs and consumed drills,
and the maximum of boots, losses, uptime and trial floor. So a lone clean slot never
allows C2, and a surviving ARMED, SHUTDOWN or quarantined slot keeps its quarantine (and
adds UNINIT).

**Trial floor.** A DT token N clears a **trusted** record only if N > floor and N >
0xc2b10001 (rounds 0/1 retired). N then becomes the floor. Tokens are one increasing
sequence over all trial DTBs, so a token that was used, or superseded by a later one,
never clears again (alternating A, B, A cannot clear a third time). Without a trusted
record the token clears nothing; if it is above the floor it becomes the floor ("spent"),
because its earlier use cannot be ruled out. A clear then needs renewed authorization: a
new DTB with a higher token, or root. Issued tokens:

| Token | DTB | State |
|---|---|---|
| 0xc251a001 | S1a round 0/1 | retired |
| 0xc2b10001 | drill (b) round 1 | retired |
| 0xc2c20001 | S1a round 2 (`romram-v23a-int6-c2`) | current |
| 0xc2c20002 | drill (b) round 2 (`delayed-ufs-c2-drill-loss-diag`) | current |
| 0xc2c20003 | S2 round 0 (`romram-v23a-int6-c2s2`, crownlte-c2-s2) | retired, never booted |
| 0xc2c20004 | S2 fix round 1 (`romram-v23a-int6-c2s2r1`, crownlte-c2-s2) | current |

The next token must be above 0xc2c20004.

**Write (`e9810_qm_commit()`, under `e9810_qm_lock`):**
1. Build the full new record in RAM with seq = newest + 1.
2. Store it word by word into slot `seq & 1`, which is never the slot of the newest record.
3. `dsb(sy)`, then read the slot back and compare it.

A reset in the middle of step 2 leaves a slot that fails its CRC. The previous record,
in the other slot, then survives alone: **ambiguous**, all C2 off (round 1 used it as
the record). A failed read-back sets `write_failed`, and the caller handles it: the boot
write leaves all C2 off, a failed session write refuses arming, a failed drill-consumed
write skips the drill, and a failed SHUTDOWN or IDLE write leaves the older (stricter)
session.

**Transitions** (each one a single commit unless noted):

| Event | Change | If interrupted, the next boot sees |
|---|---|---|
| probe, trusted record | session not IDLE → flags \|= UNCLEAN (reason unclean/shutdown); token above the floor → clear, floor = token; session = IDLE; boots++ | the newest record alone: ambiguous, all C2 off |
| probe, ambiguous or no record | merged (or zero) record + UNINIT, token folded into the floor; two commits | still ambiguous or none: all C2 off |
| first arm of the boot | session = ARMED (read back before any CPU is armed) | IDLE alone: ambiguous; nothing was armed |
| drill consumed | drills \|= bit, before the drill fires or arms | old record (drill not consumed, but it never fired) or ambiguous |
| loss | flags \|= LITTLE/BIG (+DRILL), cpus, uptime, reason, losses++ (session stays ARMED) | the ARMED record alone: ambiguous + UNCLEAN |
| shutdown drain, all CPUs acknowledged | ARMED → SHUTDOWN (reboot notifier) | ARMED alone: ambiguous + UNCLEAN |
| last controlled point | SHUTDOWN → IDLE (restart handler; syscore shutdown for halt/power-off) | SHUTDOWN alone: ambiguous + UNCLEAN |
| root `c2_quarantine_clear` | flags/cpus/uptime/reason = 0 (session, drills, floor kept) | the quarantined record, or ambiguous |
| root `c2_drill_rearm` | drills = 0 | drills still consumed, or ambiguous |

So whenever any CPU is armed, the newest durable record is ARMED or newer, and no
interrupted write, lost slot or replayed token can bring back a record that allows
more than the one before it.

## 7. Fix round 1 (C2 Sol R1 on ce5267cee)

Kept for the history. Round 2 (section 8) replaced the v2 record with v3, the session
close after `cpuidle_pause()` with the acknowledged drain and the late close, and the
"token differs" rule with the trial floor.

| # | Finding | Fix |
|---|---|---|
| 1 BLOCKER | `cpuidle-exynos9810.c:1669`: an invalid or corrupt record, or a reset during the in-place rewrite, became zero flags (C2 on); a stale CRC-valid clear record was accepted | v2 record (section 6): two sequenced slots, never written in place. No valid record means all C2 off (UNINIT). Only an explicit clear (root sysfs, or a one-shot DT trial token on a valid record) enables C2. Staleness: every armed boot persists ARMED first, so a record that allows C2 cannot be older than an arming |
| 2 BLOCKER | `:1932` and `crownlte_c2.fragment:9`: a buddy hard-lockup panic or a CPU0 wedge resets without any record, so a reboot loop is possible | armed-session record (section 1, item 5) persisted and read back before the first CPU is armed; cleared only by the reboot notifier after `cpuidle_pause()` on a clean restart or power-off, never after a loss. A leftover ARMED at boot means UNCLEAN, all C2 off. Nothing runs in panic context. The fragment comment is corrected |
| 3 MAJOR | `s3c2410_wdt.c:608`: `WDIOC_SETTIMEOUT` stretches the final countdown; B4 `:1329`: suspend can mask the reset after stop-petting began, and resume reloads | d9c0de8a9 reverted. Suspend-s0 watchdog commits cherry-picked. ea54a6b81 makes one latched final-stop state with the Sol-specified `s3c2410wdt_crownlte_stop_petting()`. Plain suspend/resume is serialized with it under `wdt->lock` (section 5) |
| 4 MAJOR | `:1853`: hotplug can make a healthy CPU look lost | enqueue failure ≠ unanswered delivery; `cpus_read_trylock()`, never an unbounded wait (lock-free scan after 5 busy scans); revalidation of online + firmware-entry generation after each wait |
| 5 MAJOR | `:1916`: drill (a) repeats despite quarantine | consumed bit persisted before the drill fires; root `c2_drill_rearm`. Drill (a) scope documented (section 1, item 7) |
| also | an offline requested CPU refused the whole boot mask and was never retried | per-CPU boot arming; offline/MCT retried for 60 s; an offline CPU is armed when it comes online |

## 8. Fix round 2 (C2 Sol R2 on 8e5400c3e)

Line numbers are at 8e5400c3e.

| # | Finding | Fix |
|---|---|---|
| 1 HIGH | `cpuidle-exynos9810.c:835`: shutdown could mark a lost firmware CPU clean. `cpuidle_pause()` does not wait for every CPU to leave C2, and a CPU stuck in firmware (RCU idle) does not hold up the grace period | per-CPU busy word set before the C2 entry reads the gate (full barrier); the reboot notifier closes the gate, issues the same barrier and waits, at most 1 s while kicking idle CPUs, for busy = 0 on every possible CPU. Any missing acknowledgement leaves the record as it is (ARMED). `cpuidle_pause()` removed (section 1, item 5) |
| 2 MEDIUM | `:679`, `:814`, `:1957`: the notifiers run before `SYSTEM_RESTART` is set, so a pending arming work could arm after the close and persist ARMED | irreversible `e9810_shutdown` flag set under `e9810_arm_lock` before the mask is cleared; checked in `e9810_arm_refused()`, `e9810_qm_session_arm()`, the C2 entry gate and the detector |
| 3 HIGH | `:836`: IDLE was committed before the later notifiers, `device_shutdown()`, syscore and the reset, so a panic or watchdog expiry there left a clean record; kexec too | the drain writes the intermediate `SHUTDOWN` (next boot: UNCLEAN, reason `shutdown`). IDLE only at the last controlled point: a restart handler at INT_MAX after an orderly `kernel_restart()` (syscore shutdown passed, no panic, oops, loss or kexec, try-lock only), syscore shutdown for halt and power-off. kexec is unclean by definition. Final window documented (section 1, item 5) |
| 4 HIGH | `:1798`: a corrupt newest slot resurrected an older clean record | record v3: trusted only with both slots valid and chained; one slot or two that do not chain are ambiguous (UNINIT, reason `ambiguous`), merged conservatively, so a surviving ARMED, SHUTDOWN or quarantined slot keeps its quarantine; fresh records go into both slots (section 6) |
| 5 MEDIUM | `s3c2410_wdt.c:911`: `.restart` disabled WTCON, reloaded WTCNT and changed the prescaler without `may_pet()` or `wdt->lock`; self-test double reload at `:1253/:1254` | guarded `.restart`: bounded trylock, latch without reload, WTCNT only lowered, prescaler untouched, counter/reset only switched on. Self-test: timeout without reload, then the final stop (one reload). Shared API unchanged and described (section 5) |
| 6 MEDIUM | `:1861`: only the last token was remembered (A, B, A cleared each time), and invalid-record init erased the token history | monotonic trial floor (word 12): a token clears only above it; rounds 0/1 retired; without a trusted record the presented token is spent into the floor, so a new DTB with a higher token (or root) is required |
| also | the recovery procedure was not documented | section 9 |

## 9. Recovery: clearing a quarantine

The record never clears itself. Every boot logs what is in force and the two ways out.

**With root** (a real BOOT kernel, Lineage 6.12 with a root shell, or the serial
console):

```sh
cat /sys/bus/platform/devices/14070000.cpuidle/c2_quarantine     # what is recorded and why
echo 1 > /sys/bus/platform/devices/14070000.cpuidle/c2_quarantine_clear
reboot                                                            # a clean restart
```

- The clear writes a record with no flags (UNINIT and UNCLEAN included). It keeps the
  session word, the consumed drills and the trial floor. It applies from the **next**
  boot; this boot keeps the quarantine it started with, so nothing is armed now.
- Because this boot is quarantined, it never armed C2, its session is IDLE, and the
  clear survives even an unclean end of this boot. The exception is a LITTLE-only
  quarantine with big C2 armed this boot: then end the boot with a clean `reboot`, or
  the next boot is UNCLEAN again.
- If the page is wiped before the next boot (cold boot, a kernel that overwrites it),
  the next boot is UNINIT again: clear once more.
- `echo 1 > .../c2_drill_rearm` makes the drills run again on the next boot whose DT
  asks for them. `echo 1 > .../c2_disarm` disarms every CPU until the next boot.
- `sysfs` writes return `-EIO` if the record write failed its read-back (nothing
  changed) and `-ENODEV` without the page.

**Without root (ROM-RAM: adb root stays shell).** The only way is a trial DTB whose
`samsung,c2-quarantine-trial-token` is above the record's trial floor, booted onto a
**trusted** record:
- the token clears every flag once, then becomes the floor. Further boots of the same
  DTB do not clear again, so a loss after the token boot stays quarantined on every
  later boot of that pack;
- on a missing or ambiguous record (first boot after a cold boot, the page overwritten
  or damaged) the token clears nothing and is spent. That pack can no longer clear
  anything: you need a new DTB with a higher token (the next one above 0xc2c20004) in a
  new pack, booted after a boot that left a trusted record;
- a boot with no token (for example drill (a)) leaves a trusted record (UNINIT if it
  found none) without spending anything. That is how to prepare the page for a token
  boot.
- A quarantine itself is safe: all C2 (or LITTLE C2) stays off and the phone runs on WFI.

## 10. Limits and open points (residual risks)

- **Token cost after a wipe.** A token boot onto a missing or ambiguous record spends the
  token (renewed authorization by design). The S1a pack therefore must follow a boot that
  left a trusted record (section 3.2). If the first 6.12 boot after the 4.9 bounce always
  finds the page garbage (4.9 or sboot overwrites it), no token boot can ever clear in the
  shim flow, and C2 never arms there (safe).
- **Older consumed tokens after a wipe.** The floor only knows the tokens it has seen
  since the last wipe. After a wipe, an older token that was used before the wipe but is
  still above the current floor can clear once more. Bounded by: all round 0/1 tokens are
  retired in the driver, tokens are issued in increasing order (an older DTB has a lower
  token than the current floor once the newest pack was booted after the wipe), and the
  shim bounces every non-SWRESET reset to Lineage (no autonomous loop).
- **First boot after a cold boot, after round 0/1, or after an ambiguous record** never
  arms C2 (UNINIT). That is the intended fail-closed cost. A real BOOT kernel needs root
  `c2_quarantine_clear` once per wipe of the page.
- **False UNCLEAN.** Any panic, watchdog reset, power loss or kexec while C2 was armed
  quarantines C2, even when C2 was not the cause (the suspend deadline, an unrelated oops,
  a hang in `device_shutdown()` after the drain). Root clear is needed then.
- **Drain bound.** A healthy CPU leaves C2 within its exit latency. A CPU that needs more
  than 1 s after the kick (only a firmware fault) counts as not acknowledged: the session
  stays ARMED and the next boot is UNCLEAN (fail closed).
- **Final window** (section 1, item 5): a failure after the IDLE write, in the remaining
  restart handlers, the reboot-failed loop, or `machine_halt()`/`machine_power_off()`,
  leaves a clean record. Every CPU had acknowledged leaving C2 before that, behind a
  closed gate.
- **Power-off handlers.** No sys-off handler is registered, so power-off closes the
  session at syscore shutdown, before `machine_power_off()`. A failed power-off
  afterwards becomes a watchdog reset with a clean record. Power-off is not a trial path.
- **Page survival across 4.9 and sboot** is expected from the 4.9 snapshot layout, not
  measured. Drill (a)'s second boot measures it. If the page does not survive, every
  6.12 boot after 4.9 is UNINIT (safe, but C2 never arms in that flow).
- **Detector blind spots:** a CPU stuck in kernel code with IRQs off (stage
  `cal_cpu_disable`, `cal_cpu_enable`, `cpu_pm_exit`) is left to the buddy hard-lockup
  panic (12-24 s). After 5 busy hotplug scans the detector runs without the hotplug lock
  and relies on revalidation alone. The detector stops once the shutdown gate is closed;
  the drain covers that phase.
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
  - The guarded `.restart` makes its writes without the lock if the 100 ms trylock
    fails (a CPU stopped while holding it). They only shorten the countdown.
  - On the ROM-RAM DTB the diagnostic restart guard (255) resets first, so the
    watchdog `.restart` (128) only runs if that guard returns, which it never does.
    The guarded `.restart` path is therefore exercised only on DTBs without the guard.
- **Selftest-only DTBs without the page** (`delayed-ufs-diag`) may still arm in self-test
  phases with no session record. They are diag only and run on the shim vehicle.
- The detector's ping cadence vs the C2 entry rate: compare the C2 entries/s of S1a with
  4.9 (design open point).
- Drill (b) and S1b (LITTLE cpu1-2) are not packed.
