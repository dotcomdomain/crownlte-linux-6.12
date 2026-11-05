# Exynos 9810 C2: the q22b "secondaries" failure and the idle-c changes

Branch `crownlte-integration-idle-c` (based on `crownlte-integration-idle`
42bdff2d0). Trial q22b ran stage B (`crownlte-integration-idle`, Image
6d49719c, DTB 787591e2, recovery 0403a9f0) on 2026-11-04. The serial log is
`trial-acpm-q22b-idleb-s2-20261104/serial-ready.bin`. Trial q40-idlec runs
this branch.

Evidence labels used below:

- **CONFIRMED**: read directly from the q22b log, the 4.9 source
  (`/root/crownlte-hid-kernel`) or the APM firmware image.
- **INFERRED**: the most likely reading. q40 is set up to confirm or
  refute it.

## 1. What q22b actually shows

### 1.1 Corrections to the first reading

- **cpu1's `last_err=-22` did not come from the failing phase.**
  `last_err` is a running value, not a per-phase delta. In phase
  `secondaries`, cpu1 has `psci_fail=+0`. The -22 is the one
  `psci_fail=+1` from phase little-1 (`little-1 cpu1 c2_enter=+301
  resumed=+299 psci_fail=+1 last_err=-22 wfi_fallback=+1`). In
  `secondaries`, cpu1 has `c2_enter=+2 resumed=+1`: its second
  CPU_SUSPEND never returned. CONFIRMED (log lines 793 and 807).
- **"None of the others answered" is a reporting artifact.** The 6.12
  check loop stopped at the first unanswered CPU (cpu1). It never counted
  the answers from cpu2..7, so they all print `answered=0/1`. Their
  sleeper threads exited normally, and their `c2_enter` equals
  `resumed`. CONFIRMED (6.12 `e9810_run_phase()`: the loop `break`s
  after cpu1; log lines 809-820).
- **cpu4's -95 is benign.** arm64 `cpu_suspend()` turns a finisher that
  returns 0 (the firmware came back without powering down, for example
  because an interrupt was already pending) into `-EOPNOTSUPP`
  (6.12 `arch/arm64/kernel/suspend.c:150-154`). CONFIRMED.
- **Two CPUs were lost, not one: cpu1 and cpu3**, both LITTLE.
  cpu3 has `c2_enter=+5 resumed=+4`, and its sleeper also "did not exit;
  leaked". After the disarm, `grp2_en=0xa0`: bits 5 and 7 (cpu1 and
  cpu3) were still armed. That means `cal_cpu_enable()` never ran for
  either core. CONFIRMED.
- The BIG cores (4-7) and cpu2 came back in every case. CONFIRMED.

### 1.2 Register states (PMU 0x14060000, PMU_INTR_GEN 0x14070000)

INTR_GEN bits: LITTLE phy0..3 use bits 4..7 and BIG phy4..7 use bits 0..3
(4.9 `flexpmu_cal_cpu_exynos9810.h`).

| when | cpu1 STATUS (0x202c) | INFORM1 (0x864) | GRP1_EN (0x100) | GRP1_PEND (0x108) | GRP2_EN (0x200) | GRP2_PEND (0x208) | EVT_EN_GRP2 (PMU 0x7f08) |
|---|---|---|---|---|---|---|---|
| before the test | 0x100f0f (on) | **0x10000** | 0 | 0xff | 0 | **0xef** | **0xffffffff** |
| FAIL (~125 ms in) | **0x880000 (off)** | 0x1 (C2) | 0x6c (cpu1 set) | 0xdc (cpu1 clear) | 0xef | 0 | 0xffffffff |
| after disarm +300 ms | **0x100f0f (on)** | **0x1** | 0 | 0xff | **0xa0** | 0 | 0xffffffb0 |

What this means for cpu1:

1. At FAIL, the APM had powered cpu1 off (STATUS[3:0]=0) and armed its
   GRP1 wakeup (GRP1_EN bit 5). No wakeup was latched for it
   (GRP1_PEND bit 5 = 0), although an IPI had been pending for up to
   100 ms. CONFIRMED.
2. In the same sample, GRP1 wakeups for cpu2, cpu6 and cpu7 were both
   pending and enabled (0xdc & 0x6c = 0x4c) but not yet serviced. If the
   APM serves GRP1 within microseconds, three unserviced wakeups at one
   random instant are unlikely. INFERRED: the APM was busy or blocked at
   that moment, for example in the WAIT step of another core's power
   sequence.
3. 300 ms after the disarm, cpu1 was powered on again (STATUS 0xf). But
   INFORM1 was still 1 and its GRP2 interrupt was still enabled, so
   `cal_cpu_enable()` never ran. cpu1 was not back in the kernel's C2
   exit path, or it was stuck before `cal_cpu_enable()` finished.
   CONFIRMED. *Where* it was stuck (EL3 warm boot, or spinning on
   `pmucal_cpu_lock` inside `cal_cpu_enable()`) cannot be read from q22b.
   The idle-c stage marker answers this.

### 1.3 State left by the bootloader (the key 4.9 difference)

The "before" dump was taken at 91 s, before any CPU had entered C2. It
shows that nothing in 6.12 had reset the C2 hooks of any core:

- `PMU_EVENT_INTERRUPT_ENABLE_GRP2` (PMU 0x7f08) = 0xffffffff. Every
  core's GRP2 event source was enabled.
- GRP2_PEND = 0xef: a GRP2 (core-down) event was latched for every
  secondary core.
- INFORM1 = INFORM2 = 0x10000, a value the kernel never writes. The APM
  reads INFORM bit 16 separately from the mode (APM firmware
  `FUN_00011408`: `(INFORM[cpu] & 0x1ffff) >> 16`; `FUN_00011348`
  reads the mode as `INFORM & 0xffff`). CONFIRMED (log lines 784-791;
  APM image `next-research/apm.bin`, decompiled with Ghidra 12.1.4).

4.9 never runs C2 in this state. Every CPU that comes online, CPU0
included, runs `cal_cpu_enable()` on itself. That clears INFORM, disables
the core's GRP2 interrupt, clears a latched GRP2 event and clears the
core's 0x7f08 bit:

- `drivers/soc/samsung/exynos-powermode.c:693-705`
  `exynos_hotplug_in_callback()` -> `cpu_enable()` (:81-87) ->
  `cal_cpu_enable(phycpu)`
- registered at `exynos-powermode.c:925-938` as cpuhp state
  `CPUHP_AP_EXYNOS_CPU_UP_POWER_CONTROL` (AP state: it runs on the CPU
  being brought up).

CONFIRMED.

In q22b, cpu1 and cpu4 had already cycled through C2 in their solo
phases, so their hooks were clean. cpu2, cpu3, cpu5, cpu6 and cpu7 did
their **first** C2 entry inside phase `secondaries`, all within the same
few milliseconds, each with a stale latched GRP2 event. The failure came
about 125 ms into that phase. In `cal_cpu_disable()`, step 1 enables the
core's GRP2 interrupt (`GRP2_INTR_BID_ENABLE |= bit`). With a latched
event, the APM receives a "core down" request right away, while the core
is still in kernel code (inside `pmucal_cpu_lock`, before `cpu_suspend()`
and before EL3). CONFIRMED that this ordering is what the code does.
INFERRED that it is what broke q22b. Five such early requests at once
can leave the APM waiting for a core that is not in WFI yet. During that
time GRP1 wakeups go unserviced (point 2 above). A core whose CPU_SUSPEND
then fails or returns early can be cut at its next WFI without a saved
context. The single `-22` in phase little-1 fits the same pattern: it was
cpu1's first C2 entry, also with a stale event.

The APM image confirms that the APM really runs per-core power sequences
on these registers. Its sequence tables (6-word entries at 0x12308-0x14800)
contain, for example, `CLUSTER0_CPU1_CONFIGURATION (0x2028)[3:0]=0xf`
followed by `GRP1_EN bit5 = 0` (core on), and
`CLUSTER0_CPU2_CONFIGURATION (0x2030)[3:0]=0` followed by
`GRP1_EN bit6 = 1` (core off, then arm the wakeup). The tables are
CONFIRMED. The meaning of the entry types (1 = masked write, 9 = write
and wait?) is INFERRED.

## 2. What 4.9 writes before and after each CPU_SUSPEND (question 2)

Path: `cpuidle-exynos64.c` `exynos_enter_idle()` (:80-100) ->
`prepare_idle()` (:41-53) -> `enter_idle()` -> `arm_cpuidle_suspend()` ->
`psci_cpu_suspend_enter()` (`drivers/firmware/psci.c:475-495`) ->
`post_idle()` (:55-64). All of this is CONFIRMED from source.

Before CPU_SUSPEND:

1. `cpu_pm_enter()` (GIC CPU interface save, arch timer, and so on).
2. `exynos_cpu_pm_enter()` (`exynos-powermode.c:521-559`) takes `c2_lock`,
   then `cal_cpu_disable(phycpu)` under `pmucal_cpu_lock`
   (`cal-if.c`, `pmucal_cpu.c:52-79`):
   1. `PMU INFORM[phycpu]` (0x14060860 + 4*phy) = `CPU_INFORM_C2` = 1
      (`flexpmu_cal_define_exynos9810.h:2`, `cal_data.c:176`). The slot is
      picked from the **live MPIDR** (`pmucal_powermode.c:21-28`).
   2. `INTR_GEN 0x14070200 GRP2_INTR_BID_ENABLE |= bit` (read-modify-write)
   3. if `0x14070108 GRP1_PEND & bit`: `0x1407010c GRP1_INTR_BID_CLEAR`
      RMW with `bit`
   4. `PMU 0x14067f08 EVENT_INTERRUPT_ENABLE_GRP2 |= bit` (RMW)

   (`flexpmu_cal_cpu_exynos9810.h:9-13` for core00; :22-26 for core01.)
3. Only if the next timer event is at least `min(cpd, sicd)_residency` =
   10 ms away:
   - **CPD** (`is_cpd_available()`, :441-466): never on the boot (LITTLE)
     cluster (:458). Only when every online core of the BIG cluster is in
     C2: `big_reset_control(0)` and `cal_cluster_disable()`
     (`MAILBOX_APM2AP` 0x14100010 bit 24 = 1). The power_state becomes
     `PSCI_CLUSTER_SLEEP`, packed as affinity level 1 = **0x01000000**
     (`psci.c:446-472`).
   - **SICD**: boot cluster only, all CPUs in C2, IDLE_IP check
     (PMU 0x3e0.. against masks at 0x3f0..), `exynos_prepare_sys_powerdown()`
     (wakeup masks, `cal_pm_enter`). power_state `pack(1,0,0)` = 0x1.
4. Otherwise plain C2: `CPU_SUSPEND(0x10000, cpu_resume)`, the same
   value as the DT `arm,psci-suspend-param = <65536>`
   (`exynos9810-crownlte_eur_open_26.dts:4808-4831`). PSCI IDs are
   `cpu_suspend = 0xC4000001`, `cpu_on = 0xC4000003`,
   `cpu_off = 0x84000002` (:5000-5003). 6.12 uses the same values.

After CPU_SUSPEND returns (resumed or failed):

5. `exynos_cpu_pm_exit()` (:561-583): `cal_cpu_enable(phycpu)`
   (`pmucal_cpu.c:16-42`):
   1. INFORM[live phycpu] = 0
   2. `0x14070200 GRP2_INTR_BID_ENABLE &= ~bit`
   3. if `0x14070208 GRP2_PEND & bit`: `0x1407020c GRP2_INTR_BID_CLEAR`
      RMW with `bit`
   4. `0x14067f08 &= ~bit`

   Then, if CPD was entered, `cal_cluster_enable()` + `big_reset_control(1)`;
   if SICD was entered, `exynos_wakeup_sys_powerdown()`.
6. `cpu_pm_exit()`.

Answers to the candidate list:

- **There is no separate CPU_OUT handshake.** The handshake is the
  INFORM value plus the GRP2 arm/disarm above. The kernel never writes
  CPU_OPTION or a CMU register for C2. CONFIRMED (no other register
  appears in this path).
- **IDLE_IP checks and wakeup masks** only matter for SICD.
  **Cluster votes** only matter for CPD on the BIG cluster. 6.12 uses
  neither, and neither applies to a LITTLE core while CPU0 is awake.
  CONFIRMED.
- **The last CPU in a cluster** gets a different state ID only for CPD
  (BIG cluster only, 0x01000000) or SICD (0x1). For LITTLE C2, 4.9 issues
  0x10000 for every core no matter how many are down. **The power_state
  composition is therefore not the 6.12 difference.** CONFIRMED.
- **The 6.12 C2 sequence itself matches 4.9 step for step.**
  `pmucal_cpu.c`, `pmucal_powermode.c`, `cal_data.c` and the sequence
  tables are byte-identical to 4.9. `pmucal_rae.c` differs only by
  `fallthrough;` annotations. The same `pmucal_cpu_lock` and the same
  live-MPIDR INFORM slot are used. CONFIRMED (`diff` on gekko).
- **The difference is the per-CPU C2 hook reset at CPU online** that 4.9
  does and 6.12 did not (section 1.3). CONFIRMED.

## 3. Root cause

- **CONFIRMED.** The 6.12 driver ran C2 on cores whose PMU C2 hooks the
  bootloader had left dirty. The GRP2 event source was enabled for all
  cores (0x7f08 = 0xffffffff), a core-down event was latched for every
  secondary, and INFORM1/INFORM2 held 0x10000. 4.9 clears all of this per
  CPU at online. 6.12 did not, so the first `cal_cpu_disable()` of each
  core handed the APM a core-down request while that core was still
  running kernel code.
- **CONFIRMED.** The failure came only in the first phase where several
  never-cycled cores did their first C2 entry at the same time. The two
  cores used alone (cpu1, cpu4) had already been cleaned by their own
  first `cal_cpu_enable()`.
- **INFERRED** (mechanism): the early core-down requests block or delay
  the APM's GRP1 (wakeup) servicing, and/or cut a core outside its
  CPU_SUSPEND context. cpu1 and cpu3 then never complete the C2 exit
  (`cal_cpu_enable()` never ran for them). Whether cpu1 was stuck in EL3
  warm boot or spinning on `pmucal_cpu_lock` held by cpu3 is not
  decidable from q22b.
- **INFERRED** (cpu1's earlier -22): the one EL3 error in little-1 is
  cpu1's first entry with a stale event. PSCI `INVALID_PARAMS`,
  `INVALID_ADDRESS` and any non-standard return code all map to -22
  (`psci_to_linux_errno()`), so the value alone does not say which.
- **CONFIRMED** (why the 120 s restart hung): q22b has two CPUs lost
  outside a clean resume. The restart path runs `cpuidle_pause()` ->
  `synchronize_rcu()` from this driver's reboot notifier, and later
  `smp_send_stop()`. A CPU lost outside an RCU-idle section blocks the
  grace period forever. The kernel keeps petting the watchdog from
  elsewhere, so nothing resets the SoC.

## 4. Changes on this branch

1. `d3e358457` (cherry-pick of d52ab57b6): the watchdog restart guard.
   A top-priority (INT_MAX) reboot notifier arms 30 s and stops petting
   on SYS_RESTART, so a restart hung by a lost CPU ends in a CL0 watchdog
   reset. Only `drivers/watchdog/s3c2410_wdt.c` changes, unmodified from
   crownlte-integration.
2. cal-if: `cal_cpu_lock_is_held()`, a diagnostic only.
3. cpuidle-exynos9810:
   - **C2 hook reset (the fix, 4.9 parity).** A cpuhp
     `CPUHP_AP_ONLINE_DYN` callback runs `cal_cpu_enable(phy)` on each CPU
     as it comes online, and on all online CPUs at probe. Every CPU gets
     it again, on itself and by IPI, right before it is armed. Results:
     `hooks_reset=N ret=R` per CPU in the self-test setup lines.
   - **Stale-event guard.** Before each `cal_cpu_disable()`, a latched
     GRP2 event for the core's own bit is cleared (single-bit W1C write
     to 0x1407020c, which is what the 4.9 core-on sequence does), and
     `stale_grp2` counts it. After the reset this should stay 0.
   - **Stage marker.** Each CPU records where it is in the C2 path:
     `kernel`, `cal_cpu_disable`, `firmware`, `cal_cpu_enable` or
     `cpu_pm_exit`. A lost CPU is located exactly.
   - **Every CPU is checked.** All unanswered CPUs are named, each with
     its FAIL-state, after-disarm and stage, plus the state of
     `pmucal_cpu_lock`.
   - **Recovery of a lost CPU**, logged step by step:
     1. "ipi": another SGI.
     2. "pmu-on": if the PMU reports the core off, request power-on with
        `CONFIGURATION[3:0] = 0xf`, as 4.9 `exynos_cpu_up()`
        (`exynos-pmu.c:68-76,132-135`) and the APM core-on table do.
     3. "psci-cpu-on": only if the marker says `firmware`. A raw SMC
        `CPU_ON(mpidr, __pa(cpu_resume))` (ID 0xc4000003 from the DT) is
        issued from a healthy helper CPU, so CPU0 survives if EL3 blocks.
        If EL3 still holds the core as suspended or on, it refuses
        (ALREADY_ON -4 / ON_PENDING -5). If it has the core as off, the
        core enters `cpu_resume` and restores the context it saved
        before CPU_SUSPEND.

     A core stuck in kernel code (any other marker) cannot be helped by
     these steps; that is reported as such.
   - **Loud failure.** Each CPU that is not recovered gets a
     `pr_emerg` "CPU-LOST cpuN NOT RECOVERED ..." line. A reminder
     `cpuidle: CPU-LOST cpuN never came back from C2 ...` repeats every
     5 s, 8 times, so the pmsg tail carries it. The `done` line ends with
     `lost=<mask>`.
   - **Restart safety.** With lost CPUs, the reboot notifier skips
     `cpuidle_pause()` (and its RCU wait) and only kicks the idle CPUs.
     Its priority is now INT_MAX-1, so the watchdog guard always runs
     first.
   - **Finer phases**, 1.5 s each: little-1 (1), big-1 (4), little-2
     (1-2), little-3 (1-3), big-4 (4-7), secondaries (1-7). They should
     finish at about 100 s. Each failure still disables deep idle and
     disarms every CPU (WFI only).
   - `last_err` is printed only if it changed in that phase.
   - A one-time readback of the GRP1/GRP2 CLEAR registers. If they read
     back the pending bits, PMUCAL's read-modify-write on them would also
     clear other cores' events, a second possible race. 4.9 has the same
     code.

## 5. Reading the q40-idlec result

Lines start with `exynos9810-cpuidle: cpuidle selftest:` (about 90-101 s)
unless noted.

1. Setup: `cpuN ... hooks_reset=<>=1> ret=0` for all 8 CPUs. Then
   `psci cpu_on fn=0xc4000003`.
2. `before cpuN ...`: expect `inform=0x0`, `event_grp2` with bits 0-7
   clear (0xffffff00 if the upper bits stay set) and `grp2_pend` with
   bits 0-7 clear. **This confirms the reset works.** If they are still
   dirty, the reset did not reach the PMU: report that before anything
   else.
3. `clear-register readback grp1_clr=... grp2_clr=...`: 0 means the
   PMUCAL read-modify-write race is ruled out. A nonzero value equal to
   the pend value means it is a real second issue.
4. Phases, in order: little-1, big-1, little-2, little-3, big-4,
   secondaries. For each: `stale_grp2=+0` and `psci_fail=+0` are
   expected. A `psci_fail` in little-1 again would refute the
   "-22 = stale first entry" inference.
5. `done result=PASS phases_passed=6/6 ... lost=` is the goal (an empty
   `lost=` mask).

If a phase fails:

- `FAIL-state cpuN ... stage=<s>` for every unanswered CPU, then
  `phase X: FAIL ... unanswered=<mask> pmucal_cpu_lock=HELD|free`. The
  first phase that fails gives the concurrency limit: two LITTLE cores,
  three LITTLE cores, four BIG cores, or mixed.
- `stage=firmware` + status off: the core is powered off and its wakeup
  is lost on the APM/GIC side.
- `stage=cal_cpu_enable` + `pmucal_cpu_lock=HELD`: the core is back but
  spins on the CAL lock. Look for another CPU with
  `stage=cal_cpu_disable`/`cal_cpu_enable`: that is the lock holder that
  died.
- `RECOVERED cpuN via ipi|pmu-on|psci-cpu-on`: tells which wake path
  works. `PSCI CPU_ON returned -4` = EL3 still holds the core as on or
  suspended.
- `CPU-LOST cpuN NOT RECOVERED` (pr_emerg) plus 5 s reminders: the core
  is gone. The 120 s restart then probably hangs. With d52ab57b6 the
  watchdog must reset the SoC about 30 s later (`restart: armed for 30 s
  and no longer petted` in the log). The phone then returns to Lineage
  through the shim v2 bounce. Read the pmsg tail
  (`/sys/fs/pstore/pmsg-ramoops-0`) for the last lines.
- Whatever happens, after a failed phase the kernel stays on WFI only,
  and the later self-tests (cpufreq 100 s, devfreq/TMU 105 s) should still
  run.

## 6. Still open

- The exact APM behaviour for an early core-down request (entry-type
  semantics of its sequence tables, WAIT timeouts) was not decompiled.
  Ghidra did not find code functions behind the table references;
  `next-research/apm.bin` is the starting point.
- The EL3 CPU_SUSPEND error path (which code becomes -22) was not
  reverse-engineered.
- BIG-cluster CPD (4.9 0x01000000 + MAILBOX_APM2AP bit 24) is still not
  used. It only becomes relevant after the secondaries pass.

## 7. Trial q40-idlec result and the q43-idled changes

q40 ran on 2026-11-04 (Image 0ad1b4cf, recovery d799010d). The log is in
`trial-acpm-q40-idlec-s2-20261104/serial-ready.bin`.

### 7.1 What q40 confirmed and refuted

- **The C2 hook reset works.** Before the test, `event_grp2=0xffffff00`,
  `grp2_pend=0`, and every `inform` is 0. The CLEAR registers read back 0
  (`grp1_clr=0x0 grp2_clr=0x0`), so PMUCAL's read-modify-write on them
  writes only the core's own bit. The RMW race is ruled out. CONFIRMED.
- **The stale-state explanation for -22 is refuted.** `psci_fail` with -22
  still happens about once per 170-240 LITTLE entries (little-1 cpu1 +1,
  little-2 cpu1 +1 and cpu2 +2) with clean hooks. It never hit the BIG
  core, which instead gets `early` (-95, 12 times in big-1). CONFIRMED.
  These are harmless: the core falls back to WFI and comes back.
- **The failure needs three LITTLE secondaries.** little-1 and little-2
  (about 430 cycles with two cores) passed. In little-3, **cpu3's very
  first C2 entry never returned**: `c2_enter=+1 resumed=+0`, and its
  sleeper never completed a loop (`loops=0`). So cpu3 went down right
  after arming. CONFIRMED.
- **`grp2_en=0x80` is cpu3's own bit**, not cpu7's: LITTLE phy N uses
  INTR_GEN bit 4+N, and FAIL-state `grp2_en=0xe0` = cpu1+2+3. CONFIRMED.

### 7.2 cpu3's sequence

| when | STATUS | CONFIG | INFORM | GRP1_EN | GRP1_PEND | GRP2_EN | stage |
|---|---|---|---|---|---|---|---|
| FAIL (+123 ms) | 0x880000 off | 0x0 | 0x1 | 0x80 (bit7) | 0x7f (bit7 **clear**) | 0xe0 | firmware |
| after disarm +300 ms | 0x100f0f on | 0xf | 0x1 | 0x0 | 0xff | 0x80 | firmware |
| after PSCI CPU_ON (=0) | 0x100f0f on | 0xf | 0x1 | 0x0 | 0xff | 0x80 | firmware |

What this means:

1. **The APM did its half.** It powered cpu3 off (CONFIG=0) and armed its
   GRP1 wakeup (GRP1_EN bit 7). This matches the APM core-off table
   (`CONFIGURATION[3:0]=0`, then `GRP1_EN |= bit`). So **the wakeup was not
   lost because GRP1 was disabled**: at the failure it was enabled.
   `grp1_en=0x0` after the disarm is the normal state after the APM's
   core-on sequence, which clears it. CONFIRMED.
2. **No wake event reached the PMU for cpu3.** GRP1_PEND bit 7 stayed 0
   for more than 100 ms, although cpu3's own 10 ms broadcast-timer IPI and
   the ping SGI were pending for it. GRP1_PEND is a latch: bits for running
   cores stay 1 until cleared, and the kernel's core-off sequence clears
   the core's own bit before CPU_SUSPEND. CONFIRMED by the register values.
   *Why* the GIC wake output for cpu3 did not latch is unknown.
3. **A later wake did latch, the APM powered cpu3 on, and the core never
   left EL3.** After the disarm IPIs GRP1_PEND bit 7 is 1, GRP1_EN is
   cleared, CONFIG=0xf and STATUS is on. But the stage marker is still
   `firmware`: `psci_cpu_suspend_enter()` never returned. INFORM is still 1
   and GRP2_EN bit 7 is still set, so `cal_cpu_enable()` never ran.
   `pmucal_cpu_lock` was free, so cpu3 was not spinning on a kernel lock
   held by another core. CONFIRMED.
4. **EL3 holds cpu3 as OFF.** PSCI CPU_ON(cpu3, cpu_resume) returned 0
   (SUCCESS). In a standard PSCI implementation, CPU_ON on a suspended or
   running core returns ALREADY_ON (-4). Nothing changed afterwards: the
   core was already powered, so no new reset happened. INFERRED: the core
   came out of reset into EL3 warm boot, EL3 found no valid suspend record
   for it (or a record from a power-down it did not expect), and parked it
   (WFI/WFE) as an OFF core waiting for a CPU_ON-triggered power-up.

So the q22b/q40 failure has one signature on two different cores (cpu1
in q22b, cpu3 in both): the third concurrent LITTLE power-down loses its
wakeup, and the core that finally gets powered back on is stuck in EL3.
The trigger is concurrency inside cluster 0 (CPU0 never in C2). The EL3
monitor is not in the readable part of the boot LUN (sboot only prints
"EL3 Monitor information"), so the EL3 side could not be decompiled.

### 7.3 4.9 comparison for these questions (CONFIRMED from source)

- **4.9 never writes GRP1_EN.** The only kernel writes to INTR_GEN group
  enables are the per-core GRP2 bits in `flexpmu_cal_cpu_exynos9810.h` and
  CPU0's GRP2 bit in the SICD/SLEEP system sequences
  (`flexpmu_cal_system_exynos9810.h`). GRP1 is entirely APM-managed, as in
  6.12.
- **4.9 has no per-cluster C2 limit and no cluster vote for LITTLE C2.**
  `exynos_cpu_pm_enter()` adds only CPD (BIG cluster, all BIG cores in C2)
  and SICD (all CPUs in C2, IDLE_IP clear). `c2_lock` only serializes the
  bookkeeping, not the CPU_SUSPEND calls.
- **4.9 wakes idle cores differently.** The 4.9 MCT driver
  (`drivers/clocksource/exynos_mct.c`, `percpu_mct_tick`) gives every CPU
  its own MCT local timer: an SPI routed to that CPU. 6.12 uses one MCT
  global comparator as tick broadcast on CPU0, so every timer wakeup of a C2
  core is an SGI sent by CPU0, often with several targets at once.
- **In 4.9 CPU0 enters C2 too.** The state "cpus 1-3 powered down, CPU0 in
  plain WFI for long periods" is normal in 6.12 (CPU0 is never armed) but
  short-lived in 4.9.

The last two are the remaining environmental differences that could
explain why 4.9 survives three concurrent LITTLE power-downs.
INFERRED; q43 tests the CPU0 one directly.

### 7.4 q43-idled changes (commit e183d6ae8)

- **Mitigation, on by default:** at most 2 cluster-0 cores inside the C2
  path at once (`samsung,c2-little-limit`, 0 = none). A third LITTLE core
  that wants C2 runs WFI instead (counted as `guarded`). q40 passed with 2.
- **Timeline sampler** on cpu7 for every phase that does not use cpu7.
  For cpus 1-3 it records STATUS/CONFIG/INFORM/stage, plus the INTR_GEN
  group bytes. It stores one entry per change: the first 128 after arming
  and the last 256. Marks are recorded for arm, each ping round, fail,
  disarm and recover. The timeline is dumped only when a phase fails.
- **Recovery:** PSCI AFFINITY_INFO of the lost core and of a running one.
  Then a new last step, "pmu-cycle", only after CPU_ON returned 0 for a
  powered core: disarm its GRP2, request power-down through CONFIGURATION
  (this completes only if the core sits in WFI), then power-up. That gives
  the core the reset that EL3's recorded CPU_ON needs.
- `PSCI_VERSION` is logged at the start (raw SMC from a helper CPU).

Phases (800 ms each), in order:

| # | phase | CPUs | LITTLE limit | purpose |
|---|---|---|---|---|
| 1 | little-1 | 1 | 2 | baseline |
| 2 | big-1 | 4 | 2 | baseline |
| 3 | little-2 | 1,2 | 2 | known pass |
| 4 | little-3-lim | 1,2,3 | 2 | **the fix**: q40's failing case with the limit |
| 5 | big-4 | 4-7 | 2 | all BIG cores at once (not run alone before) |
| 6 | secondaries-lim | 1-7 | 2 | **the fix** for everything except CPU0 |
| 7 | cpu3-solo | 3 | none | is it cpu3 itself? |
| 8 | little-13 | 1,3 | none | pair with cpu3 |
| 9 | little-23 | 2,3 | none | pair with cpu3 |
| 10 | little-3-busy0 | 1,2,3 | none | three at once, CPU0 never in WFI |
| 11 | little-3-raw | 1,2,3 | none | the q40 reproducer, last |

Phases 7-11 are bisect phases and may lose a CPU. The first failing phase
ends the test with the full timeline. The watchdog guard then turns the
hung restart into a reset, as it did on q40.

### 7.5 Reading q43

1. `cluster-0 C2 limit 2` at probe, then `PSCI_VERSION=...` at about 90 s.
2. Phases 1-6 must PASS. `guarded=+N` > 0 on cpus 1-3 in little-3-lim and
   secondaries-lim shows the limit at work. If any of phases 1-6 fails, the
   mitigation is wrong: read its timeline.
3. `done result=PASS phases_passed=11/11`: three concurrent LITTLE C2s
   work in every bisect variant. The q40 loss was then intermittent, or
   it depends on timing that the timeline sampler (cpu7 busy) changes.
   Keep the limit and rerun.
4. `result=FAIL-IN-BISECT ... failed_phase=<p>`: phases 1-6 passed, so
   the limit is a working mitigation. The failing phase tells what
   matters:
   - `cpu3-solo`: something about cpu3 itself (per-core EL3/APM data or
     wake line), not concurrency. q22b's cpu1 loss argues against this.
   - `little-13` or `little-23`: any pair with cpu3 is enough. Two cores
     are fine only without cpu3.
   - `little-3-busy0`: three at once fails even with CPU0 awake, so the
     problem is the third concurrent LITTLE power-down itself.
   - `little-3-raw` only (busy0 passed): it needs CPU0 to be in WFI while
     cpus 1-3 are down. That points at a cluster-0 "all cores in WFI or
     off" condition in the APM/PMU, which never happens in 4.9 because
     CPU0 takes C2 there too. The next step would be to let CPU0 take C2
     (stage C2) or to keep one LITTLE core awake.
5. In the failing phase's timeline (`tl-first#`/`tl#`), find the lost core:
   - its stage goes 1 → 2 (cal_off → firmware);
   - STATUS goes off and CONFIG goes 0;
   - its GRP1_EN bit (`g1e`) is set.

   Then check whether its `g1p` bit ever becomes 1 between the `ping` marks.
   Compare that with what the other two cores were doing at that moment
   (stage and STATUS): which of them went down last, and how close
   together.
6. Recovery lines: `recover cpuN PSCI AFFINITY_INFO=<0 on|1 off>`,
   compared with `reference cpuM ...=0`. Then `RECOVERED cpuN via
   pmu-cycle` would show that the core was parked in EL3 and that CPU_ON
   plus a power-cycle brings it back. That would make a self-healing
   path possible.
7. If a CPU is lost, the restart hangs, and the log must show
   `restart: armed for 30 s and no longer petted` and
   `restart with lost CPUs N: cpuidle_pause() skipped`. Then a WDT reset
   follows, and the shim bounce returns the phone to Lineage, as on q40.

## 8. Trial q43-idled result and the q44-idlee policy

q43 ran on 2026-11-04 (Image 518a3ebe, recovery dfe0cb0c). The log is in
`trial-acpm-q43-idled-s2-20261104/serial-ready.bin`.

### 8.1 Result (CONFIRMED from the log and timeline)

- `PSCI_VERSION` returns -1: the firmware is PSCI v0.1, so it has no
  AFFINITY_INFO. PSCI CPU_ON returns 0 for a stuck core and has no effect.
- little-1, big-1 and little-2 passed. **little-3-lim failed even with the
  limit**: at most two LITTLE cores were in C2 (`little_in_c2=2`; guarded
  +3/+8/+27), and cpu1 was lost. So it is not "three concurrent C2
  entries": three LITTLE cores *armed* is enough. little-2 (two armed,
  cpu3 in plain WFI) has passed twice. The difference is that cpu3 also
  enters C2 sometimes.
- cpu1 had one `early` return (-95: EL3 aborted the suspend because an
  interrupt was pending) shortly before it was lost.
- **Timeline** (sampler on cpu7; cpu1's last power-down falls in the
  unsampled gap between 26 and 78 ms):
  - Normal cycle: config 0 → STATUS 11000f → 4x000c → 880000, and the
    APM sets the core's GRP1_EN bit during the power-down. Wake: the
    GRP1_PEND bit latches, the APM sets config f (STATUS c50f0c/890f00 →
    e40f0e → 100f0f), clears the core's GRP1_EN and GRP1_PEND bits, and the
    core returns about 70 µs later.
  - Early return: an IPI latches GRP1_PEND while the core is already in
    EL3 (stage 2, still powered), EL3 aborts, and the kernel then clears
    the stale PEND bit in its next core-off sequence.
  - cpu1 was off from before +78 ms with GRP1_EN bit5 set. GRP1_PEND bit5
    stayed 0 through ping 2 (+46.6 ms) and its timer IPIs, while cpu2
    kept cycling normally. The disarm IPIs at +147.0 ms latched bit5 at
    once, and the APM powered cpu1 on within 50 µs. After that cpu1 stayed
    at stage `firmware`, powered and **executing** (the pmu-cycle
    power-down request did not complete within 5 ms, so the core was not in
    WFI).
- The safety chain worked again: watchdog guard, SoC reset, shim bounce.

### 8.2 EL3 monitor: not readable

- No bootloader package (bl31/el3_mon/sboot/tzsw/ldfw) exists on gekko
  or in the backups.
- The raw boot LUN (`watchdog-research/sdb-analysis.elf`) has plaintext
  AArch64 EL3 entry code only at 0x4900, 0x16000, 0x31c00 and 0x7e000
  (boot stages: CurrentEL, VBAR_EL3, SCR_EL3 setup) and sboot. sboot
  *issues* SMCs; it does not handle them.
- No aligned PSCI function-ID constants exist in plaintext.
- The blocks at 0x160000 (64 KB) and 0x200000-0x21ffff (128 KB) have
  entropy 8.00, i.e. they are encrypted: these are the EL3 monitor / LDFW.

CONFIRMED that static RE of the PSCI path is not possible with what we
have.

### 8.3 Reading (INFERRED)

There are two linked faults, both inside firmware we cannot see:
1. a LITTLE core's power-down sometimes leaves its GIC wake output not
   reaching GRP1_PEND (a lost edge);
2. after a long, unexpected power-off, EL3's warm boot of that core
   spins (CPU_ON then reports it as OFF).

The trigger involves activity of the third LITTLE secondary. Remaining
4.9 differences that could matter:
- CPU0 also takes C2 in 4.9;
- 4.9 wakes each core with its own MCT local-timer SPI, while 6.12 uses
  SGIs from CPU0.

### 8.4 q44-idlee (commit 3bbe1acf3): measured BIG-only C2

- LITTLE arming is refused unless DT `samsung,c2-little-enable`. The
  self-test checks the refusal ("LITTLE arm check: refused (...)").
- Phases: big-1, big-2, big-3 (1 s each), big-4 (2 s), big-4-soak (10 s),
  about 90-106 s. They overlap the cpufreq (100 s) and devfreq/TMU (105 s)
  self-tests on purpose.
- The timeline samples all four BIG cores from cpu1. Its 1024-entry ring
  is frozen at the failure, so the moments before a loss survive.
- Image ccc96b45, DTB 787591e2, recovery 0efedc73.

Reading q44:
- **Pass:** `LITTLE arm check: refused`, five phase PASS lines, and
  `done result=PASS phases_passed=5/5 lost=`. Then check:
  - per BIG CPU in the soak, `c2_enter` ≈ `resumed` + `early` +
    `psci_fail`;
  - `ping answered=N/N`;
  - `pmu_off_seen` > 0;
  - `mct_bc irqs` > 0.

  The later cpufreq/devfreq/TMU self-tests should PASS and the 120 s
  restart should work without the watchdog (no "CPU-LOST" lines).
- **A BIG failure:** the frozen timeline (`tl#`) shows the four BIG cores
  around the loss. Then BIG-only C2 is not safe either, and C2 stays off
  (WFI only) until the EL3 behaviour is understood.
