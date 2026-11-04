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
