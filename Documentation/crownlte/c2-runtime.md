# Runtime C2 on crownlte 6.12: S0 (self-recovery) and S1a (big C2)

Branch `crownlte-c2-runtime` on `crownlte-integration-6` 4148bf525. Design:
`roadmap/c2-runtime-design.md` (gekko), sections 4.0 (S0) and 4.1 (S1a, C1).
Build only. Nothing here has run on the phone yet.

| Commit | What |
|---|---|
| d9c0de8a9 | watchdog: `crownlte_wdt_stop_petting()` |
| 05b293073 | mct-bc: `exynos9810_mct_bc_ready_ms()` |
| f1a2ddbb2 | cpuidle: S0 chain + delayed boot arming (C1) |
| 59b2fab54 | `arch/arm64/configs/crownlte_c2.fragment` (buddy hard-lockup detector, panic) |
| a6bcb13f5 | DT: quarantine page, ROM-RAM `v23a-int6-c2`, two drill DTBs, Wi-Fi self-test dropped from the int6 DT |

## 1. What it does

### S0: self-recovery chain (any DTB where the cpuidle driver sets up)

1. **Lost-core detector** (`cpuidle-exynos9810.c`). Every 2 s a work on CPU0 looks at
   every online CPU except CPU0. A CPU that sat in the firmware stage (PSCI CPU_SUSPEND
   issued) for more than 1 s gets an async csd ping. If it has not answered after 200 ms,
   it gets a reschedule IPI and another 200 ms. No answer means lost. A healthy CPU in a
   long C2 period answers within its exit latency, so the cost is at most one extra wake
   per 2 s per sleeping CPU.
2. **On a loss** (`e9810_c2_lost()`):
   - a `C2-LOST cpuN` dump (pr_emerg, so it reaches console-ramoops);
   - the quarantine marker;
   - deep idle off, every CPU disarmed;
   - `crownlte_wdt_stop_petting()`: timeout 30 s, one reload, never again. Keepalive,
     start, stop, the restart notifier and suspend can no longer reload or switch it off;
   - `orderly_reboot()`;
   - `emergency_restart()` 75 s after the loss as a last resort.

   A drill does not reboot by itself at first: it waits 45 s for the watchdog reset, so
   the trial proves the watchdog path.
3. **Buddy hard-lockup detector** (`crownlte_c2.fragment`): `HARDLOCKUP_DETECTOR=y` (buddy,
   since there is no pseudo-NMI on GICv2) and `BOOTPARAM_HARDLOCKUP_PANIC=y`. A CPU whose
   watchdog hrtimer stops is reported by its buddy after about 12-24 s, and the kernel
   panics (`panic=30`, the watchdog is no longer petted). Soft lockups are only reported.
4. **Quarantine marker**: 4 KB no-map page `c2-quarantine@ff607000`
   (`exynos9810-crownlte-c2.dtsi`, `memory-region` of the cpuidle node). Words: magic
   `0x51324339`, version 1, boots, flags (bit0 LITTLE lost, bit1 big lost, bit2 drill), lost
   CPU mask, uptime ms, reason (1 detector, 2 selftest, 3 drill), losses, CRC32. It is
   written through a write-combining mapping followed by a DSB. RAM only: no UFS or eMMC
   write, and no other storage.
   - Placement: this is the last page of the 4.9 exynos-snapshot carve-out
     (0xfda00000..0xff608000, already no-map here as `snapshot-after-pstore`, now 4 KB
     shorter). It is **not** right after ramoops: 4.9 writes `log_kevents` at
     0xfed18000..0xff518000. 4.9 places no snapshot item in 0xff518000..0xff608000.
   - At probe:
     - a valid record with the LITTLE flag refuses LITTLE arming;
     - a big flag refuses all C2;
     - an invalid record (cold boot, garbage) starts a clean record;
     - the boot count is increased.
   - Clear: `echo 1 > /sys/bus/platform/devices/14070000.cpuidle/c2_quarantine_clear`
     (root). It takes effect on the next boot.
   - **Without the page, `samsung,c2-boot-cpus` is refused.**
5. **Observability**, root-only files in `/sys/bus/platform/devices/14070000.cpuidle/`:
   - `c2_armed`, `c2_lost`, `c2_stats` (per CPU: enter, resumed, early, psci_fail,
     unarmed, guarded, wfi_fallback, stale_grp2, last_err), `c2_quarantine`;
   - write-only `c2_disarm` (kill switch until reboot) and `c2_quarantine_clear`;
   - `c2 stats:` KERN_INFO lines at the first detector pass and every 10 min.

   adb root does not work on ROM-RAM (adb-dpu run), so read these from the serial log or a
   root shell. The cpuidle sysfs in section 4 is world-readable.
6. **Drills** (DT, diag DTBs only):
   - `samsung,c2-lost-drill = <cpu ms>`: at `ms` boot time the detector treats `cpu` as
     lost (simulated).
   - `samsung,c2-loss-drill-ms = <ms>`: arms cpu1-3 (LITTLE, no concurrency limit), the
     configuration that lost a core in q22b, q40, q43 and c2diag-a/b. It disarms them after
     `ms` if nothing was lost. A recorded LITTLE quarantine skips it.
7. CPU0 is refused explicitly. Any self-test phase still works: the detector skips the
   CPUs of a running phase, and an unrecovered phase loss enters the same chain
   (reason `selftest`).

### S1a: big C2 at runtime

- **C1, arming order:** `samsung,c2-boot-cpus` is no longer applied in probe. A work runs
  at `samsung,c2-arm-delay-ms` boot time. While the only refusal is "MCT tick broadcast not
  active" (the cpuidle probe runs before mct-bc, link order), it retries every 1 s for up
  to 60 s. Any other refusal is final.
- **ROM-RAM DTB** `exynos9810-crownlte-romram-v23a-int6-c2` = int6 +
  `exynos9810-crownlte-c2.dtsi` + `samsung,c2-boot-cpus = <0xf0>` +
  `samsung,c2-arm-delay-ms = <45000>` (after the 40 s boot QoS). LITTLE stays WFI.
- **int6 DT change:** `samsung,wifi-selftest` and `-ms` were removed from
  `exynos9810-crownlte-romram-wifi.dtsi` (Sol finding on int6). This changes the int6 DTB
  too: 297eb4f4 → c30c8d04, and the decompiled diff is exactly those two properties.

## 2. Artifacts (gekko `/mnt/ssd/host-builds/kernel-upgrade`)

Config `panel-bright-work/ko-c2-a6bc/config` = int6 config (c7b917d1) +
`crownlte_c2.fragment`, `olddefconfig`. It adds only `HARDLOCKUP_DETECTOR`, `_BUDDY`,
`_COUNTS_HRTIMER`, `BOOTPARAM_HARDLOCKUP_PANIC`, `LOCKUP_DETECTOR` and
`SOFTLOCKUP_DETECTOR`. Banner: `Linux version 6.12.93-crownlte-diag+`.

| File (`panel-bright-work/ko-c2-a6bc/`) | sha256 |
|---|---|
| `arch/arm64/boot/Image` | 6e3bf57c0cd7032428da731c59f9f0c92abc613d339751635b05157d4ec0f8b8 |
| `.../exynos9810-crownlte-romram-v23a-int6-c2.dtb` (S1a ROM-RAM) | b8182e37e636164a2cb68537bbe1520a646b8658971eb30bd2024126d22e4a85 |
| `.../exynos9810-crownlte-romram-v23a-int6.dtb` (int6 minus Wi-Fi self-test) | c30c8d048dc10eb2ad841e6748b7063be9dc3a8ccbdb8c29428f0fbe8f4c24ee |
| `.../exynos9810-crownlte-delayed-ufs-c2-drill-lost-diag.dtb` (drill a) | 6488152a30e86eccaf392ae7749bd691ce87e05ef96da2413202860c41ec53c2 |
| `.../exynos9810-crownlte-delayed-ufs-c2-drill-loss-diag.dtb` (drill b) | 29597c8142cae11ffa9de5e422929f095ac6ff1c714a05613e164b95cce79cfe |
| `config` | 668b03c456c204e0473e883e5a31e0edd1c90349dd52537fd79ba478f16ffa7c |

Packs (shim bbea959, parent a8198be0, restore RECOVERY 8763860c):

| Pack | recovery.img sha256 | Inputs |
|---|---|---|
| `c2-drill-lost-20261107` (delayed-UFS, drill a) | 57715ff50b2beacb5204ec0e5b8e0ed45f15e05f6e50c7baf6bfbffde8e93a0f | `pack-acpm-q-paths.py`, Image 6e3bf57c, DTB 6488152a, ramdisk 506232b1, loader bb1be537. The manifest `purpose`/`kernel_source` are packer defaults; see PROVENANCE.txt |
| `rom-ram/pack-romram-int6-c2` (ROM-RAM S1a) | 611cbbb0778594b9d3850ce68a423f1e5a3edb694ed83c169f0830de7ccf62ea | `rom-ram/pack-rom-ram.py`, KO `panel-bright-work/ko-c2-a6bc`, DTB b8182e37, ramdisk build-30 e615cc8d, loader bd72d978 |

Drill (b) has a DTB but no pack yet. It loses a real core on purpose; run it only with
the owner present, after drill (a) passed.

## 3. Trials, PASS lines and failure signatures

### 3.1 Drill (a), delayed-UFS (`c2-drill-lost-20261107`)

Standard delayed-UFS run (READY only, run-trial.sh default 180 s). Timeline: big C2 armed
at 30 s, simulated loss of cpu2 at 60 s, watchdog reset about 90 s. The PID1 restart at
120 s must **not** be what ends the run.

PASS, in this order:

```
exynos9810-cpuidle: C2 quarantine marker at 0x00000000ff607000: no valid record (...), starting a clean one   (or "clear (...)")
exynos9810-cpuidle: C2-LOST drill ENABLED: cpu2 is treated as lost at 60000 ms boot time (simulated); ...
exynos9810-cpuidle: C2-LOST detector running: every 2000 ms, ping after 1000 ms in firmware, 200 ms per ping; ...
exynos9810-cpuidle: C2 boot arming of 4-7 at 30000 ms boot time
exynos9810-cpuidle: C2 armed for 4-7 after MCT broadcast ready at <t> ms (armed at ~30000 ms)
exynos9810-cpuidle: C2-LOST drill: treating cpu2 as lost at 600xx ms (simulated; the CPU is fine)
exynos9810-cpuidle: C2-LOST cpu2 (DRILL, simulated): no answer from C2 (drill): stage=kernel ...
exynos9810-cpuidle: C2-LOST: quarantine marker written at 0x00000000ff607000: flags=0x5 cpus=0x4 reason=drill losses=N crc=ok; next boot: C2 quarantined (LITTLE)
exynos9810-cpuidle: deep idle disabled, WFI only: C2-LOST drill
s3c2410-wdt 10050000.watchdog: stop petting (C2-LOST drill): watchdog reset in 30 s (WTCON=... WTCNT=...)
exynos9810-cpuidle: C2-LOST: watchdog no longer petted: SoC reset within 30 s
exynos9810-cpuidle: C2-LOST drill: waiting for the watchdog reset; orderly reboot only if none within 45 s
```

Then the log stops at about 90 s and the phone returns to Lineage without a button press:
a watchdog reset is not a SWRESET, so the shim bounces it. On 4.9, check:
- `/proc/reset_reason`, or `sec_debug.rst_stat` bit 24 (CLUSTER0_WDTRESET) in
  `/proc/cmdline`;
- `/sys/fs/pstore/console-ramoops*` holds the C2-LOST lines.

Then restore RECOVERY 8763860c.

Optional second boot of the same RECOVERY (to see the marker being read): the probe must log

```
C2 quarantined (LITTLE): cpus 0x4 lost at 600xx ms (drill, drill) on an earlier boot, ...; LITTLE C2 off, big C2 allowed; clear: ...
```

and arm 4-7 again; the drill then repeats. That boot comes after a 4.9 boot, so the line
also shows whether 4.9 left the page alone. "no valid record" there means 4.9 or sboot
overwrote 0xff607000: report it. A direct 6.12 → 6.12 restart, the real-BOOT case that the
marker is for, does not pass through 4.9.

FAIL signatures:
- `C2-LOST drill: FAIL: no watchdog reset 45 s after petting stopped` (the watchdog did not
  fire, then orderly reboot);
- `C2-LOST: no armed watchdog (-19)`;
- `C2 quarantine marker not available: ...`;
- `C2 boot arming of 4-7 refused: ...`;
- `crc=BAD`;
- the PID1 120/150 s restart line, or `C2-LOST: still running 75 s after the loss`;
- the phone not back in Lineage within about 3 min (manual reset needed).

### 3.2 S1a, ROM-RAM (`rom-ram/pack-romram-int6-c2`)

The standard int6 ROM-RAM trial (integration-6.md section 5) plus the soak in
roadmap/c2-runtime-design.md 4.1: 15 min screen on, 30 min screen off (Wi-Fi associated),
15 min load. Take a cpuidle and /proc/interrupts snapshot every 60 s (section 4). All
int6 checks apply, except that no `wifi selftest: RESULT` line is expected any more; Wi-Fi
is checked via the HAL.

PASS:

```
exynos9810-cpuidle: C2 quarantine marker at 0x00000000ff607000: ... (no "C2 quarantined")
exynos9810-cpuidle: C2-LOST detector running: ...
exynos9810-cpuidle: C2 boot arming of 4-7 at 45000 ms boot time
exynos9810-cpuidle: C2 armed for 4-7 after MCT broadcast ready at <t> ms (armed at ~45000 ms)
exynos9810-cpuidle: c2 stats: armed=4-7 lost= deep_off=0 quarantine=none ...   (every 10 min)
exynos9810-cpuidle: c2 stats: cpu4 enter=... resumed=... early=... psci_fail=0 ...   (cpu4-7)
```

- Per CPU at each `c2 stats:`: resumed + early + psci_fail = enter, or enter - 1 for a CPU
  whose stage is `firmware` at that moment.
- psci_fail = 0 on the big cores.
- `cat /proc/sys/kernel/hardlockup_panic` is 1 and `cat /proc/sys/kernel/nmi_watchdog` is 1
  (buddy detector on).
- The final restart is normal (the reboot notifier disarms).
- Residency at screen off: big ≥ 85 % (4.9: 90-92 %).

FAIL signatures:
- `C2-LOST`;
- `C2 detector: cpuN silent for 200 ms in the firmware stage` (followed by a reboot);
- `deep idle disabled`;
- `Watchdog detected hard LOCKUP on cpu`;
- `rcu: INFO: rcu_preempt self-detected stall` or `detected stalls`;
- `C2 boot arming ... refused`, `C2 quarantined`;
- big psci_fail > 0;
- the `c2 stats:` lines stop.

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

## 5. Limits and open points

- The detector cannot see a CPU stuck in kernel code with IRQs off (stage `cal_cpu_disable`,
  `cal_cpu_enable`, `cpu_pm_exit`). The buddy hard-lockup panic covers that case after
  12-24 s.
- The marker's survival across a 6.12 watchdog reset matches ramoops' (same DRAM, kept
  through q35/q37 resets). That it survives a 4.9 boot in between is expected from the 4.9
  snapshot layout, not measured.
- The detector's ping cadence vs entry rate: compare the C2 entries/s of S1a with 4.9
  (design open point).
- Drill (b) and S1b (LITTLE cpu1-2) are not packed.
