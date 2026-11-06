# crownlte 4.9 performance and efficiency parity (branch `crownlte-perf`)

Status: **build-only, not run on the phone.** Everything below is opt-in. A DT
enables it: `arch/arm64/boot/dts/exynos/exynos9810-crownlte-perf.dtsi`, or the
ROM-in-RAM variant `exynos9810-crownlte-romram-v23a-perf.dts` that includes it.
A DTB without that file behaves exactly as before. The only exception is the
defconfig I/O scheduler (item 5).

Source of the work: `roadmap/tuning-inventory.md` (gekko), items 1, 5, 6, 8 and 9
of its section 6. Not in scope: suspend, C2/cpuidle, display, the GPU power
domain, the big turbo above 1794 MHz, charging.

## Branch layout

| Commit | What |
|---|---|
| `ba4ee4518` | merge of `crownlte-panel-s2dos05-diag` (233829a14) into `crownlte-integration-4` (b4f756bed) |
| `912e62362` | cal-if: `cal_dfs_extend_max_freq()` |
| `f7a6223b6` | cpufreq: EM, LITTLE 1950, schedutil rate limit, boot QoS, perf report |
| `5c95e1144` | TMU: ECT parity log, guarded power_allocator, 96/91 °C big cap |
| `d93e1d2cf` | `drivers/input/misc/crownlte-input-booster.c` |
| `03bc15ebe` | UFS: DT clock gating with the 4.9 delay, hibern8 counters |
| `b02d3f769` | defconfig: mq-deadline, power_allocator, input booster |
| `6fb945880` | DT: perf dtsi, the v23a-perf and delayed-ufs-perf DTS files, a comment fix |
| (this commit) | this document |

**Why the merge.** The base DTS for the deliverable, `exynos9810-crownlte-romram-v23a.dts`,
exists only on `crownlte-panel-s2dos05-diag`. That branch and integration-4 share
`d4af69833`:
- integration-4 adds 3 commits: the TEE S-SIQ edge DT and a TEE doc;
- s2dos05-diag adds 5 commits: the CMGP USI wrapper, the read-only S2DOS05
  diagnostic, the v23a DTS, and one defconfig line (`CROWNLTE_S2DOS05_DIAG`).

The two sets touch disjoint files. I merged rather than rebased, so neither
branch's reviewed commit IDs change. The merge had no conflicts.

The v23a DTS in this tree compiles to the same DT as the shipped
`panel-bright-work/dtb/exynos9810-crownlte-romram-v23a.dtb`: the decompiled
v23a-perf DTB differs from it only by the additions listed here.

## Staging switches

| Switch | Where | Effect without it |
|---|---|---|
| `capacity-dmips-mhz`, `dynamic-power-coefficient` | CPU nodes | all CPUs 1024, no EM, no EAS (as before) |
| `samsung,dt-max-freq-above-cal` | `/cpufreq/cluster0` | LITTLE stops at the CAL max 1794 MHz |
| `samsung,schedutil-rate-limit-us` | `/cpufreq/cluster{0,1}` | 750 µs (1.5 × transition latency) |
| `samsung,boot-qos-ms`, `pm_qos-booting` | `/cpufreq`, cluster1 | no boot QoS |
| `samsung,perf-report-ms` | `/cpufreq` | no report |
| `samsung,ect-thermal`, `samsung,ect-pidtm`, `samsung,cooling-cpu`, `samsung,hotplug-temps` | TMU sensor nodes | step_wise as before, no ECT log, no 96 °C cap |
| LITTLE trips + cooling maps, BIG `sustainable-power` + 60 °C active trip | `/thermal-zones` | LITTLE never throttles (as before) |
| `samsung,clk-gating`, `samsung,clkgate-delay-ms` | `/soc@0/ufs@11120000` | no UFS clock gating (`clkgate=1` still works) |
| `input-booster` node | `/` | no touch boost |
| `CONFIG_MQ_IOSCHED_DEADLINE=y` | defconfig / build config | I/O scheduler `none` |
| `sched_verbose` | v23a-perf `bootargs` | the scheduler does not log "starting EAS" |

## 1. Scheduler topology, energy model, EAS

**Values**

| | LITTLE (cpu0-3, A55) | big (cpu4-7, M3) |
|---|---|---|
| `capacity-dmips-mhz` | 272 | 785 |
| `dynamic-power-coefficient` | 160 | 800 |
| expected `cpu_capacity` | **385** (with 1950) / 354 (1794 max) | 1024 |

**4.9 evidence**
- EMS `capacity-mips` 272/785 and `power-coefficient` 160/800: DT49:4776-4795.
- The 4.9 formula is P[mW] = C·f[MHz]·V[mV]²/10⁹ (`K49/kernel/sched/ems/energy.c:508-541`).
  The 6.12 OPP path (`drivers/opp/of.c` `_get_power()`) computes the same thing in µW:
  C·mV²·MHz/10⁶.
- 4.9 itself ran big at up to 2314 MHz, so LITTLE was about 299 there. With big
  capped at 1794 MHz here, LITTLE comes out at 385.

**How it is registered**
- `exynos9810_driver.register_em = cpufreq_register_em_with_opp`. The cpufreq core
  calls it for each new policy before `->ready`.
- The driver already adds every table level as an OPP, with the FVMap voltage in µV.
  `dev_pm_opp_of_register_em()` reads the CPU node's coefficient and registers a µW EM.
- It registers nothing without the coefficient, so old DTBs are unaffected.
- The `thermal-cpufreq-N` cooling devices are registered in `->ready`, so they find
  the EM and become power actors.

**Expected EM** (kernel integer maths; mV truncated):
- LITTLE: 455 25.6 mW, 598 38.8, 715 51.0, 832 65.2, 949 81.1, 1053 97.8,
  1248 131.7, 1456 180.8, 1690 250.2, 1794 290.5, 1950 364.6.
- big: 650 227.9 mW, 741 259.8, 858 306.3, 962 356.9, 1066 402.5, 1170 449.5,
  1261 502.8, 1469 648.8, 1586 762.1, 1690 865.3, 1794 976.8.

**EAS/schedutil/uclamp**
- `UCLAMP_TASK`, `UCLAMP_TASK_GROUP`, `ENERGY_MODEL`, `SCHED_MC` and default
  schedutil were already on, so no config change was needed.
- 4.9 had no uclamp. Android 16 writes `cpu.uclamp.min`/`latency_sensitive` for
  nnapi-hal only (system init.rc). No vendor rc or the power HAL writes schedutil
  tunables.
- EAS turns on by itself once capacities are asymmetric, both perf domains have an
  EM, the governor is schedutil, and cpufreq gives frequency invariance (any
  `target_index` driver does).
- schedutil rate limit: 5000 µs on both clusters. This is 4.9 `freqvar-tune`
  `up_rate_limit_table`/`down_rate_limit_table` = 5000 (DT49:10415-10434).
  Without it, 6.12 uses 750 µs, and every step is an ACPM IPC of about 0.4 ms.
  Policy min/max changes (boosts, thermal) bypass the rate limit.

**Not ported:** EMS ontime, FRT/ERT, prefer-perf, and the hmp/top-app boost.
Mainline misfit migration and uclamp take their place.

## 2. Thermal parity (device ECT + 4.9)

The decoded ECT THERMAL/PIDTM blocks (bootloader copy at 0x4FC00 of `LUN-sdb.img`)
were re-checked for this work and match the inventory:

- **THERMAL LITTLE:** 20-75 °C 1950 MHz; 76 1690; 81 1456; 86 1248; 91 1053;
  96-115 455.
- **PIDTM BIG:**
  - trips 20/60/83/90/100/105/110/115;
  - k_po 100, k_pu 300, k_i 20, i_max 1000, integral_cutoff 23;
  - p_control_t 2000 mW;
  - hotplug out 96 °C, in 91 °C;
  - limited_frequency 2496000.

### LITTLE throttling

On 4.9, `exynos_tmu.c:1380-1440` replaced the placeholder `<0 0>` maps with the ECT
ladder. Each range's lower bound became a trip, and its frequency became the
cooling level. 4.9 reacted on TMU interrupts and released 5 °C lower (the DT
hysteresis).

The 6.12 equivalent is five `passive` trips with hysteresis 5 °C, each mapped to
one cooling state (min = max) of `thermal-cpufreq-0`, under the default step_wise
governor:

| Trip | State | Frequency |
|---|---|---|
| 76 °C | 2 | 1690 MHz |
| 81 °C | 3 | 1456 MHz |
| 86 °C | 4 | 1248 MHz |
| 91 °C | 5 | 1053 MHz |
| 96 °C | 10 | 455 MHz |

- The passive polling interval is 250 ms. Normal polling stays at 1000 ms.
- The states assume that 1950 is in the table (item 3). If it is not, every state
  lands one level lower, which means more throttling.
- The TMU driver logs the frequency each trip actually resolves to, next to the ECT
  value.

### BIG power_allocator

- 4.9 used `power_allocator` with its constants from ECT PIDTM.
- 4.9's `get_governor_trips()` took the last *active* trip before the control trip
  as switch-on (60 °C) and the first passive trip (83 °C) as control. 6.12 mainline
  picks them the same way.
- The DT therefore adds one `active` trip at 60 °C (hysteresis 2 °C, as in 4.9) and
  `sustainable-power = <2000>`. The 83 °C control trip, its full-range cooling map
  and the 110 °C critical trip stay as they were.

The TMU driver (`samsung,ect-pidtm = "BIG"`) switches the zone to power_allocator
only when all of these hold:
- every cooling device on the control trip is a power actor (it has an EM);
- the DT `sustainable-power` equals ECT `p_control_t`;
- the ECT block is present and sane.

Otherwise the zone stays on step_wise and logs why. This guard matters: with no
power actor, power_allocator would not throttle at all.

After the switch, the driver writes the ECT `k_po`/`k_pu`/`k_i` (<< 10, as 4.9 did)
and `integral_cutoff` 23 over the values the governor estimated.

ECT `i_max` 1000 has no 6.12 counterpart. Mainline clamps the integral term at
the sustainable power (2000 mW) instead.

### Big cores off at 96 °C, back below 91 °C

4.9 (`exynos_tmu.c:1155-1185`) dropped the online-CPU QoS to offline the big
cluster.

On 6.12, `samsung,hotplug-temps = <96000 91000>` holds the big policy at its lowest
level (650 MHz) through a freq QoS maximum, from ≥ 96 °C until the temperature is
below 91 °C. It needs no CPU hotplug, which Android 16 handles badly.

The check runs on every BIG temperature read (50 ms above the control trip). The
QoS request follows the policy through the cpufreq policy notifier.

### Unchanged

- The bootloader-armed TMU hardware trip (rise7, 112-115 °C). The driver still
  writes no TMU registers.
- G3D (step_wise at 88 °C) and ISP (the `thermal-isp-0` stub).
- **Zone names:** BIG, LITTLE, G3D, ISP.
- **Cooling devices:** thermal-cpufreq-0 (LITTLE), thermal-cpufreq-1 (BIG),
  thermal-gpufreq-0, thermal-isp-0.

These names are the ones `V/etc/thermal_info_config.json` lists. The HAL only
monitors them (no `BindedCdevInfo`), so a governor change does not affect it.

## 3. LITTLE 1950 MHz and boot QoS

### Proof that 4.9 runs LITTLE at 1950 MHz

- The DTB inside the flashed 4.9 BOOT image (`backups-20261103/BOOT.img`, sha256
  `bab7272a…`, the image the phone boots today; FDT at offset 40992768) has
  `/cpufreq/domain@0 max-freq = <0x1dc130>` = 1950000.
- 4.9 `exynos-acme.c:1114-1124` (`init_domain()`) assigns DT `max-freq` over the CAL
  maximum. The source comment says "smaller one", but the code assigns the DT value.
- `init_table()` (`:863-866`) keeps every CAL level ≤ max_freq. 4.9
  `cal_dfs_set_rate()` has no range check. So the 4.9 table is LITTLE 455-1950.
- The ECT THERMAL LITTLE ladder also starts at 1950000.
- A read-only check on the 4.9 phone (`policy0/scaling_available_frequencies`) would
  add runtime proof. It was not possible here (no phone access).

### 6.12

- `samsung,dt-max-freq-above-cal` on cluster0 calls `cal_dfs_extend_max_freq(CPUCL0, 1950000)`.
- CAL accepts it only for an ECT DVFS level with a non-zero FVMap voltage.
- The voltage is the same FVMap entry 4.9 uses: 1950 MHz @ 1081250 µV (q15 log).
  The ACPM firmware selects it; the kernel writes no voltage.
- The cpufreq driver refuses the property for big, and above 1950000 kHz.
- Big keeps `samsung,max-freq-cap = 1794000`, so all four big cores stay ≤ 1794 MHz.
- MIF floor for 1950: 845 MHz (mif-perf-table). There is no ECT skew for CPUCL0.

### Boot QoS

4.9 `set_boot_qos()` (`exynos-acme.c:903-921`) sets min = max = the table maximum,
or `pm_qos-booting` if lower, for 40 s. The 6.12 equivalent adds two freq QoS
requests on each policy at registration and removes them after
`samsung,boot-qos-ms` = 40000:

| Cluster | Boot QoS | Why |
|---|---|---|
| LITTLE | 1950 MHz | table max |
| big | 1794 MHz | `pm_qos-booting = 1794000` |

- 1794 MHz is the 4-core limit, never above it.
- Thermal maximum requests still win, because the core clamps min to max.

## 4. Input booster

`drivers/input/misc/crownlte-input-booster.c` (`CONFIG_INPUT_CROWNLTE_BOOSTER`, DT
`samsung,crownlte-input-booster`).

On 4.9, the TOUCH booster runs at default level 2 (`include/linux/input/input.h:556`).
Levels map {2 → head = level 2, tail = level 3} (`input.h:133-160`). In DT49 (input_booster
`TOUCH`), level 2 is head 130 ms with big 1066000 and LITTLE (kfc) 832000; level 3 is
tail 500 ms with big 858000 and LITTLE 832000.

The 4.9 state machine (`drivers/input/input.c:426-700`, `input.h:201-238`):
- the head starts on touch-down;
- when it expires, the tail levels apply;
- the tail ends 500 ms after release.

The 6.12 port:
- **Matching:** an input handler on direct touchscreens (BTN_TOUCH + ABS_MT_POSITION_X).
  Only BTN_TOUCH is read.
- **Requests:** one FREQ_QOS_MIN request per listed CPU's policy.
- **Behaviour:** touch-down → head (big ≥ 1066, LITTLE ≥ 832) for 130 ms → tail
  (big ≥ 858, LITTLE ≥ 832) while the finger is down → released 500 ms after the
  finger lifts. A new touch restarts the head.
- **Context:** events only record state, in atomic context; all QoS updates run
  from a high-priority workqueue.
- **Safety:** levels above a policy's `cpuinfo.max_freq` are refused. A minimum
  never lifts a maximum, so thermal and user caps win.
- **Not ported:** 4.9's `hmp_boost` (EMS top-app boost) has no equivalent here.
  The power HAL's INTERACTION hint (LITTLE ≥ 1456 MHz for 100 ms) still runs on top.

## 5. UFS and I/O

- **Clock gating.**
  - 4.9 `ufs-exynos.c:561-563` always set `CLK_GATING | HIBERN8_WITH_CLK_GATING | INTR_AGGR`.
  - 4.9 `ufshcd.c:105,1326` used `clk_gating.delay_ms = LINK_H8_DELAY` = 20 ms.
  - The 6.12 driver already had the 4.9 `setup_clocks` gating sequence but was gated
    behind `clkgate=1`. `samsung,clk-gating` now enables it.
  - `samsung,clkgate-delay-ms = <20>` applies 20 ms once the link is up (the core
    default is 150 ms).
  - `INTR_AGGR` stays off: the 6.12 host has `UFSHCI_QUIRK_SKIP_RESET_INTR_AGGR`,
    and 4.9 had `UFSHCI_QUIRK_SKIP_INTR_AGGR`.
- **Hibern8 settings.**
  - 4.9's DT `ufs-rx-min-activate-time-cap 3`, `ufs-rx-hibern8-time-cap 2` and
    `ufs-tx-hibern8-time-cap 2` have no reader in 4.9.
  - The H8 timings come from the CAL `ufs_cal_calib_hibern8_values()`
    (PA_TActivate/PA_Hibern8Time + 1), which 6.12 already runs from the same
    `ufs-cal-9810.c`. Nothing to port.
- **I/O scheduler.**
  - 4.9 used CFQ (CFG49:363-370). There was no CFQ on 6.12, and the result was
    `none`.
  - The defconfig now leaves `MQ_IOSCHED_DEADLINE` at its default `y`, as GKI does.
    It becomes the default elevator of the single-queue UFS host.
  - Kyber and BFQ stay off, as in GKI.
  - No Lineage or vendor rc writes `queue/scheduler`.
- **Read-ahead.**
  - 4.9: 128 KB.
  - 6.12 default: 128 KB, unless the device reports an optimal transfer length.
  - Android only writes `read_ahead_kb` for Virtual A/B dm devices, which crownlte
    does not use. f2fs `seq_file_ra_mul 128` is written by system init.rc.
  - No change; verify on the phone.
- **FSYS0 bus QoS** (UFS 200 MHz while active): not ported. There is no FSYS0
  devfreq instance yet.

## Verification on the phone (ROM-in-RAM v23a-perf)

Pack the Image and DTB from the artifacts below the same way as v23a. All of these
checks are read-only. Log prefixes:

| Prefix | Source |
|---|---|
| `exynos9810-acpm-cpufreq cpufreq:` | cpufreq |
| `exynos9810-tmu 10070000.tmu-big:` / `…tmu-little:` | TMU |
| `crownlte-input-booster input-booster:` | input booster |
| `exynos9810-ufs 11120000.ufs:` | UFS |

### Expected log lines

**Probe:**
```
perf: domain0: max 1950000 kHz above CAL max 1794000 kHz (DT samsung,dt-max-freq-above-cal, 4.9 acme), FVMap 1081250 uV
registered: domain0 11 levels 455000-1950000 kHz, domain1 11 levels 650000-1794000 kHz
domain0 (cpus 0-3): cooling device thermal-cpufreq-0 = cooling_deviceN, max_state 10
perf: domain0 (cpus 0-3) boot QoS min=max=1950000 kHz for 40000 ms (4.9 set_boot_qos) ret=1/1
perf: domain1 (cpus 4-7) boot QoS min=max=1794000 kHz for 40000 ms (4.9 set_boot_qos) ret=…
sched_energy_set: starting EAS                       (sched_verbose)
root_domain 0-7: pd4:{ cpus=4-7 nr_pstate=11 } pd0:{ cpus=0-3 nr_pstate=11 }   (KERN_DEBUG: dmesg)
BIG: hotplug-equivalent cap armed on policy4: cap at 650000 kHz from 96000 mC, release below 91000 mC (ret=0)
clock gating on (hibern8 with gating), delay 20 ms (0 = core default)
clock gating delay 20 ms (4.9 LINK_H8_DELAY)
hibern8 enter #1 (exit 0, CAL errors 0)
touch boost: head 130 ms, tail 500 ms after release, 2 cluster(s) (4.9 input_booster TOUCH level 2)
policy4 (cpus 4-7): head 1066000 kHz, tail 858000 kHz (ret=0)
policy0 (cpus 0-3): head 832000 kHz, tail 832000 kHz (ret=0)
boosting touches of sec_touchscreen
```

**After the cooling devices bind (about 1 s):**
```
LITTLE: ECT LITTLE range 76-80 C max 1690000 kHz flag 0        (8 ranges)
LITTLE: trip 76000 mC passive hyst 5000 -> thermal-cpufreq-0 state 2..2 = 1690000 kHz (ECT 1690000 kHz) = ECT
LITTLE: trip 81000 … state 3..3 = 1456000 … = ECT ; 86000 → 1248000 ; 91000 → 1053000 ; 96000 state 10..10 = 455000
LITTLE: governor step_wise polling 250/1000 ms, …; ECT ladder 5 checked, 0 above ECT
BIG: ECT PIDTM BIG: k_po 100 k_pu 300 k_i 20 i_max 1000 integral_cutoff 23 p_control_t 2000 mW hotplug out/in 96/91 C limited_frequency 2496000
BIG: governor power_allocator, ECT PID constants applied
BIG: trip 83000 mC passive hyst 5000 -> thermal-cpufreq-1 state 0..10 = 650000 kHz (ECT 0 kHz)
BIG: trip 60000 mC active hyst 2000 (no cooling device)
BIG: governor power_allocator polling 50/1000 ms, sustainable 2000 mW, k_po 102400 k_pu 307200 k_i 20480 integral_cutoff 23 (FRAC_BITS 10); …
```
(For a trip, `state a..b = N kHz` is the frequency at the deepest state of that map.)

**About 40 s after cpufreq registers:** `perf: domain0 boot QoS 1950000 kHz released after 40000 ms, cmu=…`
(the same for domain1).

**At 60 s:**
```
perf report: cpu0 cluster0 core0 online=1 capacity=385 capacity-dmips-mhz=272 freq_ref=1950000 kHz
perf report: cpu4 cluster1 core0 online=1 capacity=1024 capacity-dmips-mhz=785 freq_ref=1794000 kHz
perf report: policy0 domain0 gov=schedutil rate_limit=5000us (dt 5000) cpuinfo=455000-1950000 … ext_max=1950000 boot_qos=1950000 cdev=thermal-cpufreq-0
perf report: domain0 em: cpus 0-3, 11 states, uW, artificial=0
perf report: domain0 em[0..] 455000kHz/25600uW/perf…/cost… …   (1950000kHz/364591uW at the top)
perf report: EAS preconditions: asymmetric capacity yes (385..1024), EM 2/2, schedutil 2/2, freq-invariant yes
```

**At 95 s:**
```
selftest: synthetic touch, head 130 ms, tail 500 ms
selftest: head phase=1 boosts=1 cpu4:min=1066000,cur=… cpu0:min=832000,cur=…
selftest: tail phase=2 … cpu4:min=858000 …
selftest: released phase=0 …
selftest: PASS (3/3)
```

**On the first real touch:** `first BTN_TOUCH 1 from sec_touchscreen` and
`first touch boost (head) …`.

The thermal lines that only appear when hot:
- `BIG: … mC: applying the cpu4 cluster cap (hotplug-out 96000 mC)`
- `BIG: cpu4 cluster max capped at 650000 kHz ret=1`
- and the matching `releasing` / `released` lines.

### Read-only checks with adb or the ROM-RAM shell

```
cat /sys/devices/system/cpu/cpu*/cpu_capacity                 # 385 x4, 1024 x4
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies   # starts 1950000
cat /sys/devices/system/cpu/cpufreq/policy{0,4}/schedutil/rate_limit_us     # 5000
ls /sys/kernel/debug/energy_model/                            # cpu0 cpu4
cat /sys/kernel/debug/energy_model/cpu0/ps:1950000/power      # 364591 (uW)
cat /sys/kernel/debug/sched/domains/cpu0/domain*/flags        # SD_ASYM_CPUCAPACITY on the top level
cat /proc/sys/kernel/sched_energy_aware                       # 1
for z in /sys/class/thermal/thermal_zone*; do echo $(cat $z/type) $(cat $z/policy); done
                                                              # BIG power_allocator, others step_wise
cat /sys/class/thermal/thermal_zone*/sustainable_power        # BIG 2000
cat /sys/class/thermal/cooling_device*/type                   # thermal-cpufreq-0/1, thermal-gpufreq-0, thermal-isp-0
cat /sys/bus/platform/devices/11120000.ufs/clkgate_enable     # 1
cat /sys/bus/platform/devices/11120000.ufs/clkgate_delay_ms   # 20
cat /sys/module/ufs_exynos9810/parameters/h8_{enter,exit,cal_err}   # enter/exit rising, cal_err 0
cat /sys/block/sda/queue/scheduler                            # [mq-deadline] none
cat /sys/block/sda/queue/read_ahead_kb                        # 128
cat /sys/module/crownlte_input_booster/parameters/boosts      # rises with each touch
```

**Functional checks**
- **Boost:** touch the screen while sampling
  `cat /sys/devices/system/cpu/cpufreq/policy{0,4}/scaling_min_freq`. Expect
  832000/1066000 right after touch-down, 832000/858000 while the finger is held,
  and back to 455000/650000 0.5 s after release. Or compare
  `cpufreq/stats/time_in_state` before and after a scroll.
- **Thermal:** a sustained 8-thread load. Watch `thermal_zone*/temp` and
  `policy0/scaling_max_freq`. Expect the LITTLE cap to step at 76/81/86 °C, and
  big to be held near the 83 °C control temperature.
  Do this only with someone at the phone. The 110 °C software critical trip and
  the hardware trip remain the backstops.

**Before the trial (read-only, on 4.9):** inventory section 7, in particular
`policy0/scaling_available_frequencies` (expect a 1950000 top) and the thermal
zone policies, as a runtime cross-check of the source evidence above.

## Safety summary

- No CPU frequency or voltage limit goes above what 4.9 uses.
  - LITTLE: 1950 MHz, at the same FVMap voltage, selected by the firmware.
  - big: stays at 1794 MHz (`samsung,max-freq-cap`) on all four cores.
  - Boost and boot QoS requests are minimums, at or below those limits.
- No PMIC, charger or TMU register writes. The hardware thermal trip is unchanged.
- The BIG zone stays on step_wise unless power_allocator has real power actors.
- Every DT property is opt-in. Every feature logs proof at boot or has a self-test.

## Artifacts (gekko)

- **Image:** `panel-bright-work/images/Image-romram-perf-<commit>`
  - built in `/dev/shm/build-perf`;
  - config = `/dev/shm/build-panel-s2dos05-diag/.config` (= `build/romram-6.12.93/.config`
    + IOMMU + S2DOS05 diag) + `INPUT_CROWNLTE_BOOSTER` + `THERMAL_GOV_POWER_ALLOCATOR`
    + `MQ_IOSCHED_DEADLINE`;
  - config copy: `panel-bright-work/images/config-romram-perf-<commit>`.
- **DTB:** `panel-bright-work/dtb/exynos9810-crownlte-romram-v23a-perf.dtb`
  - built with `rom-ram/src/build-dtb.sh` (KSRC = the perf worktree, KO = the perf
    build dir);
  - the preprocessed source is kept next to it as `.dts`.
- sha256 values are in the `.sha256` files next to them and in the report.
