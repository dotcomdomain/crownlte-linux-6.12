# Exynos9810 CPUFreq (drivers/cpufreq/exynos9810-acpm-cpufreq.c)

The driver ports the frequency-scaling core of Samsung's 4.9 exynos-acme
(crownlte 4.9 kernel, drivers/cpufreq/exynos-acme.c) onto the existing ACPM
firmware and the imported CAL/ECT/FVMap code.

## Source of every number

| Item | 4.9 acme | Here |
|------|----------|------|
| Levels | `cal_dfs_get_lv_num()` + `cal_dfs_get_rate_table()` | Same, through `cal_dfs_get_rate_asv_table()` |
| Voltages | `cal_dfs_get_asv_table()` (OPP only) | Same (OPP only). Firmware picks the real voltage |
| Range | CAL min/max, DT `min-freq` raises min, DT `max-freq` replaces max | DT `max-freq` can only lower the max, because 6.12 `cal_dfs_set_rate()` refuses rates above the CAL max |
| Initial user cap | DT `policy-max` | Same, applied to `policy->max_freq_req` in `.ready` |
| Hard cap | none | `samsung,max-freq-cap`, so the table stops there |
| Transition | `cal_dfs_set_rate()` FREQ_REQ | Same |
| `get()` | `cal_dfs_get_rate()` CMU readback; last rate when CAL says 0 | Same |
| Latency | 100 us | 500 us, from the measured 0.39-0.42 ms ACPM round trip |

crownlte DT values (4.9 `cpufreq/domain@0`, `domain@1`) are in
`exynos9810-crownlte-delayed-ufs-diag.dts`:

| Cluster | min-freq | max-freq | Other |
|---------|----------|----------|-------|
| LITTLE (`0x0b040002`) | 455000 | 1950000 | |
| big (`0x0b040003`) | 650000 | 2314000 | `policy-max` 1794000 |

The diag DT also sets `samsung,max-freq-cap = <1794000>` on big until thermal
throttling has been proven on hardware.

## MIF

4.9 tied CPU levels to MIF floors through the DVFS Manager:

- `mif-perf` is a DT performance table.
- `mif-skew` is ECT MINLOCK `dvfs_cpucl0`/`dvfs_cpucl1`, marked as chipset
  guidance.

With `samsung,mif-devfreq`, both floors are enforced through device PM QoS
(the earlier adapter's behaviour). Without it, nothing scales MIF:

- At probe the driver reads the MIF rate once from `samsung,mif-cal-id`
  (`dvfs_mif`, 0x0b040000).
- It drops every level whose ECT skew requirement is above that rate. An
  unreadable MIF (0) satisfies no non-zero requirement.
- A cluster without an ECT MINLOCK domain gets no skew constraint, as in acme.
- The perf table is ignored in this mode.

## Thermal

The cpufreq core registers each policy CPU node with `#cooling-cells` as a
cooling device. The BIG zone maps its passive `big-control-temp` (83 C) trip
to cpu4-7 with no limits. 4.9 used power_allocator there, which needs an
energy model, so this uses the default step_wise. 4.9 LITTLE maps were all
`<0 0>` (never throttle) and are not carried over.

## Not ported

- DVFS Manager
- UFC and legacy sysfs
- PM QoS classes
- Boot QoS: `pm_qos-booting`, 40 s at max
- Hotplug notifiers
- EMS/energy table
- exynos-ocp (`max-freq-wo-ocp` 1794000)

## Diagnostic

`samsung,cpufreq-selftest` on the cpufreq node schedules one run at about
100 s uptime. Every line starts with `cpufreq selftest:`. The run:

1. Reports the probe outcome (also for failed or deferred probes).
2. Reports the MIF and BIG thermal readings.
3. Lists each policy, the CAL table and the final table.
4. Pins each policy through `freq_qos` min=max at the minimum, middle and
   top levels, then at the 4.9 `max-freq` (must clamp to the top). It
   checks `policy->cur` and the CMU readback for each one.
5. Removes the requests.
