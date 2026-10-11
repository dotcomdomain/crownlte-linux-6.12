# Exynos9810 CPUFreq source adapter

This adapter uses the existing firmware and the imported original CAL/ASV
tables. It has no enabled device-tree node. Compilation and policy tests are
source milestones, not permission to enable scaling on the phone.

The platform compatible is `samsung,exynos9810-acpm-cpufreq`. Its root requires
`samsung,mif-devfreq`, a phandle to a real registered MIF devfreq supplier.
That supplier must use standard Hz frequency values, expose actual hardware
frequency through `get_cur_freq`, and enforce its own INT/other dependencies.
The CPU adapter applies standard `DEV_PM_QOS_MIN_FREQUENCY` requests in kHz.
It retains a supplier device reference and a managed device link.

There must be two enabled child nodes, little then big. Each contains four
`cpus` phandles, checked against original physical affinities0..3 and100..103.
`samsung,cal-id` is0x0b040002 or0x0b040003 respectively. Each child requires
`min-freq`, `max-freq`, `samsung,mif-perf-table` CPU/MIF pairs in descending
order and `samsung,mif-ect-domain` naming its original MINLOCK domain.
Optional `policy-max` sets the initial user policy cap, permitting subsequent
`scaling_max_freq` changes. Frequencies in these properties are kHz.

The original crownlte DT performance pairs are recorded in the policy report.
The actual phone has11 little-cluster levels from455000 to1950000kHz, and15
big-cluster levels from650000 to2314000kHz. Its big user-policy default is
1794000kHz. The live ECT MINLOCK block contains CPUCL1 but no CPUCL0 domain.
The original ACME driver skips that absent little-cluster skew constraint;
the adapter preserves the DT little performance floor and requires the real
big ECT table. All enabled big levels have exact ECT rows. The helper chooses
a conservative nearest-higher skew row for other hypothetical rates.

Probe requires actual CAL/FVMAP and firmware channel5 readiness. OPP voltage
values come from the real ASV/FVMAP table, with no fixed voltage fallback.
The frequency transition goes through firmware's existing paired DVFS command.
Before a CPU transition raises its MIF requirement, the adapter raises its
real request, updates the real supplier and checks actual MIF frequency.
It lowers CPU frequency before releasing a MIF floor. A failed release keeps
the conservative old requirement and does not report a completed CPU change
as failed. Requests from the other cluster and GPU aggregate through the
normal device QoS framework.

A firmware error after a raised MIF requirement retains the higher floor.
A failed acknowledgement cannot prove that the CPU stayed at its old rate.
Probe rejects pre-existing CPU OPP tables, and cleanup removes only the
frequency entries successfully added by this provider. Probe also checks
actual sharing membership because the OPP helper can silently skip failed
member allocations. Platform integration must keep it the sole CPU OPP creator.

CPUFreq registers standard cooling devices and the original100000ns latency.
Actual TMU sensors, calibrated thermal trips/cooling maps, MIF/INT devfreq,
legacy UFC/sysfs behavior, scheduler energy models, hotplug and suspend
integration remain incomplete. No driver readiness or cooling device alone
establishes working thermal protection. Keep this platform absent or disabled
until those prerequisites have been implemented and validated.

Validation so far: real ARM64 object compilation with LLVM18 and WERROR;
131123 production-helper, transition-order and partial-cleanup cases under ASAN/UBSAN, using
the actual DT performance pairs and live ECT skew values. Firmware, sensors,
concurrent supplier teardown and policy/hotplug behavior need runtime tests.
