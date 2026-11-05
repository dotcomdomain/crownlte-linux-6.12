# crownlte thermal names expected by the vendor thermal HAL

The source files were extracted read-only with `debugfs dump` from `backups-20261103/partitions/VENDOR.img`. Copies are on gekko in `next-research/vendor-thermal/`.

| File | sha256 |
|---|---|
| `/vendor/etc/thermal_info_config.json` | `14b0f626…` |
| `/vendor/bin/thermal_symlinks` | `347ac270…` |
| `/vendor/etc/init/pixel-thermal-symlinks.rc` | `a8cb6559…` |
| `/vendor/etc/init/android.hardware.thermal-service.pixel.rc` | `23ef9e5a…` |
| `/vendor/bin/hw/android.hardware.thermal-service.pixel` | `e3b8a3c3…` |

## How the HAL finds devices

`pixel-thermal-symlinks.rc` runs `thermal_symlinks` `on boot`. That script does two things:

- For every `/sys/class/thermal/thermal_zone*`, it links `/dev/thermal/tz-by-name/$(cat type)` to the zone.
- For every `/sys/class/thermal/cooling_device*`, it links `/dev/thermal/cdev-by-name/$(cat type)` to the device.

It then sets `vendor.thermal.link_ready=1`, which starts `vendor.thermal-hal`.

The script creates the links **once**, so every device must exist at that point and keep its `cooling_deviceN` number afterwards.

The HAL binary reads these attribute names (from its string table):

- Zones: `temp`, `trip_point_0_temp`, `trip_point_0_hyst`, `policy`.
- Cooling devices: `cur_state`, `max_state`.

`thermal_info_config.json` binds no cooling device to a sensor (there is no `BindedCdevInfo`). So the HAL only reports cooling devices; it never throttles through them.

## Names

| JSON name | Kind | 4.9 | 6.12 (`crownlte-next`) |
|---|---|---|---|
| BIG | zone | exynos_tmu | thermal_of zone node `BIG` ✔ |
| LITTLE | zone | exynos_tmu | `LITTLE` ✔ |
| G3D | zone | exynos_tmu | `G3D` ✔ |
| ISP | zone | exynos_tmu | `ISP` ✔ |
| ac | zone (USB_PORT, Monitor) | charger power_supply "ac" temp zone | **missing** (no charger driver yet) |
| battery | zone | fuel gauge power_supply "battery" temp zone | **missing** (no fuel gauge driver yet) |
| thermal-cpufreq-0 | cdev | acme domain@0 (cpus 0-3) through cpu_cooling idr 0 | **cpu0 policy, LITTLE** ✔ (this branch) |
| thermal-cpufreq-1 | cdev | acme domain@1 (cpus 4-7), idr 1 | **cpu4 policy, BIG** ✔ (this branch) |
| thermal-gpufreq-0 | cdev | gpu_cooling | Mali platform cooling device ✔ (crownlte-mali) |
| thermal-isp-0 | cdev | isp_cooling | **missing** (no ISP driver) |

The 4.9 sources for these names:

- `exynos_cpufreq_init` walks `cpufreq-domain` nodes in DT order (`K49/drivers/cpufreq/exynos-acme.c:1273-1331`) and calls `exynos_cpufreq_cooling_register`.
- That calls `__cpufreq_cooling_register`, which builds `"thermal-cpufreq-%d"` from the idr id (`K49/drivers/thermal/cpu_cooling.c:1068`).

## Implementation

`drivers/thermal/cpufreq_cooling.c` gains `of_cpufreq_cooling_register_named(policy, type)`. It is `of_cpufreq_cooling_register()` with a fixed type string.

The crownlte cpufreq driver (`drivers/cpufreq/exynos9810-acpm-cpufreq.c`):

- Drops `CPUFREQ_IS_COOLING_DEV`, which would name the devices `cpufreq-cpu0`/`cpufreq-cpu4`.
- Registers `thermal-cpufreq-<domain id>` from `->ready`.

The device is still bound to the policy CPU's DT node. So the BIG zone's `cooling-maps <&cpu4 …>` binds as before, and LITTLE has no map, as before.

The device stays registered across CPU offline, like the core's own device. There is no `->exit`: the driver is built in and never unregistered.

Expected log:

```
exynos9810-acpm-cpufreq ...: domain0 (cpus 0-3): cooling device thermal-cpufreq-0 = cooling_deviceN, max_state M
exynos9810-acpm-cpufreq ...: domain1 (cpus 4-7): cooling device thermal-cpufreq-1 = cooling_deviceN, max_state M
cpufreq selftest: policy0 ... cdev=thermal-cpufreq-0/N
cpufreq selftest: policy4 ... cdev=thermal-cpufreq-1/N
```

`max_state` is the number of table levels minus 1. BIG's table is capped at 1794 MHz (`samsung,max-freq-cap`), so its `max_state` is smaller than on 4.9. The HAL only reports this value.

## Open items

- **`ac`, `battery`, `thermal-isp-0` are missing.** The HAL string table contains `ThermalHAL could not be initialized properly.` and `Could not find`. Whether a missing configured sensor or cdev is fatal to this build of the HAL is not verified; the binary was not decompiled.
  - The safe fix is to provide the names: power_supply drivers for `ac`/`battery`, and a stub `thermal-isp-0` cooling device with `max_state` 0.
  - The other option is to drop the entries from the JSON through a vendor overlay.
- **Netlink events.** `CONFIG_THERMAL_NETLINK` is off. The HAL references `THERMAL_GENL_*` and falls back to polling or uevents.
- **Trip writes.** `CONFIG_THERMAL_WRITABLE_TRIPS` is off. That matters only if the HAL tries to write `trip_point_0_temp` for a `user_space`-policy zone. No zone uses `user_space`; the default governor is step_wise.
