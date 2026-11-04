# crownlte Mali-G72 (kbase r38p1-01bet0) on 6.12: stage 1

## What the vendor blob needs

- `/vendor/lib64/egl/libGLES_mali.so`: Bifrost DDK `r38p1-01bet0` (MaliG72_r38p1_LT_T).
  It sends UK **11.35** in `KBASE_IOCTL_VERSION_CHECK`.
- It uses 28 sized kbase ioctls (type 0x80) plus Samsung-private nr 42, `MALI_EXYNOS_IOCTL_EHMP` (0x4004802a).
  All 28 encodings were checked against `include/uapi/gpu/arm/bv_r38p1` (host program over the UAPI headers vs `roadmap/vendor-abi/raw/libGLES_mali-ioctls.txt`).
  Five scan hits with sizes 862/1886/1919/2353/3377 are data constants, not ioctls.
- Device node: `/dev/mali0`.

## Source provenance

`drivers/gpu/arm/bv_r38p1` and `include/uapi/gpu/arm/{bv_r38p1,exynos}` are byte-identical to the kernel running on the phone:
ExyHyperBrick/android_kernel_samsung_exynos9810 commit 8665533d (`/root/crownlte-hid-kernel` on gekko), `MALI_RELEASE_NAME "r38p1-01bet0"`.
The tree hash is the sha256 of the sorted `sha256sum` list of its 370 files: `28c4f395dc09ee159c3a3759a709b98bfc958d444372733c1806fd608271d94a`.

ARM's public Bifrost release (`r38p1-01eac0`) is a different drop and was not needed.
Samsung's `01bet0` copy is the one the blob was built against, and it already carries the Samsung core hooks.

Commits:

| Commit | Content |
|---|---|
| 6cefef4ab | Unmodified import |
| ed42f7b98 | 6.12 API adaptation, from the old graphics line: 7f0f09939, d66ecbced |
| 6dac883a3 | Platform layer `drivers/gpu/arm/exynos9810` plus Kconfig wiring |
| ca76426f2 | dtsi node |
| 6b9bb5f8b | Diag enable and self-test |

## Platform layer (replaces the 4.9 gpex stack)

| 4.9 | here |
|---|---|
| gpexbe_pm: genpd `pd-embedded_g3d` / `cal_pd_control(0xb1380008)` | **not switched.** Probe needs PMU 0x14064040/44 = 0xf; GPU never powered off at PMU level |
| gpexbe_clock: `cal_dfs_set_rate(g3d_cmu_cal_id)` | same, `samsung,cal-id = <0x0b040004>`, workqueue, CMU readback `cal_dfs_get_rate` |
| gpu_dvfs_governor "interactive", 30 ms polling | same algorithm on kbase `MALI_MIDGARD_DVFS` metrics |
| gpu_tmu / gpu_cooling `thermal-gpufreq-0` | thermal_of cooling device of the same name, bound to the G3D 88 C passive trip |
| gpex_qos MIF min lock (gpu_dvfs_table col 5) | dev_pm_qos MIN_FREQUENCY on the MIF devfreq (`samsung,mif-devfreq`) |
| gpex sysfs `/sys/kernel/gpu/*` | gpu_clock, gpu_freq_table, gpu_max_clock, gpu_min_clock, gpu_governor, gpu_busy, gpu_model (+ gpu_dvfs_debug) |
| mali_exynos_ioctl (AMIGO, GTS, HCM, CMAR, SLSI_*, INTERACTIVE_BOOST, EHMP) | same return semantics, no actions (EHMP boost ignored) |
| ION `ion_cached_dmabuf` | dma-buf `exp_name` ending in `-uncached` means skip CPU sync |
| IFPO, GTS, TSG/AMIGO, CL boost, BTS MO, LLC, secure rendering | not ported (secure rendering hooks return -ENOSYS as 4.9's dummy) |

### Why the power domain is never switched

`cal_pd_control(0xb1380008, on)` (4.9 and 6.12 cal-if.c) runs these steps:

1. `exynos_acpm_update_reg(3, 0x2a, 0xc0, 0xc0)`: an ACPM firmware RMW of a PMIC register, near the S2MPS18 B6CTRL (BUCK6, vdd_g3d).
   Type 3 is not one of the documented S2MPS18 slaves (TOP 0, PMIC 1, RTC 2, DEBUG 0xf).
2. `exynos_pd_tz_restore(need_smc 0x17410204)`: an EL3 DTZPC restore SMC.
3. The PMUCAL `embedded_g3d_on` sequence: CMU_G3D mux user, CONFIGURATION = 0xf, wait for STATUS.

The 6.12 ACPM PMIC write gate refuses step 1 (`exynos_acpm_update_reg` always returns -EPERM), so `cal_pd_control` returns -EPERM before any PMU write.
Stage 1 therefore depends on SBoot leaving the domain on. The probe log shows whether it does.

## Register writes (all paths)

- **ACPM DVFS:** `cal_dfs_set_rate(0x0b040004, kHz)` only, with the DT levels 572/546/455/338/299/260 MHz.
  Each level must be an exact CAL rate inside CAL min/max with a nonzero FVMap voltage (`cal_dfs_is_ready`).
  The firmware programs PLL_G3D and vdd_g3d.
- **MIF floor:** dev_pm_qos requests (MIF devfreq then issues its own ACPM DVFS).
- **GPU block (0x17500000), through unmodified kbase:**
  - GPU_COMMAND soft reset and cache flush
  - IRQ mask/clear
  - SHADER/TILER/L2 PWRON/PWROFF
  - MMU AS programming for user contexts
  - job slot registers
  - The self-test adds one GPU_COMMAND CLEAN_INV_CACHES.
- **No PMU, PMIC, CMU or TZPC writes.** PMU, CMU_G3D and PMIC B6CTRL are only read.

## Expected diag log (delayed-ufs-diag DTB)

Probe:

```
mali exynos9810: CAL level N: <kHz> kHz <uV> uV          (one per ECT level)
mali exynos9810: GPU_ID 0x6221xxxx: Mali-G72 arch 6.2.2 r?p? status ?
mali exynos9810: G3D domain on (PMU cfg 0xf status 0xf), CMU mux 0x..1., <kHz> kHz, 6 DVFS levels
mali exynos9810: DVFS 260000-572000 kHz, period 30 ms, cooling thermal-gpufreq-0
mali selftest: armed for 109 s uptime
mali exynos9810: MIF floor through <devfreq-mif device>   (first rate change)
```

Self-test at about 109 s:

```
mali selftest: start: PMU G3D cfg 0xf status 0xf (CAL pd status 1), CMU_G3D mux ..., clock ... kHz, kbase power on/off a/b
mali selftest: S2MPS18 B6CTRL (vdd_g3d) read 0: 0x..
mali selftest: power up: wait 0, ... us, GPU powered 1
mali selftest: GPU_ID 0x6221.... (probe ...), L2 present 0x1 ready 0x1, shader present 0x...(18 cores) ..., tiler present 0x1 ready 0x1, GPU_STATUS 0x0, IRQ_RAWSTAT ...
mali selftest: cache clean+invalidate: 0, ... us: PASS
mali selftest: MMU: <n> address spaces (8 expected), active mask 0x0, MMU_IRQ_RAWSTAT 0x0 (...)
mali selftest: dvfs 455000 kHz: set 0, CMU readback 455000 kHz, GPU_ID ..., ... us: PASS
mali selftest: dvfs 338000 kHz: set 0, CMU readback 338000 kHz, GPU_ID ..., ... us: PASS
mali selftest: dvfs restore ... kHz: set 0, CMU readback ... kHz
mali selftest: job submission: not run (no in-kernel job client)
mali selftest: power down: kbase GPU off after N ms, power on/off ..., PMU G3D cfg 0xf status 0xf (domain kept on by design)
mali selftest: PASS (0 failures, N ms)
```

Failure lines that mean "no /dev/mali0, nothing written":

- `G3D power domain is off (PMU cfg .. status ..); stage 1 never powers it`
- `CMU_G3D MUX_CLKCMU_EMBEDDED_G3D_USER .. selects OSCCLK`
- `CMU readback .. kHz is not a CAL level`
- `GPU_ID .. is not a Mali-G72`
- deferred probe reason `CAL 0xb040004 not ready (ACPM DVFS / FVMap)`

## Stage 2 (not done)

1. G3D power-domain control. Needs both of these:
   - A sealed ACPM write-policy entry for exactly the `cal_pd_control` update. Identify type 3 first; the Samsung DT says vdd_g3d = S2MPS18 BUCK6, `regulator-always-on`.
   - `exynos_pd_tz_save`/`restore` with need_smc 0x17410204.

   Then: runtime PM (4.9 gpu_runtime_pm_delay_time 50 ms), IFPO and suspend support. Remove the suspend veto.
2. Userspace job test with the vendor blob (`/dev/mali0` open, VERSION_CHECK 11.35, GET_GPUPROPS, a tiny GLES draw).
3. MIF/CPU QoS columns 6-7 of gpu_dvfs_table, BTS MO, GTS/AMIGO ioctl data (perf only).
