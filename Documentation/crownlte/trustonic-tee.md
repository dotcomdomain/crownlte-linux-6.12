# crownlte Trustonic (Kinibi / MobiCore) TEE on 6.12

Status: build-only. Branch `crownlte-tee` on integration 31caf697c. Not run on the phone.

Why it matters: LineageOS uses FBE v2 with keymaster-wrapped keys. keymaster@3.0
(`keystore.mdfpp`) is a Trustonic TA. Without the TEE, vold `init_user0` cannot unlock /data.

Paths below: K49 = /root/crownlte-hid-kernel/drivers/gud/gud-exynos9810/MobiCoreDriver,
T = drivers/gud/gud-exynos9810/MobiCoreDriver in this tree.

## 1. What the integration images do today (q18-era, 31caf697c)

`CONFIG_TRUSTONIC_TEE=y` (Kconfig default y) plus SECURE_OS_CONTROL and SECURE_OS_BOOSTER_API
are built in. The 6.12 DT has no `samsung,exynos-tee` node, so the platform driver registers
and never probes. The q18 serial log (trial-acpm-q18-20261104/serial-ready.bin, line 645)
has exactly one line:

    Trustonic TEE: mobicore_init: MobiCore mcDrvModuleApi version is 6.3

The driver made no SMC, donated no memory, requested no IRQ and created no /dev/mobicore*
node. One latent bug existed in that state: writing
`/sys/devices/system/sec_os_ctrl/migrate_os` (late_initcall, root 0600) reached
`mc_switch_core_ctrl()` with an uninitialised kthread worker. 4.9 would hang forever there;
6.12 would oops. It is now refused with -ENODEV until the TEE has started.

## 2. 4.9 sequence from probe to userspace (K49 file:line)

`module_init(mobicore_init)` (main.c:796) → `platform_driver_register` (main.c:775) →
`mobicore_probe` (main.c:624) for DT `tee { compatible = "samsung,exynos-tee";
interrupts = <0 233 0>; }` (exynos9810-crownlte_eur_open_26.dts:10318).
`MC_DELAYED_TEE_START` is not defined anywhere, so the TEE starts inside probe (main.c:714),
at device_initcall, long before init runs.

Probe, NWd only:
- `mc_fastcall_init`: creates kthread `tee_fastcall` (nice -20) pinned to CPU0
  (fastcall.c:541) and the cpuhp teardown callback (fastcall.c:548). Every SMC below is
  issued from this thread as raw `smc #0` with x0..x3 (fastcall.c:179).
- `nq_init`: MCI buffer `__get_free_pages(GFP_USER|__GFP_ZERO, get_order(buf_len))`
  (nq.c:681,683). This is normal cacheable linear-map RAM. It holds two 64-entry
  notification queues, `mcp_buffer`, 16 IWS slots and `mcp_time`.
- `mcp_init`, `iwp_init`: lists and mutexes only.
- `mc_logging_init`: trace buffer `__get_free_pages(GFP_KERNEL|__GFP_ZERO, 6)`, i.e.
  256 KiB contiguous (logging.c:31,252).
- `mc_scheduler_init`, `device_admin_init`: creates `/dev/mobicore` (class trustonic_tee,
  minor 0).

`mobicore_start` (main.c:319). Fastcall IDs are the ARM64 STD64 set (mci/mcifc.h:28-36).

| # | Call | SMC x0 | Args | Where |
|---|------|--------|------|-------|
| 1 | MC_FC_MEM_TRACE | 0xFF00000A | log PA lo, PA hi, 0x40000 | logging.c:219, fastcall.c:667 |
| 2 | MC_FC_INIT | 0xFF000001 | MCI PA lo; nq_info=(PA hi<<16)\|q_len; mcp_info=(off<<16)\|len | nq.c:567, fastcall.c:607 |
| 3 | init_values written into MCI | — | IRQ hwirq 265, time ofs/len, IWS ofs/size | nq.c:575-594 |
| 4 | MC_SMC_N_SIQ | 4 | — | nq.c:597 |
| 5 | loop: MC_FC_INFO(MCI_VERSION), then 10× N_YIELD (3) while NOT_INITIALIZED, sleep 100-500 µs; **no timeout** | 0xFF000002 / 3 | — | nq.c:605-641 |
| 6 | `request_irq(SPI 233, RISING, "trustonic")` + kthread tee_irq_bh | — | — | nq.c:650 |
| 7 | kthread `tee_scheduler` on cpu_coregroup_mask(0); it issues N_SIQ/N_YIELD when notified | 4 / 3 | — | scheduler.c:192-232 |
| 8 | MCP `GET_MOBICORE_VERSION` through the NQ (notify, then scheduler N_SIQ) | — | — | main.c:368, mcp.c:447 |
| 9 | feature flags from version_mci; `/dev/mobicore-user` (minor 1) | — | — | main.c:461 |
| 10 | `mc_switch_core(1)` → MC_FC_SWAP_CPU | 0xFF000036 | MPIDR 0x81000100 (CPU_IDS[1]) | main.c:723, fastcall.c:775, platform.h:70 |

Nothing else happens until userspace opens /dev/mobicore. mcDriverDaemon then loads
drivers/TAs, the auth token comes from /efs, and storage requests are served. The kernel
has no TA-loading, RPMB or storage path of its own. No in-kernel client calls
mc_open_session/teec_* (checked with git grep; only fingerprint/TUI use them on 4.9 and
neither is built). The 4.9 `ufs-srpmb` (SPI 460) is a separate driver and is not ported.

## 3. 6.12 port vs 4.9 (diff -ru of the two gud-exynos9810 trees)

Everything on the SWd-facing path is unchanged. mci/*.h, public/*.h, nq.c init/start,
logging, mcp_cmd, fastcall IDs, the `smc #0` wrapper, CPU_IDS, NONBOOT_LITTLE_CORE and
LOG_BUF_ORDER are all identical. Allocation size, order, GFP flags and cacheability of both
donated buffers are therefore identical.

| Difference | Where (T) | Risk |
|---|---|---|
| timespec → `ktime_get_real_ts64`/`ktime_get_raw_ts64` | nq.c nq_update_time | none (same values) |
| `DEFINE_KTHREAD_WORKER` → explicit `kthread_init_worker` in mc_fastcall_init | fastcall.c | none on the start path; it caused the sec_os_ctrl latent bug above (now guarded) |
| Kinibi user mappings: `pin_user_pages_remote(FOLL_LONGTERM)` instead of get_user_pages(FOLL_CMA); unpin with dirty | mmu.c | not used at init. With a daemon, CMA pages migrate before pinning (4.9 pinned them in place) |
| dma_buf `_unlocked` wrappers, `vm_flags_set`, `MAX_PAGE_ORDER`, `access_ok`, `freezing()`, shash flags, class_create | client/mmu/user/mcp/session.c | API only |
| CPU_IDS guard `CONFIG_SOC_EXYNOS9810` → `CONFIG_TRUSTONIC_EXYNOS9810` | platform.h | none (same table) |
| cpuhp: 4.9 (<4.10) used a CPU notifier with an ONLINE hook (`mc_cpu_online` moves the SWd back to CPU1). 6.12 has only the teardown hook | fastcall.c:597 | only on CPU hotplug: the SWd stays on whichever core it was moved to. Not exercised in diag |
| sec_os_booster: freq_qos on cpufreq policy 4; fails -ENODEV if no policy at late_initcall | sec_os_booster.c | none for init. No booster means no SWd migration to big cores |
| Logical CPU numbering | DT cpus | identical. Both DTs list reg 0,1,2,3,0x100..0x103 in the same order, so logical CPU1 is the same physical core |
| DT node | (absent) | added in this branch |
| Wall clock at TEE start | — | same as 4.9: hctosys is late_initcall in both, so the TEE starts before RTC time is set. Later updates come from the scheduler (f_time) |

Memory map: the 6.12 /memory node excludes the same secure holes (0xBC800000-0xC0000000,
0xE0000000-0xE1900000). The kernel already runs on all remaining RAM in every trial, so
the page allocator cannot hand secure memory to MC_FC_INIT.

## 4. Changes in this branch (NWd only; the SWd sees the 4.9 sequence)

1. **Start in a kthread** (`tee_start`, main.c `mobicore_start_thread`). Same calls, order
   and arguments as K49 main.c:714-723. All SMCs still come from tee_fastcall on CPU0. If
   the 4.9 unbounded INFO loop never sees INITIALIZED, or an SMC never returns, the boot no
   longer hangs before USB/serial. A RECOVERY hang means an Odin restore (see the lesson
   in the trial notes). A failed start no longer unwinds probe. 4.9 then ran `nq_exit()`
   and freed the MCI pages after they had been registered with MC_FC_INIT, a
   use-after-free by the SWd. Now nothing donated is ever freed.
2. **MCI version reject returns -EPROTO.** 4.9 jumped to err_version with ret == 0, so it
   reported success after tearing down and then still sent SWAP_CPU.
3. **Guards:** `mc_switch_core`, `mc_switch_core_ctrl` and the hotplug teardown refuse
   until `mc_tee_started()`.
4. `.suppress_bind_attrs = true`: no remove() exists, and a sysfs rebind would re-send
   MC_FC_INIT.
5. **Bookkeeping:** start stage, INFO poll count, per-ID fastcall counters, MCP command
   counters, S-SIQ IRQ count, and a cached-version accessor
   (`mcp_get_cached_version`, never sends MCP).
6. **DT:** `trustonic_tee` (disabled) in exynos9810-crownlte-clocks.dtsi, verbatim from 4.9.
   Enabled with `samsung,tee-selftest` only in exynos9810-crownlte-delayed-ufs-diag.dts.

## 5. Self-test ("tee selftest:", ~110 s, diag DT only)

It reads only existing driver state. It issues no SMC, no MCP command, no session and no
TA. It prints:
- start state (ok/PENDING/FAILED), stage, nq stage, INFO polls, start time window,
  core-switch result, active core
- product_id and the version_* fields cached at start; version_nwd must be 0x00060003
- feature flags; MCI PA/order; trace buffer PA/size, header version/length/head
- IRQ virq/hwirq (expect 265) and count
- fastcall counts per ID; MCP total vs GET_VERSION; clients/sessions (expect 0)
- /dev/mobicore and /dev/mobicore-user: class device registered, and the devtmpfs node
  (if /dev is devtmpfs in the diag initramfs) with matching major:minor

PASS requires: start ok; API 6.3; core switch 0; both devices registered and no wrong
node; zero unknown fastcalls; INIT ≤1, MEM_TRACE ≤2, SWAP_CPU ≤1; MCP commands are only
the single GET_VERSION; 0 clients and 0 sessions.

Expected PASS fingerprint: init=1 mem_trace=1 swap_cpu=1 other=0, mcp total=1, active_cpu=1,
mci 0x00010006 or similar (Kinibi 400 = MCI 1.5/1.6).

## 6. Do not, during kernel-only trials

- Do not start mcDriverDaemon or keymaster/gatekeeper/vold against the real /efs or /data.
- Do not read /sys/kernel/debug/trustonic_tee/* (some files issue MC_FC_INFO).
- Do not write sec_os_ctrl/migrate_os.

## 7. Path to keymaster and vold (proposal, nothing run)

Each stage is a separate trial and a separate go/no-go.

- **T1 (this branch):** kernel init and self-test only. Reversible: SoC reset clears all
  TEE state. There is no persistence path.
- **T2:** RAM-namespace userspace (rom-ram style), EFS mounted **read-only** from a copy
  (or `ro,noload`), `/data/vendor/mcRegistry` on tmpfs. Start mcDriverDaemon only. It
  loads the vendor drivers/TAs from /vendor/app/mcRegistry (read-only), and the auth
  token is read only. Observe `/dev/mobicore` open, driver loads, no errors. Risk: the
  daemon or a secure driver may try to write tokens/containers. With RO EFS and tmpfs
  registry those writes fail, but the failure modes are unknown. Use strace and an
  inotify-style audit first.
- **T3:** plus the keymaster@3.0 service (keystore.mdfpp) with a tmpfs keystore. Query
  `getHardwareInfo` / `addRngEntropy` only. No key generation, no key upgrade.
- **T4 (first data-bearing, needs explicit user go):** vold `init_user0` unwrap against
  the real /data metadata, with /data mounted read-only (or only the key directory copied
  to tmpfs) and the FMP/blk-crypto-fallback known-answer test done first.
  **Irreversible risk:** keymaster *key upgrade* happens if os_version/patchlevel differ.
  vold then rewrites the wrapped keys, and 4.9 keymaster may no longer unwrap them. The
  6.12 boot must present the same os_version/patch level as the running 4.9/Lineage, or
  upgrade must be prevented.
- **T5:** RW Lineage boot. Only after T4 shows matching ciphertext and no key upgrade.

Irreversible steps across the plan: any RPMB write (anti-rollback counters, Samsung
`ufs-srpmb`, which is not ported and must stay so), keymaster key upgrade or wrapped-key
rewrite in /data, any write to /efs (TEE auth token, `/efs/TEE`), and any SWd secure
storage (TbStorage) commit. T1 performs none of them.
