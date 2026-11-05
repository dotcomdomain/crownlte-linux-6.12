# crownlte display IOMMU (DPU SysMMU)

Branch `crownlte-followups`: the SysMMU work of `crownlte-dpu-iommu` (404c42dd6..d41526fcb, on
display-s2 0263a2095) ported onto the HWC display driver of `crownlte-display-flipfix`
(23b0b1202: adopt mode, WIN_CONFIG for HWC, AFBC, per-window bounce copies into decon-pool).
**Build-only, not run on the phone.**

- **Opt-in by DT.** The SysMMU path is used only when decon_f has `iommus` (and the three
  `sysmmu_dpu` nodes are enabled). No shipped DTB does that. Without `iommus` the driver runs the
  default path unchanged: contiguous buffers scanned directly, scattered ones bounce-copied into
  decon-pool (per-window slot pairs).
- Test DTB (not in the tree, derived from the ROM-in-RAM v18 DTB):
  `panel-bright-work/dtb/exynos9810-crownlte-romram-iommu.dtb`. Its diff against v18 is listed
  below.
- `CONFIG_EXYNOS_IOMMU=y` is in `crownlte_android_defconfig`. It binds nothing without the DT
  nodes (they stay `disabled` in exynos9810.dtsi).

## Why the display needs it

- The vendor gralloc allocates HWC layers from `/dev/dma_heap/system` (VENDOR-ABI.md), which
  hands out scattered pages. A DPP channel takes one 32-bit base address per plane.
- Without the SysMMU the DECON driver copies every scattered layer into a contiguous decon-pool
  buffer on each frame (about 11.5 ms for the 17.3 MB client target on ROM-in-RAM v12, a 192 MiB
  pool at 0xe4000000 for the keyguard's four layers). With the SysMMU the DPP fetches the
  gralloc buffer in place: no copy, no pool budget.

## Hardware (4.9 crownlte_eur_open_26 DT + 4.9 drivers/iommu/exynos-iommu*.{c,h})

| Instance | Non-secure SFR | Secure SFR (never touched) | IRQ NS (secure) | Ports | 4.9 qos |
|---|---|---|---|---|---|
| sysmmu_dpu0 | 0x160a0000 (0x9000) | 0x160d0000 | SPI 211 (210) | VGRF, VGF | 15 |
| sysmmu_dpu1 | 0x160b0000 (0x9000) | 0x160e0000 | SPI 213 (212) | G0, VG0 | 15 |
| sysmmu_dpu2 | 0x160c0000 (0x9000) | 0x160f0000 | SPI 215 (214) | G1, VG1, WB | 15 |

In 4.9 all three are in the DSIM device's `iommus` (`iovmm_activate()`), and DECON maps buffers
through that one IOVA space. Here all three are in decon_f's `iommus` and share one DMA domain.

## `samsung,exynos9810-sysmmu` in exynos-iommu (commit "iommu/exynos: Exynos9810 SysMMU v7 mode")

| Item | 4.9 (9810) | mainline exynos-iommu | 9810 mode |
|---|---|---|---|
| Register layout | v7 non-VM: PT_BASE 0x0c, FLUSH 0x10, FLUSH_RANGE 0x18, START/END 0x20/0x24, INT 0x60/0x64, FAULT 0x70/0x78 | `sysmmu_v7_variant`, same offsets | same offsets; probe refuses unless VERSION major = 7 without VCR (VM layout) |
| CFG | `(CFG & ~0x301F1F8C) \| QOS_OVERRIDE(bit 11) \| QOS(15)<<7` | `QOS(15) \| FLPDCACHE \| ACGEN \| EAP` | **4.9 value**, non-mask bits from the CFG the bootloader left |
| CFG_EAP / PTE prot bits | off / none | on / v5 encoding | **off / none** (4.9 PTEs bit for bit) |
| TLB invalidation | FLUSH_RANGE, no block | block + FLUSH_ENTRY | **4.9 range flush, no block** |
| Fault IRQ | panics | panics unless a handler accepts | **never panics**; rate-limited 4.9 fault decoding; the master's handler is called |
| Power | DPU genpd | runtime PM via device links | runtime PM via device links; PMU DPU_STATUS (0x14064024) must read 0xf (`samsung,pd-status`) before any SFR access |
| Bootloader state | n/a | ignored | logged; **an instance SBoot left translating (CTRL bit 0) is refused** |
| Reserved direct maps | n/a | none | `.get_resv_regions = iommu_dma_get_resv_regions` |
| IOVA aperture | 32 bit | `~0UL` | `U32_MAX` |
| TLB way/port tuning | applied | n/a | not applied (performance only) |

## Handover: the SBoot splash never fetches through an unmapped IOVA

1. DT: `iommu-addresses = <&decon_f 0x0 0xcc000000 0x1400000>` on SBoot's `lcd_buffer`
   reservation (framebuffer@cc000000, no-map), which is listed as decon_f's second
   `memory-region` (index 0 stays decon-pool, the CMA pool). IOVA == PA, so the IOMMU core treats
   it as a direct mapping.
2. SysMMU probe: PMU check, bootloader-state log/refusal, version check. No SFR writes.
3. decon_f `dma_configure` → `iommu_setup_default_domain()` allocates the DMA domain, **maps
   0xcc000000..0xcd3fffff 1:1 before attaching it**, then attaches. The driver core has already
   runtime-resumed the SysMMUs for the probe (fw_devlink supplier pre-activation), so the attach
   most likely enables translation here, while SBoot's window keeps scanning 0xcc000000:
   identity, same memory.
4. DECON probe, `dpu_iommu_init()`: takes the DMA domain, sets the max segment size to UINT_MAX
   (one IOVA run per buffer), checks `iommu_iova_to_phys()` over every MiB of fb0 and **refuses
   to probe unless every probe is 1:1**, then installs the fault handler. If probe fails, the
   driver core drops the suppliers and the SysMMUs go back to bypass; SBoot's scanout is untouched.
5. `dpu_adopt()`: an enabled SysMMU is accepted only with the IOMMU (without it, CTRL bit 0
   still refuses adoption as before). `dpu_iommu_enable()`: if the SysMMUs are still in bypass
   and the TE trigger runs, mask it and wait 34 ms (4.9 `decon_reg_init()`), then
   `pm_runtime_enable` + `pm_runtime_resume_and_get` (DECON keeps this reference, so the
   SysMMUs stay on after probe), read CTRL of all three back through read-only views of the
   `iommus` nodes and refuse unless all translate; unmask the trigger if it was masked.
6. fb0 takeover commits the boot window at 0xcc000000 = identity IOVA.

## Buffers

- **HWC dma-bufs (`dpu_iova_get()`)**: attached and mapped once on decon_f, kept in a per-dma-buf
  IOVA cache (LRU, at most 32 entries). Every later frame that scans the same dma-buf reuses the
  IOVA (4.9 did the same through `ion_iovmm_map()`). A commit counts as a user of the entry
  until it is freed, which happens only after a later update latched (d->active / d->stale), so
  an IOVA is never unmapped while a window may still fetch it. An unused entry is unmapped when
  the cache holds the last reference to its dma-buf (everyone else closed it) or when it falls
  off the LRU. CPU writers keep the dma-buf contract: the exporter syncs every mapped attachment
  at `DMA_BUF_IOCTL_SYNC` end.
- If a dma-buf cannot be mapped to one IOVA run below 4 GiB, that frame falls back to the old
  per-commit import and the bounce copy (bounce slots then come from iommu-dma, also fine).
- In-kernel buffers (fb0 wrap, self-test and display-matrix buffers) are not cached; they hand
  out their existing DMA address: identity IOVA for fb0, the IOVA `dma_alloc_wc()` returned for
  the others.
- Fault handler (`dpu_iommu_fault`, hard IRQ): count, remember the IOVA, mask the TE trigger
  and latch `trig_fault_inhibit` under `trig_lock` (all trigger read-modify-writes, including
  the panel command window, take it), log. While the inhibit is set, every restore of a saved
  trigger value (command window end or idle timeout, adopt, SysMMU enable) keeps the trigger
  masked; only a commit's own unmask in `dpu_apply()` clears it.
- Recovery (`dpu_pending_resolve()`, under `hw_lock`): if the fault masked the trigger while a
  shadow update waited for it, that update could never latch, and every later commit and panel
  command window would stop at the pending-update wait. Both paths (`dpu_process()` and
  `dpu_cmd_window_begin()`, so the ALPM exit and the panel-recovery branch too) then unmask once
  and wait up to 100 ms for the latch, logging
  `<who>: update #n held by a SysMMU fault mask: unmask k/3, latched|still pending`. Each held
  update gets at most 3 such unmasks in total. After that DECON is **quiesced**: the trigger
  stays masked (logged once: `update #n still held after 3 unmasks: quiesced …`), command
  windows return -EBUSY, and a commit with the same buffers is skipped. A commit with a
  different buffer set and no bounce copy replaces the held update (the trigger is masked, so
  its shadow registers can be rewritten), and the quiesce ends with the next latch. Nothing is
  freed by the recovery: the held frame is `d->active`, the frame before it stays on
  `d->stale` until a latch is seen, so their IOVAs stay mapped throughout.
- Probe requires `iommus` to name exactly sysmmu_dpu0..2 (0x160a0000/0x160b0000/0x160c0000,
  each once) with a read-only view of each, and all three translating after the runtime resume:
  CTRL bit 0 (enable) set in each of them, i.e. the controller bitmask is 0x7 (an enabled
  SysMMU reads CTRL = 0x5). Anything else fails the probe: with `iommus` the device's DMA ops
  are iommu-dma, so even the bounce path would hand out IOVAs. The DTB without `iommus` is the
  fallback.
- A probe that fails after the fault handler was installed removes it, waits for running
  handlers (`synchronize_irq()` on the three SysMMU IRQs), drops DECON's runtime PM hold (the
  SysMMUs go to bypass once the driver core releases the probe's supplier references; until
  then they translate the boot fb 1:1), clears the fault inhibit and writes back the trigger
  saved before the handler was installed, so the splash is never left frozen by a probe-time
  fault.

## Test DTB: exynos9810-crownlte-romram-iommu.dtb vs exynos9810-crownlte-romram-v18.dtb

- `sysmmu@160a0000/160b0000/160c0000`: compatible `"samsung,exynos9810-sysmmu",
  "samsung,exynos-sysmmu"`, `samsung,qos = <15>`, `samsung,pmu-syscon` = PMU,
  `samsung,pd-status = <0x4024 0xf 0xf>`, `status = "okay"`, phandles 0x40..0x42.
- decon_f: `reg`/`reg-names` without the sysmmu0..2 windows (exynos-iommu requests the whole
  SFR windows; a nested decon_f resource blocks that request), `iommus = <0x40 0x41 0x42>`,
  `memory-region = <decon-pool framebuffer@cc000000>`, phandle 0x44.
- reserved-memory `framebuffer@cc000000`: `iommu-addresses = <0x44 0x0 0xcc000000 0x1400000>`,
  phandle 0x43.
- Nothing else. decon_f is the only device with `iommus`.

## Expected log (test DTB)

```
exynos-sysmmu 160a0000.sysmmu: bootloader state: CTRL=0x0 CFG=0x… … VERSION=0x7…
exynos-sysmmu 160a0000.sysmmu: Exynos9810 SysMMU v7.x registered: 4.9 CFG 0x…f80 (QoS 15), …
exynos-sysmmu 160a0000.sysmmu: enable #1 for 16030000.decon_f: … readback CTRL=0x5 …   (×3)
exynos9810-decon 16030000.decon_f: decon iommu: boot fb 0x00000000cc000000+0x1042800 identity map: 18/18 probes 1:1
exynos9810-decon 16030000.decon_f: decon iommu: DMA domain attached: dma-bufs map to cached IOVAs, no bounce copies
exynos9810-decon 16030000.decon_f: sysmmu_dpuN CTRL=0x5 VERSION=…
exynos9810-decon 16030000.decon_f: decon iommu: SysMMUs 0x7 already translating at probe (…)
exynos9810-decon 16030000.decon_f: decon iommu: sysmmu_dpuN after runtime resume: CTRL=0x5 …
exynos9810-decon 16030000.decon_f: decon iommu: SysMMUs translating (0x7 -> 0x7), FRAME_COUNT …
  (0x7 / "SysMMUs 0x7" are controller bitmasks of CTRL bit 0, not CTRL values; CTRL reads 0x5)
exynos9810-decon 16030000.decon_f: fb0 takeover: … : 0, latched …
… once HWC runs:
exynos9810-decon 16030000.decon_f: decon iommu: map #1: 17305600-byte dma-buf, N physical chunks -> IOVA 0x… (cached 1, hits 0, unmapped 0)
```

The bounce lines (`scattered buffer (… segments …) copied to …`, `bounce buffer N: … bytes`)
must not appear. Screen: identical to ROM-in-RAM v16 (SBoot splash, boot animation, setup
wizard), with no copy latency.

## Failure reading

| Log | Meaning |
|---|---|
| `power domain off (PMU 0x4024 …)` | DPU domain off at SysMMU probe; no SFR touched |
| `translation already enabled by the bootloader` | SBoot used a SysMMU: that SysMMU is refused, so DECON finds fewer than three translating and refuses to probe |
| `decon iommu: "iommus" lists N SysMMUs` / `iommus[i] at …: not a DPU SysMMU / listed twice / no read-only view` | DT does not name sysmmu_dpu0..2 exactly once each: DECON refuses to probe, SysMMUs return to bypass, splash stays |
| `decon iommu: boot fb IOVA … not identity` | `iommu-addresses` not applied: DECON refuses to probe, SysMMUs return to bypass, splash stays |
| `decon iommu: SysMMUs 0x… of 0x7 translating` | runtime PM/device link did not enable all three: DECON refuses |
| `decon iommu: SysMMU read fault #n at IOVA …` + `TE trigger masked` | a buffer was unmapped while scanned (lifetime bug); the panel freezes on its last frame until the next commit |
| `decon iommu: <who>: update #n held by a SysMMU fault mask: unmask k/3, …` | recovery of an update the fault mask had blocked; `still pending` means that commit or command window was skipped and the next one tries again |
| `decon iommu: <who>: update #n still held after 3 unmasks: quiesced …` | the held update never latched: trigger masked until a commit with other buffers (`quiesced: a commit with other buffers replaces held update #n`) |
| `decon iommu: probe failed: fault handler removed, TE trigger back to …` | DECON did not probe; the trigger as SBoot left it (read before the handler) is restored |
| `decon iommu: no IOVA for a …-byte dma-buf (…)` | mapping failed; that frame bounces as before |

## Not ported from crownlte-dpu-iommu

- The `samsung,dpu-iommu-selftest` scattered-page/system-heap/CMA flip test (~250 lines, built
  on the old stage 2 self-test helpers). The HWC path of the ROM-in-RAM boot is the test now; the
  old self-test can be ported if a kernel-only check is wanted.
- `DMA_ATTR_FORCE_CONTIGUOUS` for in-kernel buffers: not needed, they work at any IOVA.
- The display-s2-iommu diagnostic DTS (the old stage 2 DT had the dpp/dpp-dma names swapped).

## Limits / follow-ups

- TLB way/port tuning (`sysmmu,tlb_property`) not ported: risk of underruns under heavy load.
- The secure fault IRQs (SPI 210/212/214) are not requested (they belong to the secure world).
- Orphaned cache entries (the app freed the buffer) are dropped at the next import or commit
  release; while the display is idle they keep their pages for that long.
