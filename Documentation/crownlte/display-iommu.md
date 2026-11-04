# crownlte display IOMMU (DPU SysMMU) — plan and skeleton

Status: **DT skeleton only, disabled, no driver change.** Nothing here runs.

## Why the display needs it

- The vendor gralloc allocates HWC layers from `/dev/dma_heap/system` (VENDOR-ABI.md),
  which hands out scattered pages. A DPP channel takes one base address per plane, so
  such a buffer can only be scanned through the DPU SysMMU.
- The adopt-mode DECON driver (`drivers/video/fbdev/exynos/dpu_9810/`) therefore accepts
  only buffers that map to one contiguous run below 4 GiB (CMA heap, the `decon-pool`
  reserved pool, the SBoot framebuffer). Scattered buffers are rejected with -EINVAL and
  the log line `buffer not contiguous (segment N); needs the DPU SysMMU`.
- SBoot scans 0xcc000000 with the SysMMUs in bypass. The stage 1 and stage 2 drivers
  read `CTRL` and refuse to adopt if bit 0 is set.

## Hardware (4.9 crownlte_eur_open_26 DT + 4.9 drivers/iommu/exynos-iommu.[ch])

| Instance | Non-secure SFR | Secure SFR | IRQ (NS, secure) | Ports | 4.9 clock |
|---|---|---|---|---|---|
| sysmmu_dpu0 | 0x160a0000 (0x9000) | 0x160d0000 | SPI 211, 210 | VGRF, VGF | vclk 854 (GATE_SYSMMU_DPUD0) |
| sysmmu_dpu1 | 0x160b0000 (0x9000) | 0x160e0000 | SPI 213, 212 | G0, VG0 | vclk 855 (GATE_SYSMMU_DPUD1) |
| sysmmu_dpu2 | 0x160c0000 (0x9000) | 0x160f0000 | SPI 215, 214 | G1, VG1, WB | vclk 856 (GATE_SYSMMU_DPUD2) |

In 4.9 all three are listed in `iommus` of the DSIM device, which calls `iovmm_activate()`.
DECON maps buffers through that one shared IOVA space. In 4.9 the DPU clocks are HWACG
Q-channel gates whose enable/disable are no-ops, and the same holds in the 6.12 legacy
clock driver.

## Does mainline exynos-iommu fit? (6.12 tree, `drivers/iommu/exynos-iommu.c`)

| Item | 4.9 (9810) | mainline | Verdict |
|---|---|---|---|
| VERSION decode | `(reg >> 17) & 0x7fff`, major = `>> 11`, minor = `(>> 4) & 0x7f`, i.e. major bits 31:28, minor 27:21 | `reg >> 21`, major = `>> 7`, minor = `& 0x7f`: same bits | same |
| Variant selection | — | major < 5: v1; < 7: v5; ≥ 7: v7, VM layout only if CAPA0 bit 11 and CAPA1 bit 14 (VCR) | 9810 expected to read major 7 (4.9 has the CAPA0/1_V7 registers at 0x870/0x874) |
| PT base / flush / INT / fault regs | PT_BASE_PPN 0x0c, FLUSH 0x10, FLUSH_ENTRY 0x14, FLUSH_RANGE 0x18, RANGE_START/END 0x20/0x24, INT_STATUS/CLEAR 0x60/0x64, FAULT_AR 0x70/0x78 | `sysmmu_v7_variant`: identical offsets | match (non-VM) |
| Page table | 4096 × 1 MiB lv1, 256 × 4 KiB lv2, `PG_ENT_SHIFT 4`, 36-bit PA | same format for v5+ (`PG_ENT_SHIFT` 4) | match. Prot bits: mainline uses `SYSMMU_V5_LV1/LV2_PROT`; 4.9 has no access-protection bits, so verify on hardware with a read fault test |
| CTRL enable | 0x5 enable, 0x7 block, 0x0 disable | v5+: `CTRL_ENABLE` 0x5 / `CTRL_BLOCK` 0x7 | match |
| CFG | `CFG_ACGEN`, FLPD cache, QoS 15 (`qos = <15>`), no access-protection check | `CFG_QOS(15) \| CFG_FLPDCACHE \| CFG_ACGEN \| CFG_EAP` | QoS matches. `CFG_EAP` enables PTE access-protection checks that 4.9 never used: if the 9810 interprets the v5 prot bits differently, reads fault. Test first; if needed, skip EAP for this SoC |
| TLB way config | `sysmmu,tlb_property` → public/private way registers 0x120/0x200+ | not configured | performance only. Defaults work but may underrun at 1 GB/s. Port as an optional DT property |
| Fault IRQ | index 0 NS; index 1 secure (`sysmmu,secure-irq`, via SMC) | index 0 only | fine. Never map the secure SFR block |
| Clocks | "aclk" (no-op HWACG gate) | `devm_clk_get_optional("sysmmu"/"aclk"/"pclk"/"master")` | omit clocks or pass the GATE_SYSMMU_DPUDn vclks (no-ops) |
| Power | DPU PD + runtime PM | runtime PM with device links to the master | DECON must use runtime PM (`pm_runtime_enable` + `get_sync` at probe) so that the link resumes the SysMMUs |
| Reserved direct maps | n/a (4.9 re-allocates fb0) | **no `.get_resv_regions`**: DT `iommu-addresses` in reserved-memory is ignored | **patch needed** (see step 2) |

Conclusion: mainline `exynos-iommu` with `sysmmu_v7_variant` should drive the 9810 DPU
SysMMUs with no new register layout. It still needs (a) a reserved-region hook for the
handover, (b) a check of `CFG_EAP` against the 9810's PTE protection bits, (c) optional
TLB-way tuning, and (d) the DECON driver switching to runtime PM. The VERSION value is logged by both the stage 1 handover helper
(`display handover: sysmmu_dpuN … VERSION=`) and the stage 2 driver
(`decon: sysmmu_dpuN CTRL= VERSION=`). Confirm major = 7 from trial q25 before
enabling anything.

## The handover problem

When a DPU SysMMU is enabled, every DPP fetch is translated. DECON is still scanning
SBoot's buffer at 0xcc000000 (and, after takeover, fb0 at the same address). An empty
page table would make the very next frame fault (garbage on screen plus a fault-IRQ
storm). The IOVA 0xcc000000 must be identity-mapped **before** the first enable:

1. Add `iommu-addresses = <&decon_f 0x0 0xcc000000 0x1400000>;` to the
   `framebuffer@cc000000` reserved-memory node (DT schema `reserved-memory.yaml`).
2. Give exynos-iommu `.get_resv_regions = iommu_dma_get_resv_regions` (a few lines). The
   IOMMU core then creates an `IOMMU_RESV_DIRECT` mapping in the default DMA domain
   when the device attaches, before the driver probes.
3. Only then add `iommus = <&sysmmu_dpu0>, <&sysmmu_dpu1>, <&sysmmu_dpu2>;` to `decon_f`,
   drop its read-only `sysmmu0..2` reg entries (they would collide with the SysMMU
   driver's `devm_ioremap_resource`), and enable the three nodes.
4. In the DECON driver, replace the "contiguous below 4 GiB" check with "one contiguous
   IOVA run" (`dma_set_max_seg_size(dev, UINT_MAX)` so iommu-dma merges the list). The
   check in `dpu_buf_import()` already tests DMA-address contiguity, which holds for IOVAs
   as well.

## Steps and tests

| Step | Change | Test (diag DT gated) | Risk |
|---|---|---|---|
| I-1 | this skeleton (disabled nodes) | none | none |
| I-2 | trial q25 logs: CTRL/VERSION of all three | read-only | none |
| I-3 | exynos-iommu `.get_resv_regions`, DT `iommu-addresses`, enable **sysmmu_dpu1 only** (G0 port, the window SBoot scans if the survey shows dpp0) on decon_f | DECON keeps showing fb0; a fault IRQ would be logged by exynos-iommu | display garbage on a wrong map, no damage. A fault IRQ storm is possible: bounded by the 120 s diag reboot |
| I-4 | all three, self-test flips scattered system-heap buffers | `decon selftest:` with `/dev/dma_heap/system` buffers | as above |
| I-5 | TLB way config from `sysmmu,tlb_property` | underrun counter (`EXTRA_INTERRUPT` resource conflict) under load | none |

Panel/power safety: none of this touches DSIM, the panel, regulators or clocks rates.
