# crownlte integration 6: perf + DPU SysMMU + Wi-Fi + TEE T3 (TEE DT off)

Branch `crownlte-integration-6`. Build only; this exact combination has not run on the
phone yet. Every part has passed on hardware on its own branch.

| Branch | Tip merged | Hardware result of that tip |
|---|---|---|
| `crownlte-perf` | 48b64c36d (base of this branch) | EAS, cpufreq (LITTLE 1950), thermal parity, input booster, UFS clock gating PASS (ROM-RAM done-check, Image 90444652, DTB v23a-perf 9896c18e) |
| `crownlte-dpu-scale-iommu` | 457abc59a (Image from 7d0935d7a; 457abc59a only adds the doc) | home screen on the panel, scan proof #1-#4 PASS through the DPU SysMMU, 0 bounce copies, no faults (pack dpu-iommu 76d3016b, Image 1a32a91d, DTB v23a-dpu-iommu a9747cb9) |
| `crownlte-wifi` | 15cebae03 (includes the FORTIFY fixes) | self-test `RESULT PASS-SHARED chip=0x4361 rev=5`, Android HAL scan lists 2.4 and 5 GHz BSSes (pack wifi-adb2 80cbe664, Image 0283f904, DTB romram-wifi-s1 dfd395d8) |
| `crownlte-tee-3` | 22a6b88cc (registry guard, T3 S1) | built and reviewed; in this integration the driver is built in, but its DT node stays **disabled**, so it does not probe |

## 1. Merges

```
48b64c36d crownlte-perf (start of the branch)
5b84e9146 Merge branch crownlte-dpu-scale-iommu      no conflicts
d50f84639 Merge branch crownlte-wifi                 no conflicts
d6da5e74c Merge branch crownlte-tee-3                no conflicts
6aa61465f arm64: dts: crownlte ROM-in-RAM v23a-int6 (DPU SysMMU + perf + Wi-Fi)
```

All three merges were clean text merges. The branches change different files. The
files they share (`exynos-usi.c`, `i2c-exynos5.c`, the s2dos05 diag,
`crownlte_android_defconfig`, `exynos9810-crownlte-clocks.dtsi`, `trustonic-tee.md`, the dts
`Makefile`) hold the same commits on both sides (integration-3/4 and s2dos05-diag), or the
changes sit in separate hunks. Semantic checks:

- The DPU, Wi-Fi and TEE code paths are separate (`dpu_9810/`, `pcie-exynos9810.c` +
  `bcmdhd_101_16/`, `gud-exynos9810/`). The merges do not change any perf code (cpufreq, TMU,
  booster, UFS).
- The earlier unreviewed WIP `c9a54af45` (integration-5) merged only `crownlte-dpu-scale`
  4ac2e1d38. Its `drivers/` tree is identical to 5b84e9146 minus the SysMMU commits
  9d4fdf396..457abc59a. It was compared but not used.

## 2. Config

ROM-RAM config = the config embedded in the hardware-tested perf Image
(`scripts/extract-ikconfig panel-bright-work/images/Image-romram-perf-48b64c36d`, sha 90444652)
plus `arch/arm64/configs/crownlte_wifi.fragment`, then `olddefconfig`.
Result: `panel-bright-work/ko-int6-6aa6/config` (sha256 c7b917d1).

Checked against the configs embedded in the four hardware Images (perf 90444652, dpu-iommu
1a32a91d, wifi 0283f904, tee e4a5a04e). Every `CONFIG_*=` value set in any of the four is
in the result, except `NO_IOPORT_MAP`. Every set value in the result is also set in perf or
in wifi. Differences, with the reason for each:

| Option(s) | perf | dpu | wifi | tee | int6 | Why |
|---|---|---|---|---|---|---|
| `NO_IOPORT_MAP` / `HAS_IOPORT_MAP` | NO | NO | HAS | NO | HAS | derived: arm64 selects `NO_IOPORT_MAP` only without PCI; same as the Wi-Fi hardware Image |
| `PCI`, `PCI_MSI`, `PCIE_EXYNOS9810`, `PCIE_DW(_HOST)`, `ARM_GIC_V2M`, `BLK_MQ_PCI`, `PCI_QUIRKS`, ... | - | - | y | - | y | Wi-Fi RC; `PCIEPORTBUS`, `PCIEASPM`, `PCI_EXYNOS`, `USB_PCI`, `VGA_ARB` forced off by the fragment |
| `CFG80211`, `NL80211_TESTMODE`, `BROADCOM_WIFI`, `BCM4361`, `BCMDHD_*`, `WLAN_REGION_CODE=100`, `WLAIBSS`, `WL_RELMCAST` | - | - | y | - | y | Wi-Fi (4.9 symbols and values) |
| `FW_LOADER_USER_HELPER(_FALLBACK)` | - | - | y | - | y | Wi-Fi firmware from /vendor through ueventd. Global: see risk R3 |
| `ASN1`, `CRYPTO_RSA`, `X509_CERTIFICATE_PARSER`, `PKCS7_MESSAGE_PARSER`, `SYSTEM_TRUSTED_KEYRING`, ... | - | - | y | - | y | pulled in by `CFG80211_REQUIRE_SIGNED_REGDB` (not user-selectable). Harmless |
| `NET_VENDOR_*=y` (menus only, all drivers off) | - | - | y | - | y | PCI-dependent menu defaults |
| `INPUT_CROWNLTE_BOOSTER`, `MQ_IOSCHED_DEADLINE`, `THERMAL_GOV_POWER_ALLOCATOR` | y | - | - | - | y | perf (the other branches fork before b02d3f769) |
| `CROWNLTE_S2DOS05_DIAG` | y | y | - | - | y | read-only PMIC diag driver; its node is in v23a (Wi-Fi forks before s2dos05-diag) |
| `EXYNOS_IOMMU`, `IOMMU_DMA`, `IOMMU_SUPPORT`, `OF_IOMMU`, `IOMMU_DEFAULT_DMA_STRICT`, ... | y | y | y | - | y | DPU SysMMU. The tee-3 Image used the older flipfix config without SysMMU |
| `TRUSTONIC_TEE` (+ `CRYPTO_LIB_SHA256`, selected by tee-3) | y | y | y | y | y | TEE driver built in; DT node off |

No conflicting values: no option is `y` in one Image and set to a different value in
another. Only `NO_IOPORT_MAP` changes, and it is derived.

## 3. DT: `exynos9810-crownlte-romram-v23a-int6.dts`

```
#include "exynos9810-crownlte-romram-v23a-dpu-iommu.dts"   (v23a + DPU HDR caps + DPU SysMMU)
#include "exynos9810-crownlte-perf.dtsi"                   (as v23a-perf)
#include "exynos9810-crownlte-romram-wifi.dtsi"            (new: romram-wifi-s1 nodes as path overlays)
model = "... integration 6 ..."; bootargs = "panic=30 ro sched_verbose" (as v23a-perf)
&{/tee} status = "disabled" (unchanged from v23a, restated)
```

Build: `KSRC=<tree> KO=<build dir> rom-ram/src/build-dtb.sh <dts> <dtb>` (kbuild does not build
the flattened ROM-RAM DTBs). DTB sha256 297eb4f4.

`exynos9810-crownlte-romram-wifi.dtsi` holds the nodes and values of the generated
`panel-bright-work/dtb/exynos9810-crownlte-romram-wifi-s1.dts` (generator `romram-wifi-s1-gen.py`):

- `pcie@116a0000` (RC ch0, SPI 253, PERST# gpf2-2);
- `wlan` (`samsung,brcm-wlan`, REG_ON gpg0-5, host-wake gpa0-7);
- the five pin groups `cfg-wlanen`, `pcie0-clkreq`, `pcie0-perst`, `pcie-wake`, `wlan-host-wake`.

It refers to nodes by path, because the flat v23a DTS has no labels. dtc assigns the
phandles. The only structural change: `pcie@116a0000` is now the last child of `soc@0`, and
`wlan` the last child of `/`. The generator put them before `system-controller@14060000` and
before `soc@0`. The glue driver returns `-EPROBE_DEFER` until the RC exists, so the order does
not matter.

Checks on the build host:

- The int6 tree rebuilds `v23a-dpu-iommu` = a9747cb9 and `v23a-perf` = 9896c18e byte for byte.
- A semantic compare decompiles each DTB and resolves every phandle cell to its node path.
  The cell positions come from the int6 source compiled dts->dts. It shows that int6 is
  exactly `v23a + (dpu-iommu - v23a) + (perf - v23a) + (wifi-s1 - v23a)`. The deltas are
  29 dpu, 100 perf and 67 Wi-Fi properties/nodes. The deltas do not overlap, except `model`,
  which int6 overrides. With the perf DTB left out, the same compare reports 101 mismatches.
- dtc with the kbuild W=1 checks (`-Wnode_name_chars_strict -Wproperty_name_chars_strict
  -Wunique_unit_address_if_enabled`) reports no warning that v23a-dpu-iommu or v23a-perf do
  not already report.

### Self-tests armed by this DTB

All of these were armed in the hardware DTBs above. None writes anything except as listed.
The DT arms no write-capable test, and adds no new test.

| Time | DT property | What it does | Writes |
|---|---|---|---|
| 40 s after cpufreq | `samsung,boot-qos-ms` (perf) | boot QoS release | none (freq QoS only) |
| 60 s | `samsung,perf-report-ms` (perf) | capacities/EM/EAS report | none |
| 95 s | `samsung,booster-selftest-ms` (perf) | synthetic touch through the booster state machine | none (freq QoS minimums only) |
| 104 s | `adc samsung,adc-selftest-ms` | 5 conversions per channel | ADC conversion registers only (normal read path, 9 per conversion) |
| 104.5 / 107 s | `battery-monitor@36 samsung,adc-selftest-ms` / `samsung,selftest-ms` | thermistors, FG/charger register snapshot | none except the existing `samsung,charger-wdt-kick` (CNFG_06 WDTCLR, power-gates) |
| 105 s | `acpm-tmu samsung,tmu-selftest` | TMU readback over ACPM | none |
| 106 s | `s2mps18-pmic samsung,pmic-selftest` | PMIC id/RTC reads (RTC latch read as in v23a) | none (no RTC write property) |
| 109 s | `touchscreen@48 samsung,touch-selftest` | chip id/fw version read | none |
| boot | `pmic-diagnostic@60` (s2dos05, `crownlte,read-only-diagnostic`) | S2DOS05 snapshots | none (IRQ register 0x11 is clear-on-read, read once) |
| 150 s | `wlan samsung,wifi-selftest` (`-ms` 150000) | power, link, chip id, firmware, wlan0 up/down; no kernel scan | Wi-Fi chip power and firmware only (as Android); skipped (`PASS-SHARED`) if userspace already owns wlan0 |

Off: `display-handover samsung,display-selftest` (the node is disabled, as in v23a), every
v23a-dpu-selftest compose step (not included), and every TEE stage (node disabled, no guard
or pin properties).

## 4. Artifacts (gekko, /mnt/ssd/host-builds/kernel-upgrade)

| File | sha256 |
|---|---|
| `panel-bright-work/ko-int6-6aa6/arch/arm64/boot/Image` | b7298541deda7877135e4c0094cf341fa175dc75f8e935e80c454fcff6106506 |
| `panel-bright-work/ko-int6-6aa6/arch/arm64/boot/dts/exynos/exynos9810-crownlte-romram-v23a-int6.dtb` | 297eb4f48eafd77f6e832c3eec8c2ebd00385602c0458e452851f86796ab4b81 |
| `panel-bright-work/ko-int6-6aa6/config` | c7b917d1f7e3e7869bf943a208e25cb180724ce5745657215ff52a132a359f39 |
| `rom-ram/pack-romram-int6/recovery.img` (ROM-RAM trial) | 8e4c5c462a4889618dd4b7f03bd613bfa40d3820942df7968374dbcf81ccc4f3 |

Pack: `rom-ram/pack-rom-ram.py rom-ram/pack-romram-int6 <dtb sha>` with
`KO=panel-bright-work/ko-int6-6aa6`, `DTB_NAME=exynos9810-crownlte-romram-v23a-int6.dtb`,
`SHIM=bbea9590b` (as dpu-iommu), `RAMDISK=rom-ram/build-30/diagnostic-initramfs.cpio.gz`
(e615cc8d: ADB, DONE frame), loader f8131c92, parent a8198be0, restore RECOVERY 8763860c.
The Image banner is `Linux version 6.12.93-crownlte-diag+`. The Image is built from
d6da5e74c. 6aa61465f adds only the DTS and does not change the Image.

## 5. Trial checklist (ROM-RAM, standard trial)

The standard trial: `run-trial.sh <dir> 630` with romram-capture.ps1 in the background, then
romram-adb.ps1. Boot to Lineage. Then run the checks, then touch `<dir>\done.flag`. Read-only
throughout. Grep the serial log and `adb shell dmesg`.

### A. Display (DPU composition + SysMMU)

PASS:

```
exynos-sysmmu 160a0000.sysmmu: Exynos9810 SysMMU v7.x registered: 4.9 CFG ... (QoS 15)   (x3: 160a/160b/160c)
decon iommu: boot fb 0x00000000cc000000+0x1042800 identity map: 18/18 probes 1:1
decon iommu: SysMMUs translating (0x7 -> 0x7)
decon iommu: scan proof #1 ... PASS   (#2, #3 likewise; #4 after Home: N/N windows via IOVA)
... 0 bounce-copied; SysMMUs 0x7 of 0x7 translating; ... SysMMU faults 0: PASS
```

Camera: the panel shows the home screen and follows `adb exec-out screencap`.

FAIL signatures: `bounce buffer`, `scattered buffer (... segments ...) copied`,
`cma: decon-pool: alloc failed`, `decon iommu: no IOVA`, `SysMMU ... fault`,
`scan proof ... FAIL`, `active IDMA base ... programmed`, `boot fb IOVA ... not identity`
(DECON refused, splash stays), `SysMMUs 0x3 of 0x7` (a SysMMU stopped translating). See
display-composition.md 10.5.

### B. Wi-Fi (PCIe RC + bcmdhd)

PASS, boot:

```
exynos9810-pcie 116a0000.pcie: RC0 ready (link down), I/A on, max GEN2 x1
... link up: ... GEN1 x1 (max GEN2)          (GEN1 as on the wifi-adb2 run; GEN2 also acceptable)
0000:00:00.0 [144d:ecec], 0000:01:00.0 [14e4:441f/4420/4421]; exynos9810: async suspend off, DMA limit 32 bit
crownlte-brcm-wlan wlan: BCM4361 powered, PCIe ch0 link up, host-wake irq N
wifi selftest: armed for 150000 ms after boot
... poweroff done, state 0                  (4.9 post-attach power-off)
```

PASS, at 150 s: `wifi selftest: RESULT PASS chip=0x4361 rev=5 ...` or, if Lineage already
turned Wi-Fi on, `wifi selftest: RESULT PASS-SHARED chip=0x4361 rev=5 (wlan0 owned by
userspace, scan skipped)` (the wifi-adb2 result). Expected values: fw 13.50.37,
FWID 01-c47464bc, clm 201112, nvram `nvram.txt_r02b_b2`.

PASS, HAL over adb: `cmd wifi set-wifi-enabled enabled`; `cmd wifi status` shows
"Wifi is enabled"; `cmd wifi start-scan`; then `cmd wifi list-scan-results` lists 2.4 and
5 GHz BSSes.

FAIL signatures: `link is not up, try count: 10` + LTSSM history (PHY/PERST/REG_ON),
`iATU viewport reads all ones`, no `0000:01:00.0`, `RESULT FAIL stage=...` (`chip-id`;
`firmware/bring-up` at about 400 s means ueventd did not serve /vendor/firmware),
ioctl/`dhd_bus_init` timeouts after firmware download (DMA), a FORTIFY
`detected buffer overflow` panic (sec_save_wlinfo, fixed in 2e9ebfc5d/15cebae03), any
`swiotlb buffer is full`.

### C. Performance (perf-tuning.md "Expected log lines")

PASS:

```
perf: domain0: max 1950000 kHz above CAL max 1794000 kHz (DT samsung,dt-max-freq-above-cal, 4.9 acme), FVMap 1081250 uV
registered: domain0 11 levels 455000-1950000 kHz, domain1 11 levels 650000-1794000 kHz
perf: domain0 (cpus 0-3) boot QoS min=max=1950000 kHz for 40000 ms ... ; domain1 1794000
sched_energy_set: starting EAS
BIG: hotplug-equivalent cap armed on policy4: cap at 650000 kHz from 96000 mC, release below 91000 mC (ret=0)
LITTLE: trip 76000 mC ... = ECT ; 81000 ; 86000 ; 91000 ; 96000 state 10..10 = 455000
BIG: governor power_allocator, ECT PID constants applied
clock gating delay 20 ms (4.9 LINK_H8_DELAY); hibern8 enter #1 (exit 0, CAL errors 0)
touch boost: head 130 ms, tail 500 ms after release, 2 cluster(s) ...; boosting touches of sec_touchscreen
~40 s: perf: domain0 boot QoS 1950000 kHz released after 40000 ms (domain1 likewise)
60 s:  perf report: EAS preconditions: asymmetric capacity yes (385..1024), EM 2/2, schedutil 2/2, freq-invariant yes
95 s:  selftest: PASS (3/3)
first touch: first BTN_TOUCH 1 from sec_touchscreen
```

adb (read-only): `cpu_capacity` 385 x4 / 1024 x4; policy0 `scaling_available_frequencies`
starts 1950000; `rate_limit_us` 5000; `sched_energy_aware` 1; BIG zone `power_allocator`,
`sustainable_power` 2000; `/sys/block/sda/queue/scheduler` `[mq-deadline]`; UFS
`clkgate_enable` 1, `clkgate_delay_ms` 20.

FAIL signatures: no `starting EAS` (check `perf report: EAS preconditions ... no`), domain0
registered with max 1794000 (1950 refused: FVMap), `ret=-...` on boot QoS / booster requests,
`selftest: FAIL`, BIG zone on step_wise, `CAL errors` > 0 in the hibern8 lines.

### D. TEE (must stay inert)

PASS: the `MobiCore mcDrvModuleApi version is ...` banner from module init, and no further
`mcDrvModule`/`TEE` probe, SMC, `session-guard` or `registry` lines. `/dev/mobicore*`
is absent.

FAIL: any TEE probe or SMC line. That would mean the node is enabled; the DTB is wrong.

### E. Unchanged base lines (v23a)

`adc selftest: ... -> PASS` (both the ADC and the battery thermistor lines),
`battery selftest: ... -> PASS`, `pmic selftest: rtc advance 1 s over 1 s: PASS` and
`rtc plausibility ... PASS`, `touch selftest: PASS`, `acpm-tmu selftest: done sensors=N`,
s2dos05 `complete; no further reads`, PID1 ROM-RAM START/DONE as with build-30.

## 6. Risks

- R1: never run in this combination. New at run time: the DPU SysMMUs translating while the
  PCIe link and DHD swiotlb DMA are active. The two DMA paths are independent: PCIe has no
  `iommus` and stays on direct DMA with a 32-bit limit, as on the Wi-Fi run, where the DPU
  SysMMUs were not enabled.
- R2: boot QoS (LITTLE 1950 / big 1794 for 40 s) overlaps the PCIe link training and the DHD
  attach. Each part ran under boot QoS separately (the perf run had no Wi-Fi). Thermal
  headroom at boot is large.
- R3: `FW_LOADER_USER_HELPER_FALLBACK` is global (wifi-port.md R3): a missing firmware file
  now waits up to 60 s for userspace. Callers in this config: DHD, cfg80211 regulatory.db
  (async). The perf and DPU code request no firmware.
- R4: node order of `pcie@116a0000` / `wlan` differs from the tested Wi-Fi DTB (section 3).
  Probe ordering depends on deferral, not on node order.
- R5: Wi-Fi association, L1SS, suspend and host-wake are untested (Wi-Fi stage 1 limits).
  The TEE guard is untested on hardware.
- R6: the combined config was derived from Image-embedded configs, not from an in-tree
  defconfig. To reproduce it, follow section 2.
