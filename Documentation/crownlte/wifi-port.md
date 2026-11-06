# crownlte Wi-Fi on 6.12: Exynos 9810 PCIe RC + Samsung bcmdhd (BCM4361), stage 1

Status: build-only. Branch `crownlte-wifi` on crownlte-integration-4 (b4f756bed). Not run on
the phone. Stage 1 = power, PCIe link, enumeration, chip id, firmware download, wlan0 up, one
passive scan, all reported by `wifi selftest:`. No association, no file writes.

Paths: K49 = /root/crownlte-hid-kernel (the running 4.9 kernel), V = VENDOR.img
(backups-20261103/partitions, mounted ro,noload), T = this tree.

## 1. Findings (4.9 + vendor, with evidence)

| Item | Value | Evidence |
|---|---|---|
| Chip | Broadcom BCM4361, revision B2 | K49 out/.config `CONFIG_BCM4361=y`; 4.9 build defines `-DBCM4361_CHIP` (K49 out/drivers/net/wireless/bcmdhd_101_16/.dhd_linux.o.cmd); V:/firmware `bcmdhd_sta.bin_b2`, `nvram.txt_*_b2`, BT `bcm4361B2_{murata,semco}.hcd`. chipcommon id 0x4361 (T bcmdevs_legacy.h:126), PCI ids 14e4:441f/4420/4421 (:74-76). Exact chiprev is printed by the self-test |
| Driver | Samsung bcmdhd 101.16 (Broadcom DHD), PCIe full dongle, built in, module name `dhd` | K49 drivers/net/wireless/Makefile:22, bcmdhd_101_16/Makefile:2069-2070 |
| Bus | Exynos 9810 PCIe **channel 0**, `pcie0@116A0000`, DesignWare core, ip-ver 0x981000, Gen2 x1, INTx (use-msi "false"), SPI 253 for RC and INTx | K49 arch/arm64/boot/dts/exynos/exynos9810-crownlte_eur_open_26.dts:11682-11715; dhd_pcie.h:758-764 (`EXYNOS_PCIE_CH_NUM 0`, RC id 144d:ecec) |
| RC register windows | elbi 0x116A0000, PHY/PMA 0x116D0000, sysreg 0x11411044 (16 B), DBI 0x11700000, PCS 0x116C0000 (0x1fc), config 0x127FE000 (8 KiB), I/A 0x11680000; MEM window 0x11800000 + 0xFF0000 | same DTS node |
| PERST# | gpf2-2 (pinctrl@11430000), output, high = released | DTS:11684 `gpios = <gpf2 2 1>`, pcie0_perst:1558; pci-exynos.c:1174 |
| CLKREQ# | gpf2-0 function 2 | DTS:1535 |
| PCIE_WAKE | gpa3-2 input | DTS:241 |
| WLAN_REG_ON | gpg0-5, output high = on | DTS:13431-13437 (`gpios = <gpg0 5 1 gpa0 7 1>`), cfg_wlanen:2634; K49 dhd_custom_exynos.c:202,213 |
| WLAN_HOST_WAKE (OOB) | gpa0-7 (alive, EINT function 0xf), rising edge, threaded | DTS:249 wlan_host_wake; dhd_custom_exynos.c:231,246,275; define `DHD_USE_PCIE_OOB_THREADED_IRQ` |
| Power sequence | REG_ON high, 200 ms, `exynos_pcie_pm_resume(0)` at late_initcall; DHD module init at late_initcall_sync | K49 dhd_custom_exynos.c:213-223,343; define `USE_LATE_INITCALL_SYNC` |
| After attach | DHD powers the chip off again (`wl_android_wifi_off`), firmware loads only at wlan0 up | K49 dhd_linux.c:12871 (`WLAN_ACCEL_BOOT`), wl_android.c:12863 (`ENABLE_INSMOD_NO_FW_LOAD`) |
| Regulators / clocks | none: WLAN power is the REG_ON GPIO only; pcie0 has `pcie-clk-num = <0>`, `phy-clk-num = <0>` (4.9 enables no clock) | DTS:11703-11704 |
| PMU | PHY isolation PMU+0x71C bit0, wakeup mask PMU+0x610 bit 5 | K49 pci-exynos.h (PCIE_PHY_CONTROL, WAKEUP_MASK) |
| SysMMU / coherency | 4.9 enabled the PCIe SysMMU (sysmmu@11420000, 32-bit IOVAs) and sysreg sharability (use-cache-coherency "true") | DTS:11669-11680, 11706, 11709; pci-exynos9810_cal.c:409 (exynos_pcie_dma_ops) |
| NCLK_OFF | ELBI+0x2F0 bit2 set whenever the link is down (speculative access hang fix, 9810 only) | pci-exynos.c:48-53, 1550-1572, 2588, 2910-2966 |
| CP/WLAN shared memory | none for Wi-Fi. Only Samsung DHD preallocated buffers (`CONFIG_BROADCOM_WIFI_RESERVED_MEM`, dhd_custom_memprealloc.c) | K49 out/.config |
| Firmware | `bcmdhd_sta.bin_b2` (sta), `bcmdhd_mfg.bin_b2`, `bcmdhd_mon.bin_b2`, NVRAM `nvram.txt_{murata_r041,r02a,r02b}_b2` chosen by the module CID read from OTP, `bcmdhd_clm.blob`; all in /vendor/firmware | V:/firmware listing; 4.9 defines `DHD_LINUX_STD_FW_API`, `DHD_FW_NAME "bcmdhd_sta.bin"`, `DHD_NVRAM_NAME "nvram.txt"`, `DHD_CLM_NAME`, `SUPPORT_MULTIPLE_REVISION`, `USE_CID_CHECK` |
| Firmware loader | request_firmware(name); 4.9 reached /vendor/firmware via `FW_LOADER_USER_HELPER(_FALLBACK)` + ueventd `firmware_directories` | VENDOR-ABI.md item 7; K49 out/.config |
| sha256 (reference only, never in the tree) | `bcmdhd_sta.bin_b2` e3c66440a4a6c3d30918f922c20ccd0fbecaa1aaa0460b4458b48e202473f452, `bcmdhd_clm.blob` 1c1477864b0858bd0ad8700a766318c2f7220e4766dba0e8ee15d8bf13435356, `nvram.txt_r02a_b2` 5bb574b0f3debe080855f00cde0381416aeca7fafc4a3f4a488bbe2e6b5d8be9, `nvram.txt_r02b_b2` 68cb903937a5c2af055bee36141047df9cf5e7d79a23a24a0677e2faf85ccba2, `nvram.txt_murata_r041_b2` 6bf82955037750e1cd9534d524a62ea072b07058c5a5797030030ddde59c0173 | V:/firmware |
| MAC | 4.9 region 100 builds `GET_MAC_FROM_OTP`: MAC read from the module OTP CIS, else `cur_etheraddr` from firmware; written only into dongle RAM. The code then calls `dhd_write_file("/efs/wifi/.mac.info")`, which did nothing because 4.9 is built **without** `DHD_SUPPORT_VFS_CALL` (dhd_filp_open() is a NULL stub) | K49 dhd_sec_feature.h:218-221 (forced `-include`), dhd_custom_cis.c:1197-1350, :503, :1329; dhd.h:3818-3886. NVRAM `macaddr=00:90:4c:12:d0:01` is a placeholder. No macloader binary on V (wifi.rc:51 points to a missing file) |
| HAL | AIDL `android.hardware.wifi-service` + Broadcom legacy HAL `libwifi-hal.so` ("BCMDHD vendor HAL"): nl80211 + Broadcom vendor commands; changes firmware via `/sys/module/dhd/parameters/firmware_path`; iface wlan0, p2p0/`p2p-dev-`, static `swlan` | V:/lib64/libwifi-hal.so strings, V:/bin/hw/android.hardware.wifi-service strings, V:/etc/init/android.hardware.wifi-service.rc |
| Supplicant / hostapd | `wpa_supplicant -g@android:wpa_wlan0`, `driver_param=use_p2p_group_interface=1`; Android `DRIVER` commands as SIOCDEVPRIVATE+1 | V:/etc/init/android.hardware.wifi.supplicant-service.rc, V:/etc/wifi/*.conf, VENDOR-ABI.md item 12 |
| rc / sysfs | `modprobe dhd.ko` at early-init (no /vendor/lib/modules: harmless with built-in), chown `/sys/wifi/{mac_addr,cid,wifiver,otp,pm,ant,softap,memdump,frameburst,bad_ap_list,control_he_enab}`, `/sys/module/dhd/parameters/{firmware,nvram}_path`, `/data/vendor/conn/.*.info`, `wifi.interface=wlan0` | V:/etc/init/wifi_brcm.rc:2,16-40; V:/etc/init/wifi.rc:12-40 |

## 2. Strategy

**DHD, not brcmfmac.** brcmfmac has no BCM4361 id (T brcm_hw_ids.h) and implements none of the
Broadcom vendor nl80211 commands, Android private commands, /sys/wifi nodes or the
`dhd` module parameters that the unchanged Lineage HAL and supplicant use. The Samsung
firmware is a DHD/msgbuf image that also expects the Samsung OTP/CID/NVRAM selection logic.
DHD 101.16 already has compat code up to ~5.6, so the forward port is mostly mechanical and keeps
the HAL ABI. Cost: a large vendor tree (220k lines) compiled with -Wno-error.

**RC: new 6.12 DesignWare glue, 4.9 sequences.** Mainline `pci-exynos.c` only supports Exynos
5433 (different PHY/ELBI). The 4.9 driver is built on the 4.9 DesignWare API. So
`drivers/pci/controller/dwc/pcie-exynos9810.c` uses the 6.12 dwc core (config/iATU/bus scan)
and copies the 4.9 9810 CAL and ELBI sequences value for value.

## 3. Design

### RC (`pcie-exynos9810.c`, `include/linux/exynos-pci-{ctrl,noti}.h`)
- Probe: map windows, mask/clear ELBI IRQs, request SPI 253 shared and disable it, 4.9 probe
  sequence (INTx enables, PMU PHY isolation release, PHY config + I/A, PHY power down,
  NCLK_OFF), idle pin state. Link stays down. Channel 0 only.
- `exynos_pcie_pm_resume(0)` (4.9 poweron): wake mask, default pins, PHY power-down clear,
  history buffer, IRQ on, 4.9 establish_link (soft core reset, PERST# high 18-20 ms, L1 exit mode,
  linkdown reset manual, Q-channel gating off, PHY config + I/A, RC setup, LTSSM, 10 x 20 ms).
  First success: 100 ms, `dw_pcie_host_init()` enumerates, root port state saved. Later
  power-ons: `dw_pcie_setup_rc()` + root port state restored (4.9 did the same).
- `exynos_pcie_pm_suspend(0)` (4.9 poweroff): PME_Turn_Off/L23 (sleeping polls), PERST# low,
  LTSSM off, PHY power down, NCLK_OFF, idle pins, wake mask.
- DBI accesses while the link is not up are wrapped exactly like 4.9 rd/wr_own_conf
  (NCLK_OFF clear + RX elecidle ignore, then restore).
- Link-down IRQ -> history dump -> `exynos_pcie_register_event()` callback (DHD hang path).
- `exynos_pcie_l1ss_ctrl()` / `exynos_pcie_l1_exit()`: accepted, no-op (no L1SS in stage 1).
- PCI final fixup: async suspend off (4.9 quirk), `bus_dma_limit` = 32 bit.
- `exynos_pcie_get_status()`: read-only snapshot for the self-test.

### DHD (`drivers/net/wireless/bcmdhd_101_16`)
- Commit 8a5d26500 is the verbatim 4.9 import (provenance). `Kbuild` overrides the 4.9 Makefile
  with the exact 4.9 define set + forced `-include dhd_sec_feature.h`, minus
  `DHD_ALLOC_COHERENT_MEM_FROM_ATOMIC_POOL` and `WL_BAM` (see c3 commit message for the full
  list of API updates). Version-guarded changes only; 4.9 paths stay intact.
- Glue (`dhd_custom_exynos.c`): platform driver for `samsung,brcm-wlan`; 4.9 power sequence at
  probe (device_initcall, after the RC by link order), `-EPROBE_DEFER` until the RC exists;
  same `dhd_wlan_control` / `dhd_wlan_resources`. `dhd_module_init()` returns early unless the
  glue probed successfully: on every existing DTB DHD registers nothing.
- `ndo_siocdevprivate` added (HAL/supplicant `DRIVER` commands; not used by the self-test).

### Self-test (`dhd_crownlte_selftest.c`, scan helper in `wl_cfgscan.c`)
One kthread, armed only by `samsung,wifi-selftest` (ms in `samsung,wifi-selftest-ms`). Steps and
exact lines in section 7. The scan is a passive firmware escan with a synthetic request that is
never handed to cfg80211 (`cfg->scan_request` stays NULL, DHD's existing internal-scan case),
summarised from the escan buffer under `scan_sync`; only counts, channels, RSSI and BSSID OUIs are
printed (no SSIDs, no full BSSIDs). A userspace scan arriving meanwhile wins and is reported as
preempted. If wlan0 is already up (userspace), nothing is touched.

### DT
- `exynos9810.dtsi`: `pcie0: pcie@116a0000` (disabled).
- `exynos9810-crownlte-clocks.dtsi`: pins, `perst-gpios`, `wlan` node (disabled). Alive-bank pins
  (gpa0-7, gpa3-2) drop the 4.9 `*-pdn` properties: 6.12 alive banks have no PDN fields and
  pinctrl would reject the state.
- `exynos9810-crownlte-delayed-ufs-wifi-diag.dts`: delayed-UFS diagnostic + both nodes, self-test
  at 100 s with `samsung,wifi-selftest-no-firmware` (kernel-only vehicle: no /vendor, PID1
  restarts at 120 s).
- ROM-in-RAM: `exynos9810-crownlte-romram-wifi-s1.dts` = v23a flat DTS + the same nodes and the
  full self-test at 150 s (diff in `.dts.diff`, generator `romram-wifi-s1-gen.py`); not in the
  tree, like the other ROM-RAM DTBs.

### Config
`arch/arm64/configs/crownlte_wifi.fragment` on top of the ROM-RAM v23a .config (PCI, RC,
cfg80211, DHD with the 4.9 symbols, FW_LOADER_USER_HELPER + _FALLBACK; no PCIe port services,
no ASPM). cfg80211 pulls in the signed-regdb verification code (`CFG80211_REQUIRE_SIGNED_REGDB`
is not user-selectable without CERTIFICATION_ONUS); harmless, no regulatory.db on /vendor.

## 4. Deviations from 4.9

| Area | 4.9 | 6.12 stage 1 | Why |
|---|---|---|---|
| DMA path | PCIe SysMMU, 32-bit IOVA, IO-coherent | no SysMMU (bypass), non-coherent, 32-bit bus limit + swiotlb | SysMMU driver for PCIe not ported; non-coherent DMA is always correct with the DMA API, IO-coherent hardware with non-coherent software is safe |
| Ring allocation | `dma_alloc_coherent(GFP_ATOMIC)` | GFP_KERNEL when sleepable (DHD's own CAN_SLEEP branch) | atomic pool is small on non-coherent arm64 |
| L1SS / ASPM | L1.2 enabled after first link-up | off, link stays in L0 | removes the L1.2-exit/CDR failure mode for bring-up; costs power |
| MSI | off | off | same |
| Host bridge | registered at probe, `pci_rescan_bus()` at first link-up | registered at first link-up | 6.12 dwc core; avoids bridge-window resizing on rescan |
| Link-down without client | power cycle the link | log only | conservative |
| Long polls | udelay loops with IRQs off (up to 200 ms) | sleeping polls | 6.12 hygiene |
| Firmware loading | user-helper fallback via ueventd | same (config fragment) | parity |
| Glue | late_initcall, global GPIO numbers, sec class links | platform driver, gpiod, deferral, no sec class links | 6.12 APIs |
| WL_BAM | on (writes /data/misc/conn) | off | stage 1: no writes |
| File I/O | compiled out (`DHD_SUPPORT_VFS_CALL` off) | compiled out + `#error` guard + `dhd_write_file()` refuses | make the /efs guarantee explicit |
| PCIe sysfs `pcie_sysfs`, argos, idle-IP, pm_qos | yes | no | not needed for stage 1 |

## 5. Safety

- Nothing powers the Wi-Fi chip unless both DT nodes are enabled. Only the new diag DTB and the
  ROM-RAM `wifi-s1` DTB enable them.
- No file I/O from DHD at all (4.9 parity, now enforced); the only write path that could target
  /efs (`dhd_check_module_mac` -> `/efs/wifi/.mac.info`) is refused twice. No NVRAM/OTP writes:
  the BCM4361 has no flash, NVRAM is downloaded to dongle RAM, OTP is only read (CIS).
- MAC: read from OTP/firmware and set into dongle RAM only (4.9 behaviour). With
  `STA_RANDMAC_ENFORCED` DHD may also set a random MAC in dongle RAM; nothing is persisted.
- Self-test: passive scan only (no probe requests), no association, no softAP, no P2P, wlan0
  restored to down. It never touches wlan0 if userspace owns it.
- Worst cases: link training fails -> RC powers down, REG_ON low, `RESULT FAIL stage=platform`;
  firmware unavailable -> dev_open fails after the 60 s user-helper timeout, `RESULT FAIL
  stage=firmware/bring-up`; DMA broken -> DHD ioctl timeouts (5 s each) then FAIL. A PCIe bus
  error that the SoC turns into an SError would panic; panic=30 in the diag bootargs reboots.

## 6. Artifacts (build-only)

gekko `/mnt/ssd/host-builds/kernel-upgrade/panel-bright-work/ko-wifi-s1/` (same layout as
ko-v23a, usable as `KO=` for `rom-ram/pack-rom-ram.py`), built from crownlte-wifi 4c93de414
(this documentation commit changes no built file):

| file | sha256 |
|---|---|
| arch/arm64/boot/Image (24,985,608 B) | 66431a773da0aeb1043a30dbcf83f768e2462205864a2760c7c16fdd16fb40c3 |
| arch/arm64/boot/dts/exynos/exynos9810-crownlte-delayed-ufs-wifi-diag.dtb | 39356741b33c38c516004d6094843863d9c0fd1781ca0e4b03578abe1dda948a |
| arch/arm64/boot/dts/exynos/exynos9810-crownlte-romram-wifi-s1.dtb | dfd395d8a9ba1817b2465f29334652722e5bc18360c3c89a544e1080defb1529 |
| config | e09949383b65b8d73c7064a4e3135305dd148ff6ed8fa163d65c44c1b90e1635 |

The ROM-RAM DTS, its diff against v23a and the generator are in `panel-bright-work/dtb/`
(`exynos9810-crownlte-romram-wifi-s1.dts{,.diff}`, `romram-wifi-s1-gen.py`). Kernel footprint
`_end - _text` = 0x18b0000 (25.9 MB) from 0x92000000, below the first reservation at
0xa0000000; the loader budget in the parent RECOVERY layout is ~67.8 MB.

Build: base `.config` = ROM-RAM v23a IKCONFIG (`scripts/extract-ikconfig` of
panel-bright-work/ko-v23a Image), `merge_config.sh -m` with `crownlte_wifi.fragment`,
`olddefconfig`, `make ARCH=arm64 LLVM=-18 Image dtbs`. Note: v23a was built from
crownlte-panel-s2dos05-diag; integration-4 has no `CROWNLTE_S2DOS05_DIAG`, so that option is
absent here (the v23a DTB property for it is inert).

## 7. Trial plan

Vehicle A (kernel only, safest, do this first): RECOVERY diag boot (shim + delayed-UFS ramdisk,
`pack-acpm-q.py` with `KO=ko-wifi-s1` and `exynos9810-crownlte-delayed-ufs-wifi-diag.dtb`),
send READY only. The self-test runs at 100 s in no-firmware mode (PID1 restarts at 120 s) and
proves power, link, enumeration, chip id and the RC power-off path. Expected end:
`wifi selftest: RESULT PASS-NOFW chip=0x4361 rev=N`.

Vehicle B (firmware + scan, only after A passed): ROM-in-RAM, `rom-ram/pack-rom-ram.py` with
`KO=ko-wifi-s1`, `DTB_NAME=exynos9810-crownlte-romram-wifi-s1.dtb` (sha above), the current
rom-ram ramdisk, READY -> PROBE -> START. Lineage init's ueventd (inside the container,
/vendor mounted ro) serves `/sys/class/firmware/*` from /vendor/firmware. The self-test runs
at 150 s; the ramdisk deadline (600 s or more) covers it. If only READY is sent, nobody answers
the firmware fallback: DHD tries 4 times (`POWERUP_MAX_RETRY`) x 60 s, then
`RESULT FAIL stage=firmware/bring-up` at about 400 s. If Lineage turns Wi-Fi on by itself first,
expect `PASS-SHARED` or `stage=scan-preempted` (not a driver failure).

Expected log, in order (steps 1-4 for A and B, step 5 as marked):
1. `exynos9810-pcie 116a0000.pcie: RC0 ready (link down), I/A on, max GEN2 x1`
2. `exynos9810-pcie ...: poweron start, state 0`, `D state: ...`, `link up: ltssm 0x1?, GEN2 x1
   (max GEN2)` (GEN1 acceptable), PCI scan lines for `0000:00:00.0 [144d:ecec]` and
   `0000:01:00.0 [14e4:441f]` (or 4420/4421), `exynos9810: async suspend off, DMA limit 32 bit`,
   `poweron done, state 3`
3. `crownlte-brcm-wlan wlan: BCM4361 powered, PCIe ch0 link up, host-wake irq N`,
   `wifi selftest: armed for 100000 ms after boot, no firmware step` (A) or
   `armed for 150000 ms after boot` (B)
4. DHD attach at late_initcall_sync (`dhd_module_init in`, chip/OTP prints), wlan0 registered,
   then the 4.9 post-attach power-off: `poweroff done, state 0`
5. At 100 s (A) / 150 s (B):
   - `wifi selftest: platform probed=1 ret=0 WLAN_REG_ON=0 ...` (0 = off after attach, as 4.9)
   - `wifi selftest: pcie ret=0 host=1 state=down ... poweron ok/fail=1/0 poweroff=1`
   - `wifi selftest: endpoint 0000:01:00.0 14e4:441f ... driver pcieh`
   - `wifi selftest: chip 0x4361 rev N ...`
   - B only: `wifi selftest: wlan0 up ret=0 in N ms` (second power-on: `poweron done, state 3`
     without a new host bridge), `fw "<version>" clm "<version>"`, `fw_path .../bcmdhd_sta.bin_b2`,
     `nv_path ...nvram.txt_<module>_b2`, `mac oui ...`
   - B only: `wifi selftest: scan ret=0 channels=NN ... bss=K`, up to 5 `scan topN` lines
   - B only: `wifi selftest: wlan0 down again`
   - PASS: `wifi selftest: RESULT PASS-NOFW chip=0x4361 rev=N ...` (A),
     `wifi selftest: RESULT PASS chip=0x4361 rev=N bss=K` (B; `PASS-NO-BSS` if the scan
     completed but heard nothing)

Afterwards: save serial log, read pstore pmsg as usual, restore bright RECOVERY.

Failure signatures: `link is not up, try count: 10` + LTSSM history (PHY/PERST/REG_ON),
`iATU viewport reads all ones` (unexpected core), no `0000:01:00.0` (endpoint not answering:
REG_ON timing or CLKREQ), `stage=chip-id` (BAR0/backplane access), ioctl/`dhd_bus_init`
timeouts after firmware download (DMA: SysMMU bypass or 32-bit window), `stage=scan`.

## 8. Open risks

1. PCIe SysMMU left in a blocking state (not bypass) after reset -> DMA fails after firmware boot.
2. 36-bit addressing: capped at 32 bits by assumption, never proven on this RC.
3. Firmware in ROM-RAM depends on ueventd in the container answering the sysfs fallback.
   `CONFIG_FW_LOADER_USER_HELPER_FALLBACK` is global: every missing-firmware request now waits
   up to 60 s for userspace. Built-in callers in this config (llvm-nm on the build): DHD,
   cfg80211 `regulatory.db` (asynchronous `request_firmware_nowait`, so it only occupies a
   worker; no regulatory.db on /vendor, cfg80211 then uses the world domain), Mali
   `dummy_job_wa` (only for GPUs with TTRX_3485, not the G72), ACPM and ethtool use
   `request_firmware_direct` (no fallback). TMU firmware is built in.
4. dev_open holds rtnl while waiting for firmware (same as 4.9 via the HAL).
5. No L1SS: higher idle power, untested against the dongle's expectations.
6. First power-on registers the host bridge with the RC mutex held; a PCI driver that powers the
   link from its own probe would deadlock. DHD registers later (late_initcall_sync), so the
   order is safe; keep it that way.
7. System suspend/resume and OOB host-wake are untested (suspend path not in stage 1).

## 9. Remaining plan

- Stage 2 (Lineage userspace): HAL start/stop cycles via /sys/module/dhd/parameters/firmware_path,
  wpa_supplicant scan + association on a test AP, DHCP, throughput; check `/sys/wifi/*`,
  vendor commands (gscan, link stats, logger rings, APF, NAN) and `DRIVER` private commands.
  Decide on WL_BAM (writes /data/misc/conn) and DHD file I/O to /data/vendor/conn only.
- Stage 3: L1SS/ASPM per 4.9 (`exynos_pcie_config_l1ss`, I/A), SysMMU or coherent DMA if
  throughput needs it, suspend/resume with OOB wake, softAP/P2P, MFG firmware.
- Bluetooth (BCM4361 UART, rfkill) is separate.
