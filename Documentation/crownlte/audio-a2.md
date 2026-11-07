# crownlte 6.12: audio stage A2, ABOX DSP firmware boot (volatile)

Design: gekko `roadmap/audio-design.md`, section 5 "A2" (also 1.1, 2 item 5,
3.2, 6). Branch `crownlte-audio-a2`. Build only; not run on the phone yet,
not reviewed yet.

A2 boots the ABOX ("Calliope") audio DSP firmware in the DSP's own SRAM and
in kernel DRAM buffers, waits for the firmware's "ready to sing" message,
suspends it and powers the DSP CPU off again. It does this twice, then
reports one `audio selftest A2:` line. Everything it writes is volatile: a
reboot clears it.

A2 does not write the codec, the amps, the PMIC (LDO34 stays off, owner gate
G1), `/efs`, the UAIF/DSIF pins or pad retention, and it does not control
pd-aud power. Nothing can reach a speaker or the earpiece: the amps have no
driver and their bus stays disabled, the codec stays in reset, and the
ABOX's UAIF pins are not muxed.

## 1. Base and vehicle

Base: `crownlte-bt` e57473e08 (= `crownlte-integration-6` 4148bf525 +
Bluetooth B1) merged with `crownlte-audio-a1` 20ec6944b (=
`crownlte-suspend-s0` 60774ac87 + A1). Merge 9a54efa41, fixed by 4be0059de.

- int6 has the 9810 SysMMU v7.2 driver (DPU work, proven on HW) and the
  ROM-RAM support. A2 needs an ABOX SysMMU instance.
- suspend-s0 has the CL0 watchdog deadline API
  (`s3c2410wdt_crownlte_set_deadline()`), which A2 arms before the first
  pd-aud access, as A1 did. A1 itself is not enabled; its amp-bus nodes
  stay disabled.
- `crownlte-bt` rather than bare int6, so that the vehicle matches the last
  hardware-passed ROM-RAM env (`pack-romram-bt-b1-hal-r2-efs2`, f2a8ee3d):
  same kernel config plus one symbol, same DTB plus three nodes, same
  ramdisk, shim and parent.
- The merge had two additive conflicts: the dts Makefile, and
  `exynos9810-crownlte-clocks.dtsi` (BT block + A1 block). The suspend-s0
  code (watchdog, PM core, `exynos9810-pm` disabled in the DT) merged
  without conflicts.

Vehicle: **ROM-RAM**, not delayed-UFS. Lineage's ueventd serves
`/vendor/firmware` (VENDOR mounted ro), so the proprietary calliope
images are read from the phone at run time. They are **not** in the
kernel, the initramfs or the pack, so the licensing question does not
come up. The delayed-UFS vehicle would have needed the blobs embedded.

## 2. Minimal boot path instead of the vendor import

The 4.9 ABOX core is 16k lines: `sound/soc/samsung/abox/` 14k plus headers,
mostly ASoC components, DMA, UAIF, effects, dump and PM QoS. The boot path
alone is about 900 lines, so A2 extracts it into
`drivers/soc/samsung/exynos9810-abox-a2.c` (no ASoC, `CONFIG_SOUND` stays
off). The full import with the adaptation layer starts at A5 (section 11).

| 4.9 function | A2 |
|---|---|
| `samsung_abox_probe()` IOMMU part: `dma_alloc_coherent` 18 MiB + 512 KiB, `iommu_map` at 0x80000000 / 0x90000000, chip-id page 0x10000000 identity, VSS 0xA0000000/0xA1000000 | own unmanaged domain (`iommu_paging_domain_alloc`, `driver_managed_dma`), DRAM in 1 MiB chunks (64 KiB fallback) mapped at 0x80000000, IVA at 0x90000000, chip-id identity. **VSS not mapped** (no modem), dump 0xD0000000 not mapped (no `abox_debug`) |
| `abox_enable()`: GIC IRQ on, CPU gear max, `SYSREG MISC_CON=1`, `clk_enable(cpu)`, `clk_set_rate(audif, 24.576 MHz)`, `clk_enable(audif)`, regcache sync, `abox_gic_init_gic()`, UAIF pins "default", download, `abox_request_dram_on()`, `abox_cpu_power(1)`, `abox_cpu_enable(1)`, pad retention release | same order, except: **no CPU gear** (AUD stays at its boot DVFS level), **no regcache sync** (the 4.9 cache equals the defaults at first enable, so it writes nothing), **no UAIF pins**, **no pad retention write** |
| `abox_download_firmware()` (+ ext_bin area 1) | same, synchronous `request_firmware()` before any write |
| IRQ: `abox_gic_irq_handler()` + `abox_irq_handler()` | same IAR/EOI/DIR loop; system messages only: BOOT_DONE ("Calliope is ready to sing"), REPORT_LOG, REPORT_FAULT; the rest is logged |
| `abox_boot_done_work_func()`: `ABOX_RESUME`, restore mixer data, gear min | `ABOX_RESUME` only |
| `abox_disable()`: `ABOX_SUSPEND` + WFI poll, `dram_on(false)`, idle pins, pad retention | `ABOX_SUSPEND` + WFI poll, then CPU **enable and power off** (the 4.9 8895 path; on 9810, 4.9 relied on pd-aud power-off instead, which A2 does not do), `dram_on(false)`. Pins and retention not touched |

## 3. What A2 does

### 3.1 Probe (device initcall): the gate first

`/abox-a2` (`samsung,exynos9810-abox-a2`) has no registers. It:

1. Refuses unless `samsung,abox-a2-deadline-ms` (121000..900000) and
   `samsung,abox-a2-selftest-ms` (20000..300000, at least 60 s before the
   deadline) are set and a CPU other than CPU0 is online.
2. **Installs the CL0 watchdog deadline** and verifies it: enabled, reset
   on, still petted, counter moving. It defers while the watchdog is not
   bound. It also arms an hrtimer that stops petting at the deadline (the
   A1/S0 pattern).
3. Reads `AUD_STATUS` in the always-on PMU. Refuses unless bits 3:0 =
   0xf. A2 never powers pd-aud on.
4. **Takes the five AUD clocks through the clock framework** (Sol's A1
   point: AUD_STATUS on does not prove clocks). These are the first pd-aud
   accesses, all CAL rate reads in CMU_AUD:
   - It refuses if `PLL_OUT_AUD` or `DOUT_CLK_ABOX_ACLK` reads 0 Hz.
   - It sets AUDIF to 24.576 MHz, as 4.9 does; failure is only a warning.
   - It enables `cpu` (GATE_ABOX_DUMMY) and `audif`. On this tree both
     enables are hardware no-ops: the HWACG Q-channel gate and the DIV enable.
   - It never enables PLL_AUD: 4.9 only prepares it.
5. Opens `exynos9810_abox_a2_guard()`.

Refusal leaves the gate bound with the guard closed for the boot.

`sysmmu@17e00000` (9810 SysMMU, SPI 79, qos 15, pd-status AUD 0x4004) and
`abox@17c50000` both carry `samsung,abox-a2-guard`. The SysMMU probe in
`exynos-iommu.c` calls the guard before its first register read (its
bootloader-state dump). The guard returns:
- `-EPROBE_DEFER` while the gate is pending;
- `-EPERM` if the gate refused, the deadline is gone, or the deadline is
  less than 20 s away;
- `-ENODEV` if A2 is not built.

The ABOX device is a SysMMU consumer (`iommus`), so fw_devlink orders it
after the SysMMU. It calls the guard again.

ABOX probe: maps sfr/sysreg/sram/gicd/gicc, requests SPI 76 with
`IRQF_NO_AUTOEN` (affinity CPU1), allocates the 18 MiB DRAM and 512 KiB
IVA buffers (kernel view: write-combine vmap; cacheable alias cleaned
first), builds and attaches its domain, enables runtime PM. **No register
inside pd-aud is touched by the ABOX probe**: the SysMMU is runtime-suspended
until the self-test, so attaching only records the page table.

### 3.2 Self-test at `samsung,abox-a2-selftest-ms` (60 s), on CPU1 (pinned)

1. Watchdog deadline still armed and at least 30 s away; re-verified.
2. PMU snapshot. Refuses with nothing written if pd-aud is off, or if the
   ABOX CPU is already on (STATUS bit0 or OPTION bit28 set).
3. Reads SFR VERSION, SYSPOWER_CTRL/STATUS, SYSREG MISC_CON, CPU PC.
4. `request_firmware()`: `calliope_sram.bin`, `calliope_dram.bin`,
   `calliope_iva.bin` (required) and the seven 4.9 ext_bin files (optional,
   area 1 at the 4.9 offsets). Sizes are checked; SRAM firmware must end
   below the IPC area. Without firmware nothing is written.
5. `pm_runtime_resume_and_get()` → the SysMMU translates
   (`enable #1 for 17c50000.abox`).
6. Per boot (2 boots):
   1. ABOX GIC init (GICC PRIMASK 0xff, GICD CTRL 3, 40 priority words,
      GICC CTRL 3).
   2. `MISC_CON=1`, then the firmware copy: SRAM (rest zeroed), DRAM (rest
      zeroed), IVA, ext files.
   3. `SYSPOWER_CTRL=1`, then PMU `ABOX_CPU_CONFIGURATION` bit0 = 1 and
      `ABOX_CPU_OPTION` bit28 = 1.
   4. Poll `ABOX_CPU_STATUS` bit0 for up to 100 ms.
   5. Wait up to 5 s for BOOT_DONE.
   6. `ABOX_RESUME`, then the DSP runs for 1 s. A2 prints the firmware log
      tail if the firmware registered a DRAM log buffer.
   7. `ABOX_SUSPEND`, then poll `ABOX_CPU_STANDBY` bit28 (WFI) for up to
      100 ms.
   8. OPTION bit28 = 0, CONFIGURATION bit0 = 0, poll STATUS bit0 = 0, then
      `SYSPOWER_CTRL=0`.
7. GIC IRQ off.
8. If the CPU is off: `ABOX_CPU_CONFIGURATION` bit0 is set back to its
   boot value and the SysMMU is released (bypass). OPTION was 0 at boot
   and stays 0.
9. Result line, also in sysfs `a2_result`; `calliope_version` in sysfs.

If the CPU cannot be stopped, A2 keeps the runtime-PM reference, so the
SysMMU keeps translating and the DSP cannot DMA to physical addresses. It
logs this. The ABOX `.suspend` callback refuses system suspend while the
test is pending or running, or while the DSP runs. `.shutdown` aborts the
test and clears ENABLE_CPU.

### 3.3 Writes in an A2 boot (all volatile)

| Where | What | Precedent |
|---|---|---|
| CMU_AUD | AUDIF divider (rate 24.576 MHz) | 4.9 `abox_enable()` |
| PMU 0x14060000 | `ABOX_CPU_CONFIGURATION` bit0, `ABOX_CPU_OPTION` bit28 (on/off, ends as found) | 4.9 `abox_cpu_power/enable()` |
| SYSREG_AUD 0x17C1032C | `MISC_CON = 1` | 4.9 |
| ABOX SFR 0x17C50010 | `SYSPOWER_CTRL` 1 / 0 | 4.9 `abox_request_dram_on()` |
| ABOX GIC 0x17CF1000/0x17CF2000 | CTRL, PRIMASK, priorities, SGI (SOFTINT), EOI/DIR | 4.9 `abox_gic.c` |
| ABOX SRAM 0x17D00000 | firmware image, IPC mailboxes | 4.9 |
| kernel RAM (18.5 MiB, A2's own pages) | DRAM/IVA firmware images | 4.9 (`dma_alloc_coherent`) |
| ABOX SysMMU 0x17E00000 | CTRL/CFG/PT_BASE/flush through the 9810 SysMMU driver | DPU SysMMU path |
| CL0 watchdog | deadline in the ping path | A1 / S0 |

Not written: codec (SPI/GPIO reset), amps (HSI2C18 stays disabled, no
child), S2MPS18 (no PMIC access at all), `/efs`, UAIF/DSIF pin mux,
`PAD_RETENTION_ABOX_OPTION`, `GPIO_MODE_ABOX_SYS_PWR`, pd-aud (no DTZPC SMC
0x82000410, no CAL PD call), AUD DVFS/ACPM, PLL_AUD.

## 4. Code

| Commit | Content |
|---|---|
| 9a54efa41 | merge `crownlte-audio-a1` into `crownlte-bt` (base) |
| 08399408b | `drivers/soc/samsung/exynos9810-abox-a2.c`, `include/soc/samsung/exynos9810-abox-a2.h`, Kconfig `EXYNOS9810_ABOX_A2` (bool; depends on EXYNOS_IOMMU, S3C2410_WATCHDOG, FW_LOADER, MFD_SYSCON) |
| b2ca28d8e | `exynos-iommu.c`: guard call for SysMMU nodes with `samsung,abox-a2-guard` (others unchanged) |
| fa3027824 | `exynos9810-crownlte-clocks.dtsi`: `/abox-a2`, `sysmmu_abox`, `abox`, all disabled |
| 4be0059de | fix of the merge resolution (three stray diff context lines in the dtsi) |
| 464b9cd3c | a slowest boot over 2 s is a warning, not a failure (AUD not raised) |
| this doc | |

DT bindings (A2 only, not upstream):
- gate `samsung,exynos9810-abox-a2`: `samsung,pmu-syscon`, clocks
  `pll cpu audif dmic bus`, `samsung,abox-a2-deadline-ms`,
  `samsung,abox-a2-selftest-ms`;
- ABOX `samsung,exynos9810-abox-boot`:
  - reg `sfr sysreg sram gicd gicc` (gicc at least 0x1004 long, for DIR);
  - one interrupt;
  - `iommus`, `samsung,abox-a2-guard`;
  - `samsung,ipc-{tx,rx,tx-ack,rx-ack}-offset`;
  - `samsung,ext-firmware-names`/`-offsets`;
  - `samsung,abox-a2-boots` (1..2, default 2), `samsung,abox-a2-run-ms`
    (100..5000, default 1000).

## 5. Config and build

`.config` = `panel-bright-work/ko-bt-b1-r2/config` (e94d322e) +
`CONFIG_EXYNOS9810_ABOX_A2=y`, `olddefconfig`. The only other new lines are
`# CONFIG_EXYNOS9810_AUDIO_A1 is not set` and `# CONFIG_EXYNOS9810_PM is
not set` (both new symbols from the merge, off). `make ARCH=arm64
LLVM=-18 Image`, clang 18.1.3, `-j3`. The new driver and `exynos-iommu.c`
are clean with W=1; the full build has no new warnings. In-tree DTBs
`delayed-ufs-bt-diag` and `delayed-ufs-audio-a1-diag` build.

The ROM-RAM DTB is out of tree, like the BT one:
`panel-bright-work/dtb/romram-audio-a2-gen.py` appends the three nodes,
enabled, to `exynos9810-crownlte-romram-bt-b1.dts` (deadline 700 s,
self-test 60 s). Built with the kernel's `scripts/dtc/dtc`; the same
command reproduces the BT-B1 DTB bit for bit (adedcd7d). The `.dts` and
`.dts.diff` are next to the generator.

## 6. Artifacts

| Item | Value |
|---|---|
| Image | `panel-bright-work/ko-audio-a2/arch/arm64/boot/Image` sha256 `06f64997d2e84c0a5735147a44048e6b9bee9347c52b8cc8ea03bb8eb5a58ac5` (built at 464b9cd3c) |
| config | `panel-bright-work/ko-audio-a2/config` sha256 `197e5ff61b9a4501b3608aaccda928b9915fa8800f0cac7fb650a77a34835eaa` |
| DTB | `panel-bright-work/ko-audio-a2/arch/arm64/boot/dts/exynos/exynos9810-crownlte-romram-audio-a2.dtb` sha256 `18fb1b412739d4b51b813f60ac4b668c1b56b8094c5c14c0bd77adfd698b2a3d` |
| Pack | gekko `rom-ram/pack-romram-audio-a2/recovery.img` sha256 `6cdb80370fed9685331b8ea2584ae66489009f1c3a542887034e557edf3a0220` (68149248 bytes, mode 0600: the build-32 ramdisk contains the device MAC addresses; do not publish) |
| loader | `586f1bde8fdb72511017b87dba36f0c3b1ce5383a5643b0708212fcec5c97c00`, shim bbea959 |
| ramdisk | `rom-ram/build-32/diagnostic-initramfs.cpio.gz` `82771399d5af1ff8afc6b575ef8edddd57d5ea5cc30db67084e7a05ec848f3fa` (romram-adb 46a3670, DEADLINE 600 s) |
| parent / restore | `a8198be0...` / bright RECOVERY `8763860c...` |

Same env as `pack-romram-bt-b1-hal-r2-efs2`: only the Image and the DTB
differ. Packer `rom-ram/pack-rom-ram.py`; the shim worktree and shim-out
were removed after the loader build (see `PROVENANCE.txt`).

## 7. Running it

The usual ROM-RAM procedure (romram-adb `RUN-SHEET.md`, EFS_RO build-32
section):
1. Flash the pack to RECOVERY and start with `adb reboot recovery`.
2. On serial, send `CROWNLTE-UFS-READY-1`, then `CROWNLTE-UFS-PROBE-1`
   (intended in ROM-RAM: the read-only guarded probe), then
   `CROWNLTE-ROMRAM-START-1`.
3. Lineage boots from RAM (boot_completed ~35 s). The A2 self-test runs at
   60 s and takes about 3-5 s if the firmware loads at once.
4. The ramdisk reboots at 600 s. The A2 watchdog deadline (700 s) only
   fires if that reboot does not happen.
5. Afterwards, read `/sys/fs/pstore/pmsg-ramoops-0`, restore bright
   RECOVERY `8763860c` and wait for AshLooper `boot=booted`.

No physical help is needed: nothing is audible, and no headset is needed.
Optionally, while adb is up:
`cat /sys/devices/platform/soc@0/17c50000.abox/{a2_result,calliope_version}`.

If only READY is sent (no ROM stage), the firmware fallback has nobody to
answer: `calliope_sram.bin request failed (<errno>)` after the 60 s user-helper timeout, then FAIL
`firmware (nothing written)`. That is a safe result, not a driver bug.

## 8. Expected lines

Boot (in this order; the guard lines must come after the deadline lines):
```
s3c2410-wdt 10050000.watchdog: crownlte deadline: no reload after 700000 ms boot time (WTCON 0x..)
audio selftest A2: wdt (probe): deadline 700000 ms armed (WTCON=0x.. timeout 30 s, counter moving)
exynos9810-abox-a2 abox-a2: pd-aud: AUD_CONFIGURATION=0x.. AUD_STATUS=0x10000f (on)
exynos9810-abox-a2 abox-a2: AUD clocks (before): pll N cpu N audif N dmic N bus N Hz
exynos9810-abox-a2 abox-a2: AUD clocks (after): ... audif 24576000 ...
exynos9810-abox-a2 abox-a2: audio A2 gate open: watchdog deadline 700000 ms, self-test at ~60000 ms on CPU1
exynos-sysmmu 17e00000.sysmmu: audio A2 guard: watchdog deadline 700000 ms armed, pd-aud on, AUD clocks held: allowed at N ms
exynos-sysmmu 17e00000.sysmmu: bootloader state: CTRL=0x0 ... VERSION=0x.. (v7.2) ...
exynos-sysmmu 17e00000.sysmmu: Exynos9810 SysMMU v7.2 registered: 4.9 CFG 0x.. (QoS 15), ...
exynos9810-abox-boot 17c50000.abox: audio A2 guard: ...
exynos9810-abox-boot 17c50000.abox: dram firmware (0x1200000) alloc: 18 chunks, iva firmware (0x80000) alloc
exynos9810-abox-boot 17c50000.abox: own IOMMU domain: DRAM firmware at 0x80000000 (18 chunks), IVA at 0x90000000, chip-id page 0x10000000 identity; VSS/dump not mapped
exynos9810-abox-boot 17c50000.abox: audio A2 ABOX boot test scheduled at ~60000 ms on CPU1 (2 boots, 7 ext firmware files); nothing inside pd-aud touched yet
```
At 60 s:
```
audio selftest A2: start uptime=60xxx ms on CPU1 (pinned), boots wanted 2, run 1000 ms each
audio selftest A2: wdt (self-test): deadline 700000 ms armed (...)
audio selftest A2: pmu (before): AUD_STATUS=0x10000f ABOX_CPU_CONFIGURATION=0x1 ABOX_CPU_STATUS=0x20000 ABOX_CPU_OPTION=0x.. ABOX_CPU_STANDBY=0x.. PAD_RETENTION_ABOX_OPTION=0x.. GPIO_MODE_ABOX_SYS_PWR=0x..
audio selftest A2: abox: SFR VERSION=0x.. SYSPOWER_CTRL=0x.. SYSPOWER_STATUS=0x.. SYSREG MISC_CON=0x.. CPU PC=0x..
... calliope_sram.bin is loaded (34784 bytes) / SRAM firmware loaded / calliope_dram.bin is loaded (2513712 bytes) / calliope_iva.bin is loaded (57728 bytes) / 7 x "<ext>.bin is loaded"
exynos-sysmmu 17e00000.sysmmu: enable #1 for 17c50000.abox: ... readback CTRL=0x5 ...
audio selftest A2: boot 1: SRAM 34784 + DRAM 2513712 + IVA 57728 bytes copied, ext firmware 7/7
audio selftest A2: boot 1: abox_cpu_enable(1): STATUS=0x.. after N us
exynos9810-abox-boot 17c50000.abox: Calliope is ready to sing (version:XXXX)
audio selftest A2: boot 1: ready to sing after N ms, SFR VERSION=0x..
audio selftest A2: boot 1: ABOX_RESUME ipc acked
(optional) firmware log buffer N at 0x8....... / firmware log tail ... / calliope: ...
audio selftest A2: boot 1: ABOX_SUSPEND ipc acked, WFI seen (STANDBY=0x1.......)
audio selftest A2: boot 1: ABOX CPU off (STATUS=0x..)
(boot 2: the same lines)
exynos-sysmmu 17e00000.sysmmu: disabled (bypass): WRITE CTRL 0x0, CFG 0x0
audio selftest A2: pmu (after): ... ABOX_CPU_CONFIGURATION=0x1 (as before) ... ABOX_CPU_OPTION=(as before)
```
**PASS line:**
```
audio selftest A2: PASS boots=2 version=XXXX boot_ms_max=N faults=0 fw_faults=0 ipc_rx=N dsp=off
```
PASS means:
- both boots answered BOOT_DONE within 5 s;
- RESUME and SUSPEND were acked;
- WFI was seen and the CPU reads off after each boot;
- 0 SysMMU faults, 0 firmware fault reports;
- the firmware does not report itself incompatible.

`boot_ms_max` over 2000 ms gives
`slowest boot N ms is over the 2000 ms design target (AUD not raised)`;
that is information, not a failure (4.9 raised the AUD gear before booting,
A2 does not).

## 9. Failure signatures

| Line / symptom | Meaning | Next |
|---|---|---|
| `deferred probe pending: ... waiting for the guarded CL0 watchdog (ABOX held back)`; SysMMU/ABOX deferred | watchdog driver not bound; nothing in pd-aud touched | check s3c2410-wdt probe |
| `audio A2 refused (...)`, then `audio A2 guard: gate refused (...)` on 17e00000.sysmmu / 17c50000.abox | gate refused: no deadline, pd-aud not on, a clock at 0 Hz, bad DT. Fail closed | reason in the line. `pd-aud is not on` answers design question 1 for this boot |
| `SError` / `Asynchronous SError Interrupt` near the `AUD clocks` or `bootloader state` line | pd-aud blocks not accessible from the non-secure side (DTZPC) or clock gated | stop. Next step would be the DTZPC SMC 0x82000410 restore (design 2 item 2) |
| `exynos-sysmmu 17e00000.sysmmu: power domain off (...)` / `translation already enabled by the bootloader` / `not a SysMMU v7` | SysMMU probe refused | record CTRL/VERSION from the line |
| `... request failed (<errno>)` after ~60 s, `FAIL ... fails: firmware (nothing written)` | user-helper fallback not answered (ROM stage not started, ueventd not up) | vehicle problem; nothing was written |
| `FAIL ... fails: abox-cpu-already-on (nothing written)` | DSP already running at 6.12 boot (A1 saw it off) | record the pmu line |
| `boot 1: abox cpu enable timeout (STATUS=...)`, `fails: cpu-enable-timeout` | PMU sequence, pd-aud or the ABOX CPU clock missing | record pmu/clock lines |
| `boot 1: no "ready to sing" within 5000 ms (ipc rx N, gic spi N)` + `ABOX CPU registers (no boot done)` + `fails: no-ready-to-sing` | DSP crashed or waits for something: GIC/IPC offsets, a missing mapping (VSS), AUD clock | read PC/R0-R14 and the firmware log tail. ipc rx 0 with a moving PC means the SGI to the AP is missing |
| `abox_iommu_fault_handler: IOVA 0x... read/write (fault #n)` + SysMMU `[READ|WRITE] ... FAULT occurred at ...`, `fails: sysmmu-faults` | firmware touched an unmapped IOVA: 0xA0000000/0xA1000000 = VSS (modem shm, not mapped in A2), 0xD0000000 = dump, other = layout difference | record IOVAs. VSS mapping waits for the modem port |
| `fault N (...) reported from calliope`, `fails: firmware-fault` | firmware exception (data/prefetch abort, os error, vss error) | record params and log tail |
| `ABOX_RESUME ipc NOT acked`, `fails: resume-ipc` | DSP not polling its mailbox after boot | record |
| `ABOX_SUSPEND ipc NOT acked` / `WFI NOT seen`, `fails: stop` | firmware did not reach WFI; CPU was powered off anyway | record STANDBY; if `ABOX CPU off` follows, it was stopped |
| `ABOX CPU still on after enable/power off` + `ABOX CPU could not be stopped: SysMMU kept translating, system suspend refused`, `dsp=LEFT-RUNNING` | DSP could not be stopped; the SysMMU keeps it inside its own buffers until the ramdisk reboot | report; reboot clears it |
| `Calliope is not compatible with the driver`, `fails: incompatible` | firmware reports an ABI 'A' marker (4.9 check) | record version |
| `FAIL: ... fails: cpu0 (nothing touched)` | CPU1 went offline before 60 s | not an audio problem |
| `... fails: wdt` | deadline gone or within 30 s | check wdt lines |

## 10. Recovery path

As in the other ROM-RAM trials:
- The ramdisk reboots at 600 s; the namespace is killed at 575 s.
- The A2 deadline stops watchdog petting at 700 s, so the CL0 watchdog
  resets the SoC by about 730 s if the ramdisk reboot did not happen
  (e.g. a CPU wedged on an AUD access).
- A panic (SError) also ends in a reset: `panic=` and the watchdog.
- If the phone does not come back: owner force restart (Power + Vol
  Down ~7-10 s), the shim bounces to Lineage, restore bright RECOVERY
  `8763860c` by hand.

## 11. Risks

- **First non-secure access to SYSREG_AUD, ABOX SFR/SRAM/GIC and the ABOX
  SysMMU in 6.12.**
  - pd-aud is on and pinctrl_aud (same block) is already used by 6.12, but
    whether the DTZPC lets the non-secure side reach these blocks without
    the restore SMC is unproven.
  - Failure mode: SError, then a reset. That needs no owner help.
- **DSP CPU off by PMU power clear (8895 path) instead of pd-aud power-off
  (9810 path in 4.9).** If STATUS does not go to 0, the DSP stays in WFI
  behind a translating SysMMU until reboot. Its ABOX-internal watchdog
  (WDT_AUD) state after `ABOX_SUSPEND` is unknown.
- **Second boot after a CPU power cycle without a pd-aud cycle**: if the DSP
  needs pd-aud reset for a clean restart, boot 2 fails (`no-ready-to-sing`)
  while boot 1 passed. That is information for A5, not damage.
- **VSS not mapped**: if the firmware touches modem shared memory at boot,
  expect SysMMU faults at 0xA0000000 (no damage: the SysMMU blocks the
  access).
- **AUD at boot DVFS level**: slower boot; the firmware might expect the 4.9
  gear (boot timeout 5 s).
- **SysMMU bypass after release**: the SysMMU goes to bypass only after
  STATUS reads off. If STATUS were wrong and the DSP still ran, DMA would
  hit physical addresses 0x80000000.. (kernel RAM). The two status polls
  (WFI and power) reduce this.
- **ext firmware**: `dsm.bin`/`dsm_tune.bin` are loaded unchanged into DSP
  DRAM, as 4.9 does. No DSM parameter is changed (G5) and no amp exists.
- **Firmware user-helper**: up to 60 s per missing file; the 7 ext files
  exist on V.
- **Merge of suspend-s0 into the ROM-RAM kernel**: the s0 watchdog changes
  (deadline in the ping path, keep-armed suspend) now run in ROM-RAM for
  the first time. Without a deadline they behave as before.
  `exynos9810-pm` is not built.
- Not reviewed by Sol yet.

## 12. Plan after A2

- A3 [G1]: codec, mainline madera on SPI. Not affected by A2.
- A5: ABOX under ASoC. Import the 4.9 `sound/soc/samsung/abox/` verbatim
  plus the adaptation layer from design 3.2, reusing from A2:
  - the domain/IOVA setup (`a2_iommu_setup`, `a2_alloc_buffers`);
  - the GIC/IPC code and the PMU sequence;
  - the guard/deadline pattern.
  Missing for A5:
  - a pd-aud genpd provider (CAL PD + DTZPC SMC, from the G3D code);
  - AUD DVFS (`dvfs_aud`) for the CPU gears;
  - the VSS mapping (modem);
  - RDMA/WDMA/UAIF components with dummy codecs;
  - the machine driver.
  A2's result decides two things:
  - whether boot 2 needs a pd-aud cycle;
  - whether the firmware needs VSS at boot.
