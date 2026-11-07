# crownlte Bluetooth on 6.12: stage B1 (BCM4361 BT over UART, rfkill)

Status: B1-read ran on hardware once (2026-11-07 00:11, section 7); fix round 1 (section 8) is
build-only, not run on the phone. Branch `crownlte-bt` on `crownlte-wifi` (15cebae03, Wi-Fi
working on hardware), so Wi-Fi and BT are in one kernel. Design and evidence:
`roadmap/bluetooth-design.md` on gekko (sections 1-5); this file covers what B1 implements, how
to run the three trials and how to read their logs.

## 1. What B1 contains

| Commit | Change |
|---|---|
| 02466b11f | `exynos-usi`: `samsung,no-sw-conf` (9810 only) for USIv2 blocks with no SYSREG selector: no syscon lookup, no SW_CONF write or readback; USI_CON reset clear, CLKREQ_ON and the 9810 readback stay. Log `USIv2 configured: sw_conf=none con=... option=...` |
| 7ce78dbc1 | `samsung_tty`: `samsung,exynos9810-uart` (GS101 layout: 32-bit access, UFRACVAL, one baud clock `clk_uart_baud0`, FIFO size from DT) |
| e2637f71d | `drivers/misc/samsung-bcm43xx-bt.c` (`CONFIG_SAMSUNG_BCM43XX_BT`), `arch/arm64/configs/crownlte_bt.fragment` |
| 4bbaeb456 | DT: BT UART, USI10_I2C bus, PCAL6524 node, `/bluetooth`, `serial1` alias (all disabled in `exynos9810-crownlte-clocks.dtsi`); diag DTBs `...-delayed-ufs-bt-diag` (B1-read) and `...-delayed-ufs-bt-hci-diag` (B1-hci) |
| d3a3b71c2 | self-test: a UART clock that cannot do 3 Mbaud no longer skips the 115200 HCI step (`stage=uart-3m` at the end) |
| 28f292c33 | fix round 1 (section 8): full expander validation before/after every P0_7 write with an unsafe latch, BT_REG_ON state OFF/ON/ON (bootloader)/UNKNOWN, power cycle from ON (bootloader), ttySAC1 lookup, tty release order, PM/shutdown quiesce, no SYSREG reads |
| 4a086cc3b | `exynos-usi`, DTS comments: hardware evidence for the wrapper readbacks, trial descriptions |

Hardware path (from 4.9, see the design): HCI on `uart@10840000` = ttySAC1 (SPI 422, gpq0-0..3
RXD/TXD/RTS/CTS function 2 pull-up, 256-byte FIFO); BT_REG_ON = PCAL6524 P0_7 at 0x22 on
`hsi2c@10930000` (USI10_I2C, SPI 437, gpp6-2/3 function 3); BT_WAKE = gpg2-0 (output, high =
assert); BT_HOST_WAKE = gpa2-3 (EINT, high = chip has data).

### Driver behaviour (`samsung,bcm43xx`, platform device `bluetooth`)

- rfkill `bcm43xx Bluetooth`, type bluetooth, created with `rfkill_init_sw_state(true)`: it starts
  blocked and `rfkill_register()` calls no `set_block()` (persistent). `CONFIG_RFKILL_INPUT` is off
  because its `__rfkill_switch_all()` at registration would call `set_block()`. Path:
  `/sys/devices/platform/bluetooth/rfkill/rfkill0` (vendor rc chowns rfkill0, vendor sepolicy labels
  exactly this path). The rfkill core calls `set_block()` on every write, also a block while
  already blocked; that is the recovery path below.
- BT_REG_ON power state, read-only in `/sys/devices/platform/bluetooth/bt_reg_on_state`
  (`OFF`, `ON`, `ON (bootloader)`, `UNKNOWN`, plus ` unsafe` once latched):
  - probe writes nothing. P0_7 an output driven high (the bootloader leaves it so, section 7) and
    everything else sane (port 0 push-pull, P0_7 polarity not inverted, input level = output) ->
    `ON (bootloader)`; driven low or an input -> `OFF`; expander unreadable/bound/insane ->
    `UNKNOWN` + unsafe (never written; BT_WAKE stays low, the driver never asserted power).
  - unblock from `OFF`: BT_WAKE=1, P0_7 output bit 1, P0_7 direction output (4.9 order), 100 ms
    (4.9), host-wake edge counting on -> `ON`. Unblock from `ON (bootloader)`: power cycle
    (validated P0_7 low, BT_WAKE low, 500 ms, then as from `OFF`). Unblock from `UNKNOWN`:
    refused (`block (power off) first`).
  - block from `ON`, `ON (bootloader)` or `UNKNOWN` (recovery): counting off, P0_7 output bit 0
    (stays an output, as 4.9), BT_WAKE=0 -> `OFF`. Block from `OFF`: nothing to do.
  - `UNKNOWN` is recorded before the first expander write of every transition and replaced only
    by a validated result. BT_WAKE goes high with it (power may be asserted) and is lowered only
    after a validated `OFF`. No rollback writes: a failed transition stays `UNKNOWN`.
  - `set_block` errors are returned to the rfkill core, which then keeps the old soft state.
- BT_WAKE is held high for the whole powered time (libbt's LPM hooks are no-ops; LPM is B2).
- BT_HOST_WAKE: both-edge IRQ, requested disabled (`IRQF_NO_AUTOEN`), enabled only in `ON`,
  counts edges, disabled after 10000 edges in one session (`host-wake storm`). Not a wakeup
  source. If the IRQ cannot be obtained the driver logs it and still registers the rfkill.
- Suspend (`prepare`) and shutdown quiesce the driver: set `quiescing`, cancel the self-test if
  pending or make it abort at its next check (HCI waits are woken) and join it, then take the
  driver mutex once to wait out an rfkill transition in flight. While quiescing, rfkill
  transitions return `-EBUSY` (`rfkill block|unblock rejected: suspend/shutdown in progress`);
  `complete` clears it after resume. The self-test is not re-armed. The power state is not
  changed by suspend or shutdown (4.9 had no hooks either).
- `suppress_bind_attrs`: the driver is never unbound.

### PCAL6524 handling (pin 7 only)

The expander also drives NFC VEN (P1_3), NFC firm (P1_1), wireless charging `wpc_en` (P1_0), the
DP switch (P0_1, P0_4) and a force-sensor line (P2_5). Mainline `gpio-pca953x` was not used: its
probe writes all three polarity registers (`device_pca95xx_init`) and its resume syncs every cached
register back. Instead:

- No GPIO driver binds to the node (`compatible = "nxp,pcal6524"`, `CONFIG_GPIO_PCA953X` off). The
  BT driver looks up the I2C client by phandle (`samsung,bt-reg-on-expander`) and refuses to touch it
  if any driver is bound to it.
- Probe reads every readable register once (baseline, below) and writes nothing. 4.9 drove
  BT_REG_ON low at probe right after its own full expander init; 6.12 does neither, the first
  rfkill block/unblock (or the B1-hci self-test) drives it low.
- Every P0_7 write (rfkill on/off, power cycle, self-test, self-test cleanup) is
  validate -> record `UNKNOWN` -> write -> read back -> validate:
  - validate = no driver bound; all registers of the dump read without error (the extension
    registers too if they answered at probe); not one constant value (stuck bus); every
    non-volatile register equal to the probe baseline; output/configuration port 0 equal to the
    tracked expected value (bit 7 ignored only while `UNKNOWN`); P0_7, if an output, reading the
    level it drives.
  - write = output port 0 (0x04), then on power-on configuration port 0 (0x0c), each byte with
    only bit 7 changed, each read back.
- "Unsafe" latch (until reboot; nothing is written to the expander again, also no power-off and no
  rollback): any failed validation or mandatory read, a readback that fails or differs, a second
  I2C write transfer error. One write transfer error per boot is tolerated: the state becomes
  `UNKNOWN`, the expander is re-validated read-only (bit 7 ignored) and a later block can recover
  through a validated power-off. Messages: `expander UNSAFE (<why>, <err>): no further expander
  writes, BT_REG_ON state <state>`; `exp CHANGED <reg>[i] expected=.. now=..`; `P0_7 driven X but
  reads Y`; then `power on|off ... failed: <err>, BT_REG_ON state <state>, expander unsafe`.
- No `reset-gpios`; gpg3-6 (expander RESET#, active low) is only read, by the self-test.

Registers dumped (2 log lines): `in` 0x00-02, `out` 0x04-06, `pol` 0x08-0a, `cfg` 0x0c-0e, `drv`
0x40-45, `latch` 0x48-4a, `pull_en` 0x4c-4e, `pull_sel` 0x50-52, `int_mask` 0x54-56, `int_stat`
0x58-5a, `out_cfg` 0x5c, and the PCAL6524 extensions 4.9 never read: `int_edge` 0x60-65, `in_stat`
0x6c-6e, `out_indcfg` 0x70-72, `debounce` 0x74-76 (all answered on hardware; `--` if they ever do
not answer at probe, then they are left out of the comparison). Volatile, never compared: `in`,
`int_stat`, `in_stat`. Reading the input ports clears a pending expander interrupt; nothing on
6.12 uses that interrupt.

### Self-test (`bt selftest:`)

Armed only by `samsung,bt-selftest` (+ `samsung,bt-selftest-ms`, time since boot). Runs once on
`system_long_wq` under the driver mutex. Skips (`RESULT SKIP userspace owns BT ...`) if the rfkill is
unblocked, the state is `ON` or ttySAC1 is open; `RESULT ABORT suspend/shutdown` if quiescing.

1. `bus`: GPG3 CON/DAT/PUD (expander reset gpg3-6), read-only (4.9 precedent); expander client,
   bound driver, baseline; probe dump and a fresh validated dump (`now`). FAIL on no baseline, unsafe,
   a bound driver or any difference (which also latches unsafe). Prints P0_7 and the power state.
   (The SYSREG PERIC1 +0x3c/+0x40 reads of the first build are gone: values known, section 7.)
2. `uart`: the serial device has a driver; `ttySAC1` is looked up by dev_t in the tty class and must
   have the platform device among its ancestors (6.12: parent `10840000.serial:0.0`, the serial_base
   port device); with the UART clocks enabled: USI_CON/USI_OPTION (readable, section 7) and
   ULCON/UCON/UFCON/UMCON/UFSTAT/UMSTAT/UBRDIV/UFRACVAL; `clk_get_rate` of `clk_uart_baud0` and the
   baud error samsung_tty will produce at 115200 and 3000000 (div = rate/baud, UBRDIV = div/16-1,
   UFRACVAL = div&15). FAIL (`uart`) if the tty is missing or belongs to another device, a register
   window is missing or the 115200 error exceeds 2 %; `uart-3m` (reported at the end) if the clock is
   below 48 MHz or the 3 Mbaud error exceeds 2 %.
3. `rfkill`: our rfkill device must be `rfkill0` with sysfs path
   `/devices/platform/bluetooth/rfkill/rfkill0`. A state `UNKNOWN` here gives `stage=power`.
4. With `samsung,bt-selftest-no-power`: stop, `RESULT PASS-NOPOWER, state <state>`.
5. Open ttySAC1 in the kernel (`tty_kopen_exclusive`, then the serdev-ttyport termios: raw 8N1,
   CRTSCTS, CLOCAL, 115200; receive through temporary `tty_port` client ops). Power on through the
   rfkill path (from `ON (bootloader)`: validated P0_7 low, 500 ms, validated P0_7 high, 100 ms),
   every validation printed (`before/after power off`, `before/after power on`). HCI Reset
   `01 03 0c 00` (1 s, 3 tries 100 ms apart), Read Local Version `01 01 10 00`, Read BD_ADDR
   `01 09 10 00` (only the OUI is printed). No patchram download.
6. Cleanup, always: validated power off (refused if unsafe), then release the tty: reception off,
   TX flush, port close, flip buffers dropped, flip worker joined, default client ops restored while
   the kopen still excludes userspace, `tty_kclose()` last (also on a failed open).
   `RESULT PASS, state OFF` or `RESULT FAIL stage=<bus|uart|rfkill|power|hci|uart-3m>, state <state>`.

## 2. Config and build

`.config` = `panel-bright-work/ko-wifi-s1/config` + `crownlte_bt.fragment` (`merge_config.sh -m`,
`olddefconfig`). The resulting difference to the Wi-Fi config is exactly `CONFIG_RFKILL=y` and
`CONFIG_SAMSUNG_BCM43XX_BT=y` (RFKILL_INPUT, RFKILL_GPIO, GPIO_PCA953X, BT, SERIAL_DEV_BUS off).
`make ARCH=arm64 LLVM=-18 Image dtbs`, clang 18.1.3; the new and changed files build with W=1
without warnings (also after fix round 1). Kernel footprint `_end - _text` = 0x18c0000 (Wi-Fi build 0x18b0000).

With `CONFIG_RFKILL=y`, cfg80211 also registers a wiphy rfkill (type wlan) when DHD attaches at
`late_initcall_sync`. The BT driver probes at the latest in the deferred-probe flush at
`late_initcall` (it defers until the I2C client exists), so it should be `rfkill0` and Wi-Fi
`rfkill1`. B1-hal checks this with Wi-Fi present.

## 3. Artifacts

gekko `/mnt/ssd/host-builds/kernel-upgrade/`, built from crownlte-bt 4a086cc3b (fix round 1; the
documentation commit after it changes no built file). Config unchanged (same `.config` as the first
build, e94d322e).

| file | sha256 |
|---|---|
| `panel-bright-work/ko-bt-b1-r1/Image` (25,053,192 B) | 34cbe71170bddcb1e2de3119ae5722451b531a29e8d8780c81092bf583de4f45 |
| `panel-bright-work/ko-bt-b1-r1/config` | e94d322ef72eac75fc9507959106a001d47a35e199b5b42ecbe1bde77b5c30bc (unchanged) |
| `.../exynos9810-crownlte-delayed-ufs-bt-diag.dtb` (B1-read) | 786eedbb9dfcc3ce6d9947e705ff7858e16e9a6d34e3555e0cf88c9baad2b8fa (unchanged) |
| `.../exynos9810-crownlte-delayed-ufs-bt-hci-diag.dtb` (B1-hci) | 0be846a462f1403322e95f43521ef2478a96a978a2b3b22eaf6dd5efa5948027 (unchanged) |
| `.../exynos9810-crownlte-romram-bt-b1.dtb` (B1-hal, unchanged) | adedcd7dff302ccd7f14400e061fcf8d71a3289b5b8e4f58b500edb124f559c5 |

`panel-bright-work/ko-bt-b1-r1/` is in the `KO=` layout (Image, config,
`arch/arm64/boot/dts/exynos/*.dtb`, SHA256SUMS); hard links `images/Image-bt-b1-r1-4a086cc3b`,
`images/config-bt-b1-r1-4a086cc3b`. The DTS sources changed only in comments; the two diag DTBs are bit-identical to the first round. The ROM-RAM
DTB is the one from the first round (`panel-bright-work/dtb/`, `romram-bt-b1-gen.py`:
`romram-wifi-s1` DTS plus exactly the BT nodes, no-power BT self-test at 40 s).

Packed trials (all with shim bbea9590, parent RECOVERY a8198be0, restore RECOVERY 8763860c; each
directory has `PROVENANCE.txt`, because the delayed-UFS packer writes a fixed purpose/kernel_source
into `manifest.json`):

| trial | directory | DTB | ramdisk | recovery.img sha256 |
|---|---|---|---|---|
| B1-read | `bt-b1-read-r1-20261107` | delayed-ufs-bt-diag | delayed-UFS 506232b1 | 8e70229f1ead59de66c7d106f4519fba3193f5809b6e1bd2a1b74285562b4878 |
| B1-hci | `bt-b1-hci-r1-20261107` | delayed-ufs-bt-hci-diag | delayed-UFS 506232b1 | 142e9e02c3ab775571dd785672710a6b8178577172d24f78d944c1c4951de901 |
| B1-hal | `rom-ram/pack-romram-bt-b1-hal-r1` | romram-bt-b1 | build-30 e615cc8d (ADB, early DONE) | 4da04c783c7af73377f3d9c6d105b912adaf04b205e430eb15a56ce9c07e057b |

The first-round packs (`bt-b1-read-20261107` 218efc06, `bt-b1-hci-20261107` e9dc9073,
`rom-ram/pack-romram-bt-b1-hal` f458213d) and `panel-bright-work/ko-bt-b1` were deleted on gekko
(copies on the PC). Do not run the old B1-hci or B1-hal packs: that driver refuses power (taint)
and lacks the fixes.

## 4. Trials, expected lines, failure signatures

Order: B1-read, then B1-hci only if B1-read passed and its dumps look sane, then B1-hal. All three
go into RECOVERY the usual way; afterwards save the serial log, read pstore, restore bright RECOVERY.

### B1-read (kernel only, nothing written to the expander, chip power untouched)

Delayed-UFS vehicle, READY only. Self-test at 93 s (PID1 restarts at 120 s). Expected after fix
round 1 (values from the 2026-11-07 run where known; `N` = to record; the probe-time lines may
interleave differently, the self-test lines come in this order; every self-test line carries the
`samsung-bcm43xx-bt bluetooth: ` prefix):

```
exynos-usi 108400c0.usi: USIv2 configured: sw_conf=none con=00000000 option=00000003
exynos-usi 109300c0.usi: USIv2 configured: sw_conf=none con=00000000 option=00000001
exynos5-hsi2c 10930000.i2c: 9810 timing: clock=N bus=400000 div=N SCL=N
10840000.serial: ttySAC1 at MMIO 0x10840000 (irq = 73, base_baud = 0) is a S3C6400/10
samsung-bcm43xx-bt bluetooth: probe exp in=90,08,20 out=90,08,00 pol=00,00,00 cfg=6f,f5,ff
samsung-bcm43xx-bt bluetooth: probe exp drv=00,00,00,00,00,00 latch=00,00,00 pull_en=00,00,00 pull_sel=00,00,00 int_mask=ff,ff,ff int_stat=00,00,00 out_cfg=00 int_edge=00,00,00,00,00,00 in_stat=90,08,20 out_indcfg=00,00,00 debounce=00,00,00
samsung-bcm43xx-bt bluetooth: BT_REG_ON (P0_7) driven high by the bootloader: state ON (bootloader), not written at probe
samsung-bcm43xx-bt bluetooth: rfkill "bcm43xx Bluetooth" = rfkill0 (blocked), host-wake irq 82, expander 0-0022 P0_7 state ON (bootloader)
samsung-bcm43xx-bt bluetooth: bt selftest: armed for 93000 ms after boot, no power step
... at 93 s:
bt selftest: start at N ms, mode no-power
bt selftest: bus exp_reset gpg3-6 fn=1 level=1 pud=0 (gpg3 con=01000000 dat=43)
bt selftest: bus expander 0-0022 addr 0x22 driver=none probe_baseline=ok unsafe=0
bt selftest: probe exp ...            (2 lines, = the probe dump)
bt selftest: now exp ...              (2 lines; no "exp CHANGED", no "UNSAFE" line)
bt selftest: bus P0_7 (BT_REG_ON) now: output, out=1, level=1, state ON (bootloader)
bt selftest: uart 10840000.serial driver=samsung-uart tty=ttySAC1 (parent 10840000.serial:0.0)
bt selftest: uart usi con=00000000 option=00000003
bt selftest: uart regs ulcon=... ucon=... ufcon=... umcon=... ufstat=... umstat=... ubrdiv=... ufracval=...
bt selftest: uart clk=200000000 Hz err115200=+0.00% err3000000=+1.01%
bt selftest: rfkill rfkill0 /sys/devices/platform/bluetooth/rfkill/rfkill0 type=bluetooth soft_blocked=1
bt selftest: RESULT PASS-NOPOWER, state ON (bootloader)
```

The probe dump should equal the 2026-11-07 one (section 7); any other value there is worth a look
before B1-hci (bootloader variation or a reset expander).

Failure signatures:
- `expander N-0022 unreadable (-6): BT power disabled` (NACK) + `exp_reset ... level=0`: expander held
  in reset by gpg3-6 (design risk 2). Do nothing; report. Releasing reset would return all 24 pins
  to inputs (NFC VEN, wpc_en, DP switch): owner decision.
- `unreadable (-110)` or `-5` / i2c timeouts without NACK, or `exynos5-hsi2c ... 9810 timing` missing
  or `-EINVAL`: bus / clock problem.
- `deferred probe pending: ... expander i2c client not yet registered`: the I2C adapter never came up
  (look for `USIv2 configuration readback failed` on 109300c0 or hsi2c probe errors).
- `exp CHANGED <reg>[i] expected=.. now=..` + `expander UNSAFE`: something else is writing the
  expander or it reset between probe and 93 s: stop.
- `expander UNSAFE (P0_7 input polarity inverted | port 0 open-drain | P0_7 level differs from its
  output at probe ...)` and `state UNKNOWN ... UNUSABLE (unsafe)`: the probe state is not the one
  seen on 2026-11-07; driver will not touch the expander. Report.
- `tty=MISSING` (no ttySAC1: alias / UART probe) or `tty=ttySAC1 NOT-THIS-UART (parent X)`
  (ttySAC1 is another UART): `RESULT FAIL stage=uart`.
- `uart clk=0` or `err115200` above 2 %: `stage=uart`. `3000000 baud not reachable within 2%` and
  `RESULT FAIL stage=uart-3m`: B1-hci can still run (HCI at 115200), libbt's 3 Mbaud will need a
  `clk_set_rate` (design risk 6).
- `rfkill rfkill1 ...` / `RESULT FAIL stage=rfkill`: another rfkill registered first.
- `BT_HOST_WAKE irq unavailable (N)`: edge counting off; not fatal for B1.

### B1-hci (kernel only, power-cycles the BT core once)

Same vehicle and timing. Everything above up to the rfkill line, then (the chip comes from `ON
(bootloader)`; `out`/`cfg`/`in` port 0 shown as expected from the 2026-11-07 dump):

```
bt selftest: tty ttySAC1 open: 115200 baud, crtscts=1
bt selftest: before power off exp in=90,.. out=90,.. pol=.. cfg=6f,..   (+ second line)
bt selftest: after power off exp in=10,.. out=10,.. pol=.. cfg=6f,..    (+ second line)
samsung-bcm43xx-bt bluetooth: power off from ON (bootloader) (BT_REG_ON=0, BT_WAKE=0)
   (500 ms)
bt selftest: before power on exp in=10,.. out=10,.. cfg=6f,..           (+ second line)
bt selftest: after power on exp in=90,.. out=90,.. cfg=6f,..            (+ second line)
samsung-bcm43xx-bt bluetooth: power on (BT_WAKE=1, BT_REG_ON=1)
bt selftest: power on (bt_wake=1 reg_on_level=1)
bt selftest: hci reset ok in N ms (try 1)
bt selftest: local version hci=0x.. rev=0x.... lmp=0x.. lmp_subver=0x.... manuf=0x000f bdaddr_oui=xx:xx:xx
bt selftest: before power off exp ...     (2 lines)
bt selftest: after power off exp ...      (2 lines; out[0] bit 7 = 0, in[0] bit 7 = 0)
samsung-bcm43xx-bt bluetooth: power off (BT_REG_ON=0, BT_WAKE=0), host_wake_edges=N
bt selftest: power off, host_wake_edges=N, bt_wake=0, tty closed, rfkill soft_blocked=1
bt selftest: RESULT PASS, state OFF
```

No `exp CHANGED`, `UNSAFE` or `P0_7 driven` line may appear anywhere. P0_4 (`out[0]` bit 4, also
driven high by the bootloader) and every other pin must stay as at probe. (`RESULT FAIL
stage=uart-3m` after a successful HCI exchange means HCI works, only 3 Mbaud is out of range.) A
BCM4361 should report manufacturer 0x000f (Broadcom) and most likely HCI 5.0 (`hci=0x09`, not
verified); the LMP subversion identifies the ROM (record it).

Failure signatures:
- `hci 0x0c03: no command complete after 1000 ms, got 0 bytes [] ...` three times, `stage=hci`:
  chip silent. Check `reg_on_level=1` (P0_7 really high), `umstat` bit 0 (CTS from the chip; 0 =
  chip holds our TX with flow control), BT_WAKE, the 100 ms delay.
- `got N bytes [..]` that are not `04 0e 04 01 03 0c 00`: baud/clock mismatch (compare the uart
  clock line) or framing; `flagged=` counts parity/frame errors.
- `hci 0x0c03: status 0x..`: chip answered with an error.
- `expander UNSAFE (...)`, `power on|off ... failed: ..., BT_REG_ON state X, expander unsafe`,
  `stage=power`: an expander check, write or readback failed; nothing more was written. State
  `UNKNOWN` means P0_7 may still be high (BT_WAKE then stays high too); the 120 s restart reboots
  the SoC and the expander keeps whatever state it has. Stop and report.
- `expander: write reg 0x04 ..->.. failed: <err>` without `UNSAFE`: one tolerated transfer error,
  state `UNKNOWN`; the cleanup power-off is the recovery (validated).
- `host-wake storm: irq disabled after 10000 edges`: gpa2-3 toggling.

### B1-hal (ROM-in-RAM + adb, Wi-Fi present, real vendor HAL)

`rom-ram/pack-romram-bt-b1-hal-r1`, READY -> PROBE -> START, adb as in the Wi-Fi trial (wifi-adb2).
Kernel: the B1-read probe lines plus the Wi-Fi lines; at 40 s `bt selftest: ... RESULT
PASS-NOPOWER` with `rfkill rfkill0 ...` (Wi-Fi's phy rfkill must be rfkill1); at 150 s the Wi-Fi
self-test as before (it skips if userspace owns wlan0). If the HAL already unblocked BT before 40 s:
`RESULT SKIP userspace owns BT (rfkill unblocked)` (fine). Before the HAL touches BT the state is
`ON (bootloader)` (`cat /sys/devices/platform/bluetooth/bt_reg_on_state`); the first enable is a
power cycle (`power off from ON (bootloader)`, 500 ms, `power on`).

```
adb shell 'for r in /sys/class/rfkill/rfkill*; do echo $r $(cat $r/name $r/type $r/state); done'
#   rfkill0 = "bcm43xx Bluetooth" bluetooth 0 ; rfkill1 = phy0 wlan
adb shell ls -lZ /dev/ttySAC1 /sys/devices/platform/bluetooth/rfkill/rfkill0/state
#   bluetooth bluetooth, u:object_r:bt_device / sysfs_rfkill_writable
adb shell cmd bluetooth_manager enable ; adb shell cmd bluetooth_manager wait-for-state:STATE_ON
#   fallback: adb shell svc bluetooth enable
adb shell dumpsys bluetooth_manager | grep -iE "state|address|name"
adb logcat -d | grep -E "bt_hwcfg|bt_userial|bt_upio|bt_vendor|bluetooth@1.0"
adb shell dmesg | grep -E "bcm43xx|bt selftest|avc.*(bluetooth|bt_device|rfkill)"
adb shell cat /sys/devices/platform/bluetooth/bt_reg_on_state
#   ON (bootloader) before the first enable, ON while enabled, OFF after disable
```

Expected kernel on enable: `samsung-bcm43xx-bt bluetooth: power on (BT_WAKE=1, BT_REG_ON=1)`;
on disable: `power off (BT_REG_ON=0, BT_WAKE=0), host_wake_edges=N` (N > 0 is normal once HCI
traffic ran). Expected logcat (libbt strings): `upio_set_bluetooth_power(on: 1)`,
`userial vendor open: opening /dev/ttySAC1`, `bt vendor lib: set UART baud 115200`,
`Target name = [BCM4361B2]`, `FW patchfile: /vendor/firmware/bcm4361B2_semco.hcd` (or murata),
`bt vendor lib baud_2: set UART baud 3000000`, `vendor lib fwcfg completed`, HAL
`lpm_timeout_ms 1500`; `STATE_ON`. Then Wi-Fi on + `cmd wifi start-scan` with BT on, Wi-Fi off/on:
BT stays ON. Disable BT: rfkill0 state 0, kernel `power off`.

Failure signatures: `Fail to find right firmware` (no `.cid.info`, module query failed);
HCI timeouts after `baud_2` (3 Mbaud clock, B1-read `err3000000`); state stuck at
`STATE_BLE_TURNING_ON` (libbt cannot open the tty or write rfkill: permissions, avc lines; SELinux
is permissive in ROM-RAM, so check `ls -Z`); `expander UNSAFE` / `power on failed` in dmesg (read
the probe lines and the `exp CHANGED` lines); `rfkill unblock rejected: suspend/shutdown in
progress` (HAL raced a suspend).

## 5. Deviations from 4.9 and from the design draft

| Item | 4.9 / design draft | B1 | Why |
|---|---|---|---|
| Expander driver | 4.9: pcal6524 driver, reset pulse + full re-init at boot; draft: mainline pca953x | none; pin-7-only accessor inside the BT driver | pca953x writes polarity at probe and every register at resume; rules: touch P0_7 only |
| BT_REG_ON at probe | driven low | not written; driven high by the bootloader -> state `ON (bootloader)`; first rfkill block/unblock or the B1-hci self-test drives it low (unblock = power cycle, 500 ms low) | read-only probe; 4.9 parity from the first transition on |
| Expander checks | none (4.9 trusted its own driver) | full validation before/after every write, unsafe latch, no rollback | shared expander (NFC/WPC/DP) |
| Suspend/shutdown | no hooks | quiesce only (self-test cancelled/joined, transitions rejected), power state unchanged | Sol R1 M7 |
| `bt-en` | gpiod on the expander | `samsung,bt-reg-on-expander` phandle, pin fixed to P0_7 in code | no GPIO chip on the expander |
| I2C speed | draft 100 kHz | 400 kHz | 4.9 fast-mode default on this bus (no clock-frequency, default-clk 200 MHz) |
| Host-wake IRQ | 4.9 rising/falling flip, wake source; draft `interrupts` in DT | `gpiod_to_irq`, both edges, counting only, disabled while off | B1 scope; B2 adds wakeup |
| BT_WAKE | LPM TX hook + 1 s timer | high while powered | B2 |
| idle-IP / SICD | `exynos_update_ip_idle_status` | none | no SICD on 6.12 |
| UART SPI flags | 0 | level-high | 6.12 GIC needs a type |
| USI10_I2C / UART_BT SW_CONF | never written | never written (`samsung,no-sw-conf`) | 4.9 parity |

## 6. Open points

- Resolved by B1-read (section 7): USI10_I2C works without SW_CONF; the expander is out of reset
  (gpg3-6 fn=1 level=1) and answers at 0x22; the PCAL6524 extension registers answer; UART_BT
  clock 200 MHz without `clk_set_rate` (115200 exact, 3 Mbaud +1.01 %); rfkill0 path correct
  (without Wi-Fi rfkill in that kernel config path).
- rfkill index with Wi-Fi present (B1-hal 40 s self-test, sysfs listing).
- Idle current: the chip stays powered (`ON (bootloader)`) until the first rfkill block/unblock;
  with BT disabled in Android nothing may ever block it. 4.9 drove it low at probe. Decide in B2
  (e.g. a validated power-off once userspace is up, or at probe after B1-hci proves the write path).
- Idle current with BT_WAKE held high while `ON` (B2 removes it).
- Whether the bootloader state of P0_7/P0_4 is the same on every boot (compare probe dumps across
  runs).

## 7. Hardware evidence: B1-read, 2026-11-07 00:11

Pack 218efc06 (first-round driver, crownlte-bt d3a3b71c2 Image 9aa971c0, delayed-UFS vehicle).
No panic; the phone returned to 4.9 and RECOVERY was restored. Serial lines:

```
exynos-usi 108400c0.usi: USIv2 configured: sw_conf=none con=00000000 option=00000003
exynos-usi 109300c0.usi: USIv2 configured: sw_conf=none con=00000000 option=00000001
10840000.serial: ttySAC1 at MMIO 0x10840000 (irq = 73, base_baud = 0) is a S3C6400/10
samsung-bcm43xx-bt bluetooth: probe exp in=90,08,20 out=90,08,00 pol=00,00,00 cfg=6f,f5,ff
samsung-bcm43xx-bt bluetooth: probe exp drv=00,00,00,00,00,00 latch=00,00,00 pull_en=00,00,00 pull_sel=00,00,00 int_mask=ff,ff,ff int_stat=00,00,00 out_cfg=00 int_edge=00,00,00,00,00,00 in_stat=90,08,20 out_indcfg=00,00,00 debounce=00,00,00
samsung-bcm43xx-bt bluetooth: BT_REG_ON (P0_7) is already driven high at probe: BT power disabled
samsung-bcm43xx-bt bluetooth: rfkill "bcm43xx Bluetooth" = rfkill0 (blocked), host-wake irq 82, expander 0-0022 P0_7 UNUSABLE
bt selftest: bus sysreg_peric1 usi10=0x00000000 usi10_i2c=0x00000004 exp_reset gpg3-6 fn=1 level=1 pud=0 (gpg3 con=01000000 dat=43)
bt selftest: bus expander 0-0022 addr 0x22 driver=none probe_baseline=ok tainted=1
bt selftest: now exp ... (identical to probe at 93 s)
bt selftest: uart 10840000.serial driver=samsung-uart tty=MISSING
bt selftest: uart usi con=00000000 option=00000003
bt selftest: uart regs ulcon=00000000 ucon=000003c5 ufcon=00000111 umcon=00000000 ufstat=00000000 umstat=00000000 ubrdiv=00000000 ufracval=00000000
bt selftest: uart clk=200000000 Hz err115200=+0.00% err3000000=+1.01%
bt selftest: rfkill rfkill0 /sys/devices/platform/bluetooth/rfkill/rfkill0 type=bluetooth soft_blocked=1
bt selftest: RESULT FAIL stage=bus
```

Reading:
- The bootloader leaves P0_7 (BT_REG_ON) and P0_4 (DP switch) configured as outputs and driven
  high (`cfg[0]=6f`: bits 7 and 4 clear; `out[0]=90`; `in[0]=90`). 4.9 drives BT_REG_ON low at
  probe (its DT `bt` node: `gpios = <76 7 0 ...>`, expander pin 7). The first-round "already
  high" taint was too strict and caused `stage=bus`; fix round 1 records `ON (bootloader)`.
- The expander and all its extension registers answer; nothing changed between probe and 93 s.
- `tty=MISSING` was a lookup bug: ttySAC1 registered (irq 73), but on 6.12 its class device is a
  child of the serial_base port device (`10840000.serial:0.0`), not of the platform device.
- The USI wrapper readbacks (UART_BT 0x108400c4/c8, USI10_I2C 0x109300c4/c8) and the SYSREG
  PERIC1 reads (+0x3c = 0x00000000, +0x40 = 0x00000004) completed without an abort with the
  clocks as they were. The wrapper readbacks stay (with this evidence in comments); the SYSREG
  reads were removed from the self-test (values known).
- UART_BT: baud clock 200 MHz (no `clk_set_rate`), 115200 exact, 3 Mbaud +1.01 %; UCON 0x3c5 and
  UFCON 0x111 are the reset/probe values before any open; USI_OPTION 3 (4.9 parity unknown; the
  first doc guessed 2).

## 8. Fix log

### Round 1 (2026-11-07): B1-read hardware result + Sol review R1 (CHANGES)

| Item | Problem | Fix |
|---|---|---|
| HW (a) | P0_7 driven high by the bootloader tainted the expander; power impossible | probe records `ON (bootloader)` after the sane-state checks (push-pull, polarity, level = output), writes nothing; rfkill block and the self-test drive it low with full validation; unblock from there is a power cycle (500 ms low); P0_4 never touched |
| HW (b) | `tty=MISSING` | ttySAC1 found by dev_t in the tty class; the platform device must be an ancestor (else `NOT-THIS-UART`) |
| HW (c) / B3 | SYSREG PERIC1 reads in the self-test | removed |
| B4 | USI wrapper readbacks unproven | proven readable (section 7); kept with the evidence in comments (`exynos-usi.c`, self-test); only USI_CON/USI_OPTION are read |
| B1 | dump differences not checked before power on/off; rfkill transitions checked only out0/cfg0 | `pcal_validate()` (every register, baseline + tracked expected port 0 + P0_7 level) before and after every P0_7 write on every path (rfkill on/off, power cycle, self-test, cleanup); any mismatch latches unsafe and aborts before writing |
| B2 | failed safety read did not latch; power-on failure did a rollback write | failed mandatory reads latch unsafe; no rollback writes at all; unsafe also blocks power-off |
| M5 | `tty_kclose()` before restoring client ops (HAL open race, NULL deref) | release order: reception off, TX flush, close, flip buffers dropped, flip worker joined (`flush_work(&port->buf.work)` = `tty_buffer_flush_work()`), ops restored under the kopen, `tty_kclose()` last; same path on open failure |
| M6 | partial power-on could leave the chip powered while software thinks off | state OFF/ON/ON (bootloader)/UNKNOWN; UNKNOWN recorded before the first write, replaced only by a validated result; BT_WAKE high while power may be asserted; logged in every failure line and in sysfs `bt_reg_on_state`; recovery = rfkill block -> validated power-off (one tolerated write transfer error per boot; unsafe latch otherwise) |
| M7 | delayed self-test on `system_long_wq` not quiesced | `prepare`/`complete` and `shutdown`: cancel or abort+join the self-test, wait out an rfkill transition, reject transitions with `-EBUSY` while quiescing |
