# crownlte Bluetooth on 6.12: stage B1 (BCM4361 BT over UART, rfkill)

Status: build-only, not run on the phone. Branch `crownlte-bt` on `crownlte-wifi` (15cebae03,
Wi-Fi working on hardware), so Wi-Fi and BT are in one kernel. Design and evidence:
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

Hardware path (from 4.9, see the design): HCI on `uart@10840000` = ttySAC1 (SPI 422, gpq0-0..3
RXD/TXD/RTS/CTS function 2 pull-up, 256-byte FIFO); BT_REG_ON = PCAL6524 P0_7 at 0x22 on
`hsi2c@10930000` (USI10_I2C, SPI 437, gpp6-2/3 function 3); BT_WAKE = gpg2-0 (output, high =
assert); BT_HOST_WAKE = gpa2-3 (EINT, high = chip has data).

### Driver behaviour (`samsung,bcm43xx`, platform device `bluetooth`)

- rfkill `bcm43xx Bluetooth`, type bluetooth, created with `rfkill_init_sw_state(true)`: it starts
  blocked and `rfkill_register()` calls no `set_block()` (persistent). `CONFIG_RFKILL_INPUT` is off
  because its `__rfkill_switch_all()` at registration would call `set_block()`. Path:
  `/sys/devices/platform/bluetooth/rfkill/rfkill0` (vendor rc chowns rfkill0, vendor sepolicy labels
  exactly this path).
- Unblock: BT_WAKE=1, P0_7 output bit 1, P0_7 direction output, 100 ms (4.9), host-wake edge
  counting on. Block: counting off, P0_7 output bit 0 (stays an output, as 4.9), BT_WAKE=0.
  `set_block` errors are returned to the rfkill core, which then keeps the old state.
- BT_WAKE is held high for the whole powered time (libbt's LPM hooks are no-ops; LPM is B2).
- BT_HOST_WAKE: both-edge IRQ, requested disabled (`IRQF_NO_AUTOEN`), enabled only while powered,
  counts edges, disabled after 10000 edges in one session (`host-wake storm`). Not a wakeup
  source. If the IRQ cannot be obtained the driver logs it and still registers the rfkill.
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
  BT_REG_ON low at probe right after its own full expander init; 6.12 does neither.
- Every power change: read output port 0 (0x04) and configuration port 0 (0x0c); bits 0-6 must
  equal the probe baseline; write the byte with only bit 7 changed; read it back. On power-on the
  output bit is set before the direction bit (4.9 order).
- "Taint" (no further power-on writes, `power on refused`): bus error or NACK at probe, all registers
  reading one constant value (stuck bus), a driver bound, BT_REG_ON already driven high at probe,
  port 0 bits 0-6 differing from the baseline, or a failed write/readback. Power-off (bit 7 to 0)
  stays allowed as long as port 0 still matches the baseline.
- No `reset-gpios`; gpg3-6 (expander RESET#, active low) is only read, by the self-test.

Registers dumped (2 log lines): `in` 0x00-02, `out` 0x04-06, `pol` 0x08-0a, `cfg` 0x0c-0e, `drv`
0x40-45, `latch` 0x48-4a, `pull_en` 0x4c-4e, `pull_sel` 0x50-52, `int_mask` 0x54-56, `int_stat`
0x58-5a, `out_cfg` 0x5c, and the PCAL6524 extensions 4.9 never read: `int_edge` 0x60-65, `in_stat`
0x6c-6e, `out_indcfg` 0x70-72, `debounce` 0x74-76 (printed as `--` if they do not answer; that is
reported, not a failure). Compared with the baseline: all but `in`, `int_stat`, `in_stat`, and with
bit 7 of `out[0]`/`cfg[0]` masked. Reading the input ports clears a pending expander interrupt;
nothing on 6.12 uses that interrupt.

### Self-test (`bt selftest:`)

Armed only by `samsung,bt-selftest` (+ `samsung,bt-selftest-ms`, time since boot). Runs once on
`system_long_wq` under the driver mutex. Skips (`RESULT SKIP userspace owns BT ...`) if the rfkill is
unblocked or ttySAC1 is open.

1. `bus`: PERIC1 SYSREG +0x3c / +0x40 (USI10 / USI10_I2C SW_CONF slots) and GPG3 CON/DAT/PUD
   (expander reset gpg3-6), read-only; expander client, bound driver, baseline; probe dump and a
   fresh dump, compared. FAIL on no baseline, taint, a bound driver or any difference.
2. `uart`: the serial device has a driver and a `ttySAC1` child; with the UART clocks enabled:
   USI_CON/USI_OPTION and ULCON/UCON/UFCON/UMCON/UFSTAT/UMSTAT/UBRDIV/UFRACVAL; `clk_get_rate` of
   `clk_uart_baud0` and the baud error samsung_tty will produce at 115200 and 3000000 (div =
   rate/baud, UBRDIV = div/16-1, UFRACVAL = div&15). FAIL (`uart`) if the tty or a register window
   is missing or the 115200 error exceeds 2 %; `uart-3m` (reported at the end) if the clock is
   below 48 MHz or the 3 Mbaud error exceeds 2 %.
3. `rfkill`: our rfkill device must be `rfkill0` with sysfs path
   `/devices/platform/bluetooth/rfkill/rfkill0`.
4. With `samsung,bt-selftest-no-power`: stop, `RESULT PASS-NOPOWER`.
5. Open ttySAC1 in the kernel (`tty_kopen_exclusive`, then the serdev-ttyport termios: raw 8N1,
   CRTSCTS, CLOCAL, 115200; receive through temporary `tty_port` client ops), dump, power on, dump,
   HCI Reset `01 03 0c 00` (1 s, 3 tries 100 ms apart), Read Local Version `01 01 10 00`, Read
   BD_ADDR `01 09 10 00` (only the OUI is printed). No patchram download.
6. Power off, dump, flush and close the tty, restore the default client ops.
   `RESULT PASS` or `RESULT FAIL stage=<bus|uart|rfkill|power|hci|uart-3m>`.

## 2. Config and build

`.config` = `panel-bright-work/ko-wifi-s1/config` + `crownlte_bt.fragment` (`merge_config.sh -m`,
`olddefconfig`). The resulting difference to the Wi-Fi config is exactly `CONFIG_RFKILL=y` and
`CONFIG_SAMSUNG_BCM43XX_BT=y` (RFKILL_INPUT, RFKILL_GPIO, GPIO_PCA953X, BT, SERIAL_DEV_BUS off).
`make ARCH=arm64 LLVM=-18 Image dtbs`, clang 18.1.3; the new and changed files build with W=1
without warnings. Kernel footprint `_end - _text` = 0x18c0000 (Wi-Fi build 0x18b0000).

With `CONFIG_RFKILL=y`, cfg80211 also registers a wiphy rfkill (type wlan) when DHD attaches at
`late_initcall_sync`. The BT driver probes at the latest in the deferred-probe flush at
`late_initcall` (it defers until the I2C client exists), so it should be `rfkill0` and Wi-Fi
`rfkill1`. B1-hal checks this with Wi-Fi present.

## 3. Artifacts

gekko `/mnt/ssd/host-builds/kernel-upgrade/`, built from crownlte-bt d3a3b71c2 (this
documentation commit changes no built file):

| file | sha256 |
|---|---|
| `panel-bright-work/images/Image-bt-b1-d3a3b71c2` (25,053,192 B) | 9aa971c0c96be6124429c4366c75b53db904423b0e8a5cd09fa824c5fb9007c1 |
| `panel-bright-work/images/config-bt-b1-d3a3b71c2` | e94d322ef72eac75fc9507959106a001d47a35e199b5b42ecbe1bde77b5c30bc |
| `panel-bright-work/dtb/exynos9810-crownlte-delayed-ufs-bt-diag.dtb` (B1-read) | 786eedbb9dfcc3ce6d9947e705ff7858e16e9a6d34e3555e0cf88c9baad2b8fa |
| `panel-bright-work/dtb/exynos9810-crownlte-delayed-ufs-bt-hci-diag.dtb` (B1-hci) | 0be846a462f1403322e95f43521ef2478a96a978a2b3b22eaf6dd5efa5948027 |
| `panel-bright-work/dtb/exynos9810-crownlte-romram-bt-b1.dtb` (B1-hal) | adedcd7dff302ccd7f14400e061fcf8d71a3289b5b8e4f58b500edb124f559c5 |

`panel-bright-work/ko-bt-b1/` holds hard links in the `KO=` layout (Image, config,
`arch/arm64/boot/dts/exynos/*.dtb`, SHA256SUMS). The ROM-RAM DTS, its diff against
`romram-wifi-s1` and the generator are in `panel-bright-work/dtb/`
(`exynos9810-crownlte-romram-bt-b1.dts{,.diff}`, `romram-bt-b1-gen.py`); it is the Wi-Fi-working
`romram-wifi-s1` DTS (Wi-Fi self-test at 150 s unchanged) plus exactly the BT nodes, appended as
`&{/path}` fragments, and the no-power BT self-test at 40 s. `dtc` rebuilds the wifi-s1 DTB from
its DTS bit-exact, so the method is the one used for that DTB.

Packed trials (all with shim bbea9590, parent RECOVERY a8198be0, restore RECOVERY 8763860c):

| trial | directory | DTB | ramdisk | recovery.img sha256 |
|---|---|---|---|---|
| B1-read | `bt-b1-read-20261107` | delayed-ufs-bt-diag | delayed-UFS 506232b1 | 218efc06a579f391a5198f45aa298c83fd044fec9c3ca4991f9796fde8e2cc1f |
| B1-hci | `bt-b1-hci-20261107` | delayed-ufs-bt-hci-diag | delayed-UFS 506232b1 | e9dc9073ed02df09c6ebbb7fb31b6b4f2456816530bd69b01d35a9a11a30d00e |
| B1-hal | `rom-ram/pack-romram-bt-b1-hal` | romram-bt-b1 | build-30 e615cc8d (ADB, early DONE) | f458213d3638390bdf1c7d3736e9790e0510b475b95cdbdf58278fcbf5514b1b |

## 4. Trials, expected lines, failure signatures

Order: B1-read, then B1-hci only if B1-read passed and its dumps look sane, then B1-hal. All three
go into RECOVERY the usual way; afterwards save the serial log, read pstore, restore bright RECOVERY.

### B1-read (kernel only, nothing written to the expander, chip never powered)

Delayed-UFS vehicle, READY only. Self-test at 93 s (PID1 restarts at 120 s). Expected
(`N`, `X` = values to record; the probe-time lines may interleave differently, the self-test lines
come in this order):

```
exynos-usi 109300c0.usi: USIv2 configured: sw_conf=none con=00000000 option=0000000X
exynos-usi 108400c0.usi: USIv2 configured: sw_conf=none con=00000000 option=00000002
exynos5-hsi2c 10930000.i2c: 9810 timing: clock=N bus=400000 div=N SCL=N
10840000.serial: ttySAC1 at MMIO 0x10840000 (irq = N, base_baud = N) is a S3C6400/10
samsung-bcm43xx-bt bluetooth: probe exp in=..,..,.. out=..,..,.. pol=..,..,.. cfg=..,..,..
samsung-bcm43xx-bt bluetooth: probe exp drv=... latch=... pull_en=... pull_sel=... int_mask=... int_stat=... out_cfg=.. int_edge=... in_stat=... out_indcfg=... debounce=...
samsung-bcm43xx-bt bluetooth: rfkill "bcm43xx Bluetooth" = rfkill0 (blocked), host-wake irq N, expander N-0022 P0_7
samsung-bcm43xx-bt bluetooth: bt selftest: armed for 93000 ms after boot, no power step
... at 93 s:
bt selftest: start at N ms, mode no-power
bt selftest: bus sysreg_peric1 usi10=0x... usi10_i2c=0x... exp_reset gpg3-6 fn=F level=1 pud=P (gpg3 con=... dat=..)
bt selftest: bus expander N-0022 addr 0x22 driver=none probe_baseline=ok tainted=0
bt selftest: probe exp ...            (2 lines, = the probe dump)
bt selftest: now exp ...              (2 lines; no "exp CHANGED" line)
bt selftest: bus P0_7 (BT_REG_ON) at probe: input|output, out=X, level=X
bt selftest: uart 10840000.serial driver=samsung-uart tty=ttySAC1
bt selftest: uart usi con=00000000 option=00000002
bt selftest: uart regs ulcon=... ucon=... ufcon=... umcon=... ufstat=... umstat=... ubrdiv=... ufracval=...
bt selftest: uart clk=N Hz err115200=+0.xx% err3000000=+x.xx%
bt selftest: rfkill rfkill0 /sys/devices/platform/bluetooth/rfkill/rfkill0 type=bluetooth soft_blocked=1
bt selftest: RESULT PASS-NOPOWER
```

Record for the next step: the USI10 SW_CONF values, the reset line (fn/level), the full probe
dump (NFC/DP/wireless-charging pins), P0_7 state, the UART clock and both baud errors.

Failure signatures:
- `expander N-0022 unreadable (-6): BT power disabled` (NACK) + `exp_reset ... level=0`: expander held
  in reset by gpg3-6 (design risk 2). Do nothing; report. Releasing reset would return all 24 pins
  to inputs (NFC VEN, wpc_en, DP switch): owner decision.
- `unreadable (-110)` or `-5` / i2c timeouts without NACK, or `exynos5-hsi2c ... 9810 timing` missing
  or `-EINVAL`: bus / clock problem (USI10_I2C SW_CONF, design risk 1; compare `usi10_i2c=`).
- `deferred probe pending: ... expander i2c client not yet registered`: the I2C adapter never came up
  (look for `USIv2 configuration readback failed` on 109300c0 or hsi2c probe errors).
- `exp CHANGED <reg>[i] probe=.. now=..` or `port 0 changed outside P0_7`: something else is writing
  the expander or it reset between probe and 93 s: stop.
- `BT_REG_ON (P0_7) is already driven high at probe`: bootloader left BT powered; driver will not
  touch the expander. Report.
- `tty=MISSING`: no `serial1` alias / compatible / UART probe failure (`RESULT FAIL stage=uart`).
- `uart clk=0` or `err115200` above 2 %: `stage=uart`. `3000000 baud not reachable within 2%` and
  `RESULT FAIL stage=uart-3m`: B1-hci can still run (HCI at 115200), libbt's 3 Mbaud will need a
  `clk_set_rate` (design risk 6).
- `rfkill rfkill1 ...` / `RESULT FAIL stage=rfkill`: another rfkill registered first.
- `BT_HOST_WAKE irq unavailable (N)`: edge counting off; not fatal for B1.

### B1-hci (kernel only, powers the BT core once)

Same vehicle and timing. Everything above up to the rfkill line, then:

```
bt selftest: tty ttySAC1 open: 115200 baud, crtscts=1
bt selftest: before power on exp ...      (2 lines, no CHANGED)
samsung-bcm43xx-bt bluetooth: power on (BT_WAKE=1, BT_REG_ON=1)
bt selftest: after power on exp ...       (2 lines; only out[0]/cfg[0] bit 7 may differ, no CHANGED)
bt selftest: power on (bt_wake=1 reg_on_level=1)
bt selftest: hci reset ok in N ms (try 1)
bt selftest: local version hci=0x.. rev=0x.... lmp=0x.. lmp_subver=0x.... manuf=0x000f bdaddr_oui=xx:xx:xx
samsung-bcm43xx-bt bluetooth: power off (BT_REG_ON=0, BT_WAKE=0), host_wake_edges=N
bt selftest: after power off exp ...      (2 lines, no CHANGED; out[0] bit 7 = 0, cfg[0] bit 7 = 0)
bt selftest: power off, host_wake_edges=N, bt_wake=0, tty closed, rfkill soft_blocked=1
bt selftest: RESULT PASS
```

(`RESULT FAIL stage=uart-3m` after a successful HCI exchange means HCI works, only 3 Mbaud is out
of range.) A BCM4361 should report manufacturer 0x000f (Broadcom) and most likely HCI 5.0
(`hci=0x09`, not verified); the LMP subversion identifies the ROM (record it).

Failure signatures:
- `hci 0x0c03: no command complete after 1000 ms, got 0 bytes [] ...` three times, `stage=hci`:
  chip silent. Check `reg_on_level=1` (P0_7 really high), `umstat` bit 0 (CTS from the chip; 0 =
  chip holds our TX with flow control), BT_WAKE, the 100 ms delay.
- `got N bytes [..]` that are not `04 0e 04 01 03 0c 00`: baud/clock mismatch (compare the uart
  clock line) or framing; `flagged=` counts parity/frame errors.
- `hci 0x0c03: status 0x..`: chip answered with an error.
- `power on refused` / `power on failed` / `expander: reg 0x04 ...` / `stage=power`: expander write
  or readback failed, or a dump changed: stop and report.
- `power off failed ... (BT_REG_ON state unknown)`: P0_7 may still be high; report (the 120 s
  restart reboots the SoC, and the expander then keeps whatever state it has).
- `host-wake storm: irq disabled after 10000 edges`: gpa2-3 toggling.

### B1-hal (ROM-in-RAM + adb, Wi-Fi present, real vendor HAL)

`rom-ram/pack-romram-bt-b1-hal`, READY -> PROBE -> START, adb as in the Wi-Fi trial (wifi-adb2).
Kernel: the B1-read probe lines plus the Wi-Fi lines; at 40 s `bt selftest: ... RESULT
PASS-NOPOWER` with `rfkill rfkill0 ...` (Wi-Fi's phy rfkill must be rfkill1); at 150 s the Wi-Fi
self-test as before (it skips if userspace owns wlan0). If the HAL already unblocked BT before 40 s:
`RESULT SKIP userspace owns BT (rfkill unblocked)` (fine).

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
is permissive in ROM-RAM, so check `ls -Z`); `power on refused` in dmesg (expander tainted:
read the probe lines).

## 5. Deviations from 4.9 and from the design draft

| Item | 4.9 / design draft | B1 | Why |
|---|---|---|---|
| Expander driver | 4.9: pcal6524 driver, reset pulse + full re-init at boot; draft: mainline pca953x | none; pin-7-only accessor inside the BT driver | pca953x writes polarity at probe and every register at resume; rules: touch P0_7 only |
| BT_REG_ON at probe | driven low | not written; checked (driven high -> no power at all) | read-only first boot; no write before the first power-on |
| `bt-en` | gpiod on the expander | `samsung,bt-reg-on-expander` phandle, pin fixed to P0_7 in code | no GPIO chip on the expander |
| I2C speed | draft 100 kHz | 400 kHz | 4.9 fast-mode default on this bus (no clock-frequency, default-clk 200 MHz) |
| Host-wake IRQ | 4.9 rising/falling flip, wake source; draft `interrupts` in DT | `gpiod_to_irq`, both edges, counting only, disabled while off | B1 scope; B2 adds wakeup |
| BT_WAKE | LPM TX hook + 1 s timer | high while powered | B2 |
| idle-IP / SICD | `exynos_update_ip_idle_status` | none | no SICD on 6.12 |
| UART SPI flags | 0 | level-high | 6.12 GIC needs a type |
| USI10_I2C / UART_BT SW_CONF | never written | never written (`samsung,no-sw-conf`) | 4.9 parity |

## 6. Open points

- USI10_I2C works without SW_CONF and the expander is out of reset (B1-read).
- The PCAL6524 extension registers answer (else `--`, informational).
- UART_BT clock rate on 6.12 (no `clk_set_rate`, as 4.9).
- rfkill index with Wi-Fi present (B1-hal 40 s self-test, sysfs listing).
- Idle current with BT_WAKE held high (B2 removes it).
