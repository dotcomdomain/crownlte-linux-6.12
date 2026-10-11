Exynos9810 original clock and power foundation on Android16 ACK6.12.92

This branch imports the current crownlte kernel's CAL, CMUCAL, ASV, PMUCAL,
ACPM IPC and CCF hardware tables from original source commit
8665533db42ea7d50a4e22796c8331844ff5e596. It is based on ACK commit
1dd27fbde73f42a97d467891b3bfce94f4f26dc6 through main port commit 938c87358.

The upstream Samsung CCF driver stays intact. Legacy helpers use an
exynos9810_legacy_ namespace. Clock binding IDs and hardware table values
retain their original values. NULL tables now have explicit zero counts,
instead of evaluating sizeof(NULL). The original QCH ignore policy is enabled.

The real ECT RAM table at physical 0xa0000000, size 0x32000, is reserved.
The parser maps it through memremap, checks the outer header and propagates
failure into CAL initialization. ASV fuse reads use the original monitor
register protocol, including its x2 value return. Failed secure reads and
failed register mappings abort initialization.

The original GEN MINMAX_dvfs_g3d table selects 260 through 598 MHz and
boot/resume 260 MHz for ASV versions 0 through 6. The original runtime ASV and ACPM FVMAP selection remains
in use. No donor fixed voltages are substituted. CAL rate requests require a
real exact table entry inside the selected original min/max range and a
matching nonzero firmware voltage. Missing firmware tables reject transitions.
All CAL APIs used by the original Bifrost r38p1 GPEX backend retain signatures
and rate units.

ACPM commands use the original mailbox/SRAM transport and request channel 5
from the actual DVFS platform node. Commands before probe return an error.
The transport, SMC, DT resources and hardware clock operations have not been
tested on the phone. Dynamic firmware plugin attachment changes are rejected
during bring-up. Firmware FVMAP sizes and level counts must match the original
CAL rates in the same order before the table is published. Missing voltage
entries reject initialization. The real PMIC transport uses original channel 2.
exynos_acpm_mfd_is_ready() checks that supplier without a PMIC transaction.

The original low-level PMUCAL power sequences are compiled and error-returning
PMIC and secure power-domain calls remain real calls. Generic power-domain
registration, vendor idle-IP management and BTS bus-token policy are not yet
ported. CPU/devfreq thermal policy and MIF throughput QoS consumers also remain
unimplemented.
The old MIF notifier and legacy thermal notifier registration are not enabled.

The cal_if_is_initialized() API publishes readiness only after all CAL and
PMUCAL mappings and initialization stages succeed. Transport guards reject
invalid SRAM offsets, ring indices, queue sizes and oversized indirection data.
The original fixed sixteen-byte log entry format is required. Firmware timeout
returns an error. The remaining plugin parser has not had a complete bounds audit.

All CMU, ACPM, PMIC-client and early-wakeup controller nodes are disabled in
exynos9810-crownlte-clocks.dtsi. UFS stays
disabled. This branch provides compile and source parity evidence, not evidence
that the phone boots or Android vendor graphics operates.

Validation uses LLVM18, ARCH=arm64, CROSS_COMPILE=aarch64-linux-gnu-, LLVM=-18,
with two build jobs maximum. Separate clock/SoC directory builds passed.
The full Image+dtbs link passed with WERROR=y. Logs are build-clocks-image.log
and build-clocks-dt-final.log. The config and Image hashes are in the manifest.
Rebuild with KERNEL_OUT set to a separate initialized diagnostic output tree
and tools/crownlte/build-clock-foundation.sh. Validate the Image and both binary
DTBs with tools/crownlte/inspect-clock-artifacts.py followed by that output path.
Original table and binding parity is recorded in artifacts/clocks-6.12/manifest.json.

No commands from this workstream access the phone. No kernel has been flashed.
