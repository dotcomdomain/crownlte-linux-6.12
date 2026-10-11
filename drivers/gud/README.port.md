# Exynos9810 Trustonic Linux 6.12 port

The driver comes from the phone owner's modified crownlte 4.9 source at
8665533db42ea7d50a4e22796c8331844ff5e596. The eight public headers remain
byte-identical. An AArch64 compilation comparison verifies all 22 ioctl values
against that source. The secure-world MCI protocol and original opaque SWd
CPU identifiers are preserved.

The port adapts VMA flags, page allocation order limits, class creation,
device-tree includes, fallthrough markers, shash API, timekeeping, kernel
workers, access checks, and the debug-only freezer state query. Secure-world
user mappings now use long-term page pins with matching unpin and dirty-page
accounting. Remote pinning handles the mmap lock being dropped for faults.
DMA buffer mapping uses the 6.12 unlocked attachment wrappers.

The optional secure-OS CPU booster uses a real cpufreq policy and freq_qos
minimum-frequency request, with policy lifetime and initialization failure
cleanup. It requires an operating CPU frequency driver before its initcall.
It currently fails initialization if no policy exists. Integration must
provide the real Exynos cpufreq driver and verify init ordering or add deferred
initialization. A successful link does not establish actual frequency boosting.

Verification: LLVM 18 AArch64 Image and dtbs link completed with exit 0 using
the existing crypto diagnostic configuration plus TRUSTONIC_TEE,
TRUSTONIC_TEE_LPAE, SECURE_OS_CONTROL, SECURE_OS_BOOSTER_API, CPU_FREQ,
CRYPTO_SHA1 and DEBUG_FS. The 22 ioctl encoding blob SHA256 is
25a5ff285e102e1f1fa9b970fcf9781672c669ff0955b63cc9b345603a4c6e2d.

Hardware validation is outstanding. Integration needs the original board's
Trustonic DT node, secure IRQ, reserved/shared-memory settings, and secure
firmware handoff. Keymaster, secure sessions, encryption-key release, pinned
buffer lifetime and CPU boosting have not been exercised on the phone. This
tree is source-port evidence and is not a flash-ready Android kernel.
