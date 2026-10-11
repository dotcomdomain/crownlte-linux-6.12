# Original custom behavior on Android16 Linux6.12

The six tracked modifications are retained in the urgent brightness4.9 build.
The new kernel port reconciles them individually in the isolated
`/root/kernel-upgrade/custom-behavior-port` worktree.

- `ab3dd28e`, HID configfs: enable USB_CONFIGFS_F_HID and its modern dependencies.
- `223693fa`, Windows Precision Touchpad: recognize the same descriptor marker
  and answer feature reports2,3,5 with the original capability/mode values.
  Zero-filled certification and unknown feature reports retain their requested
  length up to EP0 capacity. Feature SET_REPORT is advisory. Modern non-feature
  GET_REPORT userspace handling remains available. Tests of the production
  helpers pass2820 descriptor/report/length/guard-byte cases. Actual Windows
  enumeration is outstanding.
- `35415538`, Droidspaces: restore legacy noprefix cgroup prefixed aliases,
  including paired removal and error handling. The configuration audit maps
  the original60 entries to actual ACK symbols or features now built into
  NF_TABLES/NF_NAT. The actual ACK tree still supports CGROUP_NET_PRIO.
  Firmware fallback and DEVTMPFS_MOUNT=n are preserved. A validated105-option
  fragment also enables modern legacy cpuset and memcg controllers. Object
  compilation passed for the actual HID, cgroup and DWC3 objects with LLVM18.
  All105 fragment requests survive the merged diagnostic configuration.
  Actual container startup remains untested.
- `faccc30f`, IPv6 device teardown: the original core called each route's
  ifdown handler before checking whether dst->dev matched the removed device.
  The modern dst_dev_put path passes that route's own dst->dev directly, and
  its ifdown handler uses the global blackhole device rather than a namespace
  loopback replacement. The original broad notifier structure is gone.
  Source inspection indicates the old ownership bug is addressed by this
  narrower caller. Veth teardown and unaffected IPv6 route tests remain.
- `c34ea358`, accepted USB request ownership: modern bulk/interrupt queueing
  adds requests to pending_list and returns0 after the transfer kick. Modern
  zero-length transfers are represented in TRBs instead of the old separate
  ZLP request allocation. Accepted isochronous requests now also return0 after
  start attempts. Failed-start cancellation defers giveback to a per-endpoint
  tasklet when no END_TRANSFER or delayed stop is pending. Its initialization
  and teardown follow endpoint lifetime. Completion no longer runs from the
  transfer kick's queue stack, following the documented gadget contract.
  See the [kernel USB gadget contract](https://www.kernel.org/doc/html/latest/driver-api/usb/gadget.html).
- `8665533d`, disconnect handling: the modern queue path already rejects a
  disconnected device, and modern gether_disconnect clears port_usb before
  disabling endpoints. The isolated patch additionally rejects late IRQ
  transfer kicks when the endpoint/device is disconnected. RX pool ownership
  captures endpoint and link generation under the link/pool locks. Submission
  checks both before allocation and again under the link lock when queueing
  with GFP_ATOMIC. Requests held privately across teardown are freed through
  their original endpoint instead of returning to a drained pool. Completion
  carries its entry generation through resubmission and reclamation. Tests of
  the actual queue and RX submission functions pass3245 fault and flag cases
  under ASAN/UBSAN. Source review covers tasklet, TRB and endpoint lifetime;
  hardware disconnect/IRQ stress testing remains outstanding.

No6.12 custom behavior has been exercised on the phone or flashed. Compilation
and helper tests do not establish USB, IPv6 or Droidspaces runtime behavior.
