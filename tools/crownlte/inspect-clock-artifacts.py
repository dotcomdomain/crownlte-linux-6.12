"""Validate clock output against captured Note9 RAM, clock and firmware resources."""
from pathlib import Path
import runpy
import struct
import sys
base=Path(__file__).resolve().parent
namespace=runpy.run_path(str(base/'inspect-artifacts.py'))
out=Path(sys.argv[1])
for name in ['exynos9810-crownlte-diag.dtb','exynos9810-crownlte-usb-diag.dtb']:
    props=namespace['fdt_props']((out/'arch/arm64/boot/dts/exynos'/name).read_bytes())
    for node in ['acpm-pmic-client','cmu-ewf@1a240000','clock-controller@15a80000','acpm@14020000','acpm-ipc@14100000','acpm-dvfs']:
        assert props[('/'+node,'status')]==b'disabled\0',node
    assert struct.unpack('>3I',props[('/clock-controller@15a80000','reg')])==(0,0x15a80000,0x8000)
    assert struct.unpack('>6I',props[('/acpm-ipc@14100000','reg')])==(0,0x14100000,0x1000,0,0x2039000,0x30000)
    assert struct.unpack('>I',props[('/acpm-dvfs','acpm-ipc-channel')])[0]==5
    assert struct.unpack('>3I',props[('/reserved-memory/ect-table@a0000000','reg')])==(0,0xa0000000,0x32000)
    assert struct.unpack('>I',props[('/ect','parameter_address')])[0]==0xa0000000
    assert struct.unpack('>I',props[('/ect','parameter_size')])[0]==0x32000
    print(name+': original CMU and firmware resources verified; all supplier nodes disabled')
