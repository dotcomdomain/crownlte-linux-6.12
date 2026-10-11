"""Check generated ARM64 header/DT against captured Note9 essentials."""
from pathlib import Path
import struct
import sys

out = Path(sys.argv[1])
image = (out / 'arch/arm64/boot/Image').read_bytes()
assert image[56:60] == b'ARM\x64'
offset, size, flags = struct.unpack_from('<QQQ', image, 8)
# image_size includes zero-filled BSS, beyond the bytes in the Image file.
assert size >= len(image), (size, len(image))
assert flags & 1 == 0 and flags & 6 == 2, hex(flags)
assert 0x92000000 % 0x200000 == 0
assert 0x92000000 + size < 0xaf600000
print(f'ARM64 Image: file_size={len(image)}, effective_size={size}, text_offset={offset:#x}, flags={flags:#x}, little-endian/4K')

def fdt_props(data):
    header = struct.unpack_from('>10I', data)
    magic, total, so, no = header[:4]
    assert magic == 0xd00dfeed and total == len(data)
    pos, names, stack, props = so, data[no:no + header[8]], [], {}
    while pos < so + header[9]:
        token = struct.unpack_from('>I', data, pos)[0]
        pos += 4
        if token == 1:
            end = data.index(0, pos)
            stack.append(data[pos:end].decode())
            pos = (end + 4) & ~3
        elif token == 2:
            stack.pop()
        elif token == 3:
            length, nameoff = struct.unpack_from('>II', data, pos)
            pos += 8
            end = names.index(0, nameoff)
            path = '/'.join(stack) or '/'
            props[(path, names[nameoff:end].decode())] = data[pos:pos + length]
            pos = (pos + length + 3) & ~3
        elif token == 4:
            pass
        elif token == 9:
            assert not stack
            return props
        else:
            raise AssertionError(f'Bad FDT token {token}')
    raise AssertionError('FDT has no end token')

for name in ['exynos9810-crownlte-diag.dtb', 'exynos9810-crownlte-usb-diag.dtb']:
    path = out / 'arch/arm64/boot/dts/exynos' / name
    if not path.exists():
        continue
    p = fdt_props(path.read_bytes())
    assert p[('/', 'model_info-system_rev')] == b'028\0'
    assert struct.unpack('>I', p[('/', 'model_info-hw_rev')])[0] == 26
    assert p[('/soc@0/ufs@11120000', 'status')] == b'disabled\0'
    memory = struct.unpack('>15I', p[('/memory@80000000', 'reg')])
    assert memory == (0, 0x80000000, 0x3c800000,
                      0, 0xc0000000, 0x20000000,
                      0, 0xe1900000, 0x1e700000,
                      8, 0x80000000, 0x80000000,
                      9, 0, 0x80000000)
    for logical, affinity in enumerate([0, 1, 2, 3, 0x100, 0x101, 0x102, 0x103]):
        value = struct.unpack('>I', p[(f'/cpus/cpu@{affinity:x}', 'reg')])[0]
        assert value == affinity, (logical, value)
    assert struct.unpack('>3I', p[('/reserved-memory/ramoops@fed10000', 'reg')]) == (0, 0xfed10000, 0x8000)
    expected = b'okay\0' if '-usb-' in name else b'disabled\0'
    assert p[('/soc@0/usb@10c00000', 'status')] == expected
    assert p[('/soc@0/phy@11100000', 'status')] == expected
    if '-usb-' in name:
        assert p[('/soc@0/i2c@14360000', 'compatible')] == b'samsung,exynos9810-hsi2c\0'
        assert p[('/soc@0/i2c@14360000/muic@25', 'compatible')] == b'samsung,exynos9810-max77705-muic\0'
    print(f'{name}: exact RAM/CPU/pstore, UFS disabled, USB status={expected.decode().rstrip(chr(0))}')
