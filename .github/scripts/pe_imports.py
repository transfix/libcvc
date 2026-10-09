"""Print the DLLs a Windows PE file imports, one per line (regular and delay-load).

    python pe_imports.py FILE.dll

Used by sdk-check-closure.sh on Windows, where the runner has no objdump or
readelf and dumpbin needs a Visual Studio developer shell.
"""
import struct
import sys


def imports(path):
    with open(path, "rb") as f:
        b = f.read()
    pe = struct.unpack_from("<I", b, 0x3C)[0]
    if b[pe:pe + 4] != b"PE\0\0":
        raise SystemExit(f"{path}: not a PE file")
    nsec, opt_size = struct.unpack_from("<H12xH", b, pe + 6)
    opt = pe + 24
    magic = struct.unpack_from("<H", b, opt)[0]
    dirs = opt + (112 if magic == 0x20B else 96)  # PE32+ / PE32
    sections = []
    for i in range(nsec):
        s = opt + opt_size + 40 * i
        vsize, va, rawsize, raw = struct.unpack_from("<IIII", b, s + 8)
        sections.append((va, max(vsize, rawsize), raw))

    def off(rva):
        for va, size, raw in sections:
            if va <= rva < va + size:
                return rva - va + raw
        raise ValueError(f"{path}: RVA {rva:#x} outside every section")

    def cstr(rva):
        o = off(rva)
        return b[o:b.index(b"\0", o)].decode("ascii")

    names = []
    # Data directory 1: imports, 20-byte descriptors, name RVA at +12.
    # Data directory 13: delay-load imports, 32-byte descriptors, name RVA at +4.
    for index, size, name_at in ((1, 20, 12), (13, 32, 4)):
        rva = struct.unpack_from("<I", b, dirs + 8 * index)[0]
        if not rva:
            continue
        d = off(rva)
        while any(b[d:d + size]):
            names.append(cstr(struct.unpack_from("<I", b, d + name_at)[0]))
            d += size
    return names


if __name__ == "__main__":
    for name in imports(sys.argv[1]):
        print(name)
