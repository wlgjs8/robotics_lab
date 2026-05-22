#!/usr/bin/env python3
import argparse, mmap, os, struct

HEADER = struct.Struct("<IIIIQQQ")
DESC = struct.Struct("<32s16s16sIIIIIIQQ")
MAGIC = 0x43534D52
SLOT_HEADER_SIZE = struct.calcsize("<QQQQQQIIIIII")

def s(raw): return raw.split(b"\0", 1)[0].decode()

def require_range(mm, offset, size, label):
    if offset < 0 or size < 0 or offset + size > len(mm):
        raise SystemExit(f"{label} out of bounds: offset={offset} size={size} shm_size={len(mm)}")

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--shm", default="/camera_server_frames")
    args = ap.parse_args(); path = "/dev/shm/" + args.shm.lstrip("/")
    with open(path, "r+b") as f:
        mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        require_range(mm, 0, HEADER.size, "header")
        magic, version, stream_count, _, total, created, desc_off = HEADER.unpack_from(mm, 0)
        if magic != MAGIC: raise SystemExit(f"bad magic {magic:x}")
        if total <= 0 or total > len(mm): raise SystemExit(f"bad total_size_bytes {total}")
        if stream_count > 16: raise SystemExit(f"bad stream_count {stream_count}")
        require_range(mm, desc_off, stream_count * DESC.size, "descriptors")
        print(f"name={args.shm} version={version} streams={stream_count} total={total} created_ns={created}")
        for i in range(stream_count):
            off = desc_off + i * DESC.size
            cam, stream, fmt, w, h, ch, bpp, slots, _, slot_bytes, slots_off = DESC.unpack_from(mm, off)
            if slots <= 0 or slot_bytes < SLOT_HEADER_SIZE: raise SystemExit(f"bad slot metadata for descriptor {i}")
            require_range(mm, slots_off, slots * slot_bytes, f"slots for descriptor {i}")
            print(f"{s(cam)}.{s(stream)} format={s(fmt)} {w}x{h} ch={ch} bpp={bpp} slots={slots} slot_bytes={slot_bytes} slots_offset={slots_off}")
        mm.close()
if __name__ == "__main__": main()
