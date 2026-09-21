#!/usr/bin/env python3
"""Минимальный автономный парсер ArduPilot DataFlash для XKF2.

Читает FMT из бинарного DataFlash, находит XKF2 по имени и декодирует
TimeUS,C,AX,AY,AZ. Внешние зависимости не требуются.
"""
import argparse
import math
import statistics
import struct
from pathlib import Path

HEAD = b"\xA3\x95"
FMT_TYPE = 0x80
FMT_LEN = 89

# DataFlash format chars -> struct format, size.
# Нужный XKF2 использует только Q/B/c/h/f, но таблица шире для проверки FMT.
DF = {
    "b": ("b", 1), "B": ("B", 1),
    "h": ("h", 2), "H": ("H", 2),
    "i": ("i", 4), "I": ("I", 4),
    "q": ("q", 8), "Q": ("Q", 8),
    "f": ("f", 4), "d": ("d", 8),
    "c": ("h", 2), "C": ("H", 2),
    "e": ("i", 4), "E": ("I", 4),
    "L": ("i", 4), "M": ("B", 1),
    "n": ("4s", 4), "N": ("16s", 16), "Z": ("64s", 64),
    "a": ("32s", 32),
}

def cstr(b):
    return b.split(b"\0", 1)[0].decode("ascii", "replace")

def parse_fmt(rec):
    # header(3), Type(1), Length(1), Name(4), Format(16), Columns(64)
    return {
        "type": rec[3],
        "length": rec[4],
        "name": cstr(rec[5:9]),
        "format": cstr(rec[9:25]),
        "columns": cstr(rec[25:89]).split(","),
    }

def unpack_payload(fmt, payload):
    sf = "<" + "".join(DF[ch][0] for ch in fmt)
    return struct.unpack(sf, payload)

def stats(name, xs):
    if not xs:
        return f"{name}: no samples"
    med = statistics.median(xs)
    mean = statistics.fmean(xs)
    sd = statistics.pstdev(xs) if len(xs) > 1 else 0.0
    return (f"{name}: n={len(xs)} mean={mean:+.6f} median={med:+.6f} "
            f"sd={sd:.6f} min={min(xs):+.6f} max={max(xs):+.6f}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bin")
    ap.add_argument("--core", type=int, default=0)
    ap.add_argument("--csv", help="необязательный CSV с XKF2")
    args = ap.parse_args()

    data = Path(args.bin).read_bytes()
    fmts = {}
    i = 0
    while i + 3 <= len(data):
        if data[i:i+2] != HEAD:
            i += 1
            continue
        typ = data[i+2]
        if typ == FMT_TYPE and i + FMT_LEN <= len(data):
            f = parse_fmt(data[i:i+FMT_LEN])
            fmts[f["type"]] = f
            i += FMT_LEN
            continue
        f = fmts.get(typ)
        if f and f["length"] >= 3 and i + f["length"] <= len(data):
            i += f["length"]
        else:
            i += 1

    x = next((f for f in fmts.values() if f["name"] == "XKF2"), None)
    if not x:
        raise SystemExit("XKF2 FMT not found")
    print("XKF2 FMT:", x)

    expected = 3 + sum(DF[ch][1] for ch in x["format"])
    if expected != x["length"]:
        raise SystemExit(f"XKF2 length mismatch: FMT={x['length']} decoded={expected}")

    rows = []
    i = 0
    while i + 3 <= len(data):
        if data[i:i+2] != HEAD:
            i += 1
            continue
        typ = data[i+2]
        if typ == FMT_TYPE:
            i += FMT_LEN if i + FMT_LEN <= len(data) else 1
            continue
        f = fmts.get(typ)
        if not f or f["length"] < 3 or i + f["length"] > len(data):
            i += 1
            continue
        if f["name"] == "XKF2":
            vals = unpack_payload(f["format"], data[i+3:i+f["length"]])
            row = dict(zip(f["columns"], vals))
            # c/C are scaled by 100 in DataFlash.
            for k, ch in zip(f["columns"], f["format"]):
                if ch == "c": row[k] = row[k] / 100.0
                elif ch == "C": row[k] = row[k] / 100.0
            if int(row["C"]) == args.core:
                rows.append(row)
        i += f["length"]

    if not rows:
        raise SystemExit(f"No XKF2 samples for core {args.core}")

    print(f"core={args.core} samples={len(rows)}")
    for k in ("AX", "AY", "AZ"):
        print(stats(k, [float(r[k]) for r in rows]))

    mags = [math.sqrt(float(r["AX"])**2 + float(r["AY"])**2 + float(r["AZ"])**2) for r in rows]
    print(stats("|ABIAS|", mags))
    print("first:", {k: rows[0][k] for k in ("TimeUS","C","AX","AY","AZ")})
    print("last :", {k: rows[-1][k] for k in ("TimeUS","C","AX","AY","AZ")})

    if args.csv:
        import csv
        with open(args.csv, "w", newline="") as fp:
            w = csv.DictWriter(fp, fieldnames=["TimeUS","C","AX","AY","AZ"])
            w.writeheader()
            for r in rows:
                w.writerow({k:r[k] for k in w.fieldnames})
        print("csv:", args.csv)

if __name__ == "__main__":
    main()
