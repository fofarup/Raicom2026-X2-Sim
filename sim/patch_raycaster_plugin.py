#!/usr/bin/env python3
"""Generate a project-owned fix for the official ray-caster plugin.

The distributed plugin reads the two-value ``size`` attribute with a minimum
length of one in both camera and lidar constructors.  The matching MuJoCo
``nsensordata`` callbacks already use two.  The bad constructors consequently
leave the second dimension at zero, so the simulator publishes an empty cloud.

Only a generated copy is modified.  The official binary is never overwritten.
"""
from __future__ import annotations

import argparse
import hashlib
import shutil
from pathlib import Path


KNOWN_SHA256 = {
    "6fe5e20e5a6c85bb8a7d959d90d8852f0f57677cdd9e6cbb0dd0422f24cef0b2"
}
# x86-64 ``mov edx, 1`` immediate bytes in camera and lidar constructors.
PATCHES = {0x1328B: (0x01, 0x02), 0x142DD: (0x01, 0x02)}


def patch(source: Path, target: Path) -> Path:
    payload = bytearray(source.read_bytes())
    digest = hashlib.sha256(payload).hexdigest()
    if digest not in KNOWN_SHA256:
        raise RuntimeError(
            f"拒绝修改未知版本插件: sha256={digest}; 请先重新审计偏移")
    for offset, (expected, replacement) in PATCHES.items():
        if payload[offset] != expected:
            raise RuntimeError(
                f"偏移 0x{offset:x} 预期 0x{expected:02x}，实际 "
                f"0x{payload[offset]:02x}")
        payload[offset] = replacement
    target.parent.mkdir(parents=True, exist_ok=True)
    # copy2 preserves the provenance timestamp/mode before replacing contents.
    shutil.copy2(source, target)
    target.write_bytes(payload)
    return target


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("target", type=Path)
    args = parser.parse_args()
    print(patch(args.source, args.target))


if __name__ == "__main__":
    main()
