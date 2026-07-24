#!/usr/bin/env python3
"""
Extract bounding-box dimensions (and volume, for the later mass pass)
from all robot_arm1 STL files, and report both raw and 4.8x-scaled values.

Run from inside the meshes/arm1/ directory, or edit MESH_DIR below.
"""

import trimesh
import os

SCALE = 4.8
MESH_DIR = "."  # change to full path if not running from inside meshes/arm1/

LINKS = ["base", "link1", "link2", "link3", "finger_a", "finger_b"]

def measure(name):
    path = os.path.join(MESH_DIR, f"{name}.stl")
    if not os.path.exists(path):
        print(f"{name:10s}  MISSING: {path}")
        return None

    m = trimesh.load(path)
    bounds = m.bounds  # [[min_x,min_y,min_z],[max_x,max_y,max_z]]
    dims_raw = bounds[1] - bounds[0]  # x,y,z extents
    dims_scaled = dims_raw * SCALE
    volume_raw = m.volume
    volume_scaled = volume_raw * (SCALE ** 3)  # volume scales cubically

    print(f"\n{name}")
    print(f"  raw bounds      : min={bounds[0]}, max={bounds[1]}")
    print(f"  raw dims (x,y,z): {dims_raw}")
    print(f"  scaled dims     : {dims_scaled}   (x{SCALE})")
    print(f"  raw volume      : {volume_raw:.6f} m^3")
    print(f"  scaled volume   : {volume_scaled:.6f} m^3   (x{SCALE**3:.1f})")

    return {
        "name": name,
        "dims_raw": dims_raw,
        "dims_scaled": dims_scaled,
        "volume_raw": volume_raw,
        "volume_scaled": volume_scaled,
    }

if __name__ == "__main__":
    results = []
    for link in LINKS:
        r = measure(link)
        if r:
            results.append(r)

    print("\n" + "="*60)
    print("SUMMARY (scaled dims, meters)")
    print("="*60)
    for r in results:
        d = r["dims_scaled"]
        print(f"{r['name']:10s}  x={d[0]:.4f}  y={d[1]:.4f}  z={d[2]:.4f}  vol={r['volume_scaled']:.6f} m^3")