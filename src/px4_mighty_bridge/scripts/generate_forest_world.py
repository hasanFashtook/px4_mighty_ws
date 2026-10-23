#!/usr/bin/env python3
"""
generate_forest_world.py

Generates lightweight, evenly distributed forest worlds in SDF format for Gazebo Sim Harmonic and PX4 SITL.
Optimized for high Real-Time Factor (RTF >= 1.0) on CPU while providing realistic, challenging obstacle fields
for MIGHTY navigation.

Usage examples:
  # Default: 40m x 40m balanced forest with ~45-50 trees centered around (0,0)
  python3 generate_forest_world.py

  # Custom world name and spacing
  python3 generate_forest_world.py --world-name light_forest --size 40 --spacing 5.5

  # Regular grid mode with jitter
  python3 generate_forest_world.py --world-name grid_forest --mode grid --spacing 5.0

  # Then launch in PX4 SITL:
  ./run_px4_sitl.sh light_forest
"""

import argparse
import csv
import math
import os
import random
import sys


def generate_poisson_disk_samples(width, height, min_dist, clear_center_radius, seed=None):
    """
    Bridson's algorithm for Poisson-disk sampling in 2D bounded box [-W/2, W/2] x [-H/2, H/2].
    Guarantees that no two points are closer than min_dist, and no points fall within clear_center_radius.
    """
    if seed is not None:
        random.seed(seed)

    k = 30  # Candidate points per active point
    cell_size = min_dist / math.sqrt(2)

    # Offset to grid coordinates [0, cols-1], [0, rows-1]
    cols = int(math.ceil(width / cell_size))
    rows = int(math.ceil(height / cell_size))

    grid = [[None for _ in range(rows)] for _ in range(cols)]
    points = []
    active_indices = []

    def to_grid(x, y):
        gx = int((x + width / 2.0) / cell_size)
        gy = int((y + height / 2.0) / cell_size)
        return min(max(gx, 0), cols - 1), min(max(gy, 0), rows - 1)

    def is_valid(x, y):
        # Boundary check
        if abs(x) > width / 2.0 or abs(y) > height / 2.0:
            return False
        # Clear center circle check
        if math.hypot(x, y) < clear_center_radius:
            return False

        gx, gy = to_grid(x, y)
        x_min = max(0, gx - 2)
        x_max = min(cols - 1, gx + 2)
        y_min = max(0, gy - 2)
        y_max = min(rows - 1, gy + 2)

        for ix in range(x_min, x_max + 1):
            for iy in range(y_min, y_max + 1):
                pt_idx = grid[ix][iy]
                if pt_idx is not None:
                    px, py = points[pt_idx]
                    if math.hypot(x - px, y - py) < min_dist:
                        return False
        return True

    # Seed initial point outside the center clear zone
    for _ in range(100):
        init_x = random.uniform(-width / 2.0, width / 2.0)
        init_y = random.uniform(-height / 2.0, height / 2.0)
        if is_valid(init_x, init_y):
            pt_idx = len(points)
            points.append((init_x, init_y))
            gx, gy = to_grid(init_x, init_y)
            grid[gx][gy] = pt_idx
            active_indices.append(pt_idx)
            break

    while active_indices:
        idx_in_active = random.randint(0, len(active_indices) - 1)
        base_idx = active_indices[idx_in_active]
        bx, by = points[base_idx]

        found = False
        for _ in range(k):
            angle = random.uniform(0, 2 * math.pi)
            dist = random.uniform(min_dist, 2 * min_dist)
            cand_x = bx + dist * math.cos(angle)
            cand_y = by + dist * math.sin(angle)

            if is_valid(cand_x, cand_y):
                new_idx = len(points)
                points.append((cand_x, cand_y))
                gx, gy = to_grid(cand_x, cand_y)
                grid[gx][gy] = new_idx
                active_indices.append(new_idx)
                found = True
                break

        if not found:
            active_indices.pop(idx_in_active)

    return points


def generate_grid_samples(width, height, spacing, clear_center_radius, jitter=0.6, seed=None):
    """
    Evenly spaced regular grid with optional random jitter.
    """
    if seed is not None:
        random.seed(seed)

    points = []
    nx = int(width / spacing)
    ny = int(height / spacing)

    x_start = -((nx - 1) * spacing) / 2.0
    y_start = -((ny - 1) * spacing) / 2.0

    for i in range(nx):
        for j in range(ny):
            base_x = x_start + i * spacing
            base_y = y_start + j * spacing

            jx = random.uniform(-jitter, jitter) if jitter > 0 else 0.0
            jy = random.uniform(-jitter, jitter) if jitter > 0 else 0.0
            x = base_x + jx
            y = base_y + jy

            if math.hypot(x, y) >= clear_center_radius and abs(x) <= width / 2.0 and abs(y) <= height / 2.0:
                points.append((x, y))

    return points


def generate_ring_samples(radius_max, ring_step, clear_center_radius, seed=None):
    """
    Concentric rings of trees around the origin.
    """
    if seed is not None:
        random.seed(seed)

    points = []
    r = max(ring_step, clear_center_radius)
    while r <= radius_max:
        circumference = 2 * math.pi * r
        num_trees = max(6, int(circumference / ring_step))
        phase = random.uniform(0, 2 * math.pi)

        for i in range(num_trees):
            angle = phase + (2 * math.pi * i) / num_trees
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y))
        r += ring_step

    return points


def build_sdf_xml(world_name, trees, ground_size, tree_height):
    """
    Builds a complete, valid Gazebo Sim Harmonic SDF file containing PX4 systems,
    lighting, physics, ground plane, and cylinders for trees.
    """
    ground_w, ground_h = ground_size
    sdf_lines = [
        "<?xml version='1.0' encoding='UTF-8'?>",
        "<sdf version='1.8'>",
        f"  <world name='{world_name}'>",
        "",
        "    <!-- Standard Gazebo Sim Harmonic Systems for PX4 & ROS 2 -->",
        "    <plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>",
        "    <plugin filename='gz-sim-sensors-system' name='gz::sim::systems::Sensors'>",
        "      <render_engine>ogre2</render_engine>",
        "    </plugin>",
        "    <plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>",
        "    <plugin filename='gz-sim-user-commands-system' name='gz::sim::systems::UserCommands'/>",
        "",
        "    <!-- Environment Constants -->",
        "    <gravity>0 0 -9.8</gravity>",
        "    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>",
        "    <atmosphere type='adiabatic'/>",
        "",
        "    <!-- ODE Physics Tuned for PX4 SITL (RTF=1.0) -->",
        "    <physics type='ode'>",
        "      <max_step_size>0.004</max_step_size>",
        "      <real_time_factor>1.0</real_time_factor>",
        "      <real_time_update_rate>250</real_time_update_rate>",
        "    </physics>",
        "",
        "    <!-- Ambient & Sun Lighting -->",
        "    <scene>",
        "      <ambient>0.5 0.5 0.5 1</ambient>",
        "      <background>0.65 0.75 0.85 1</background>",
        "      <shadows>false</shadows>",
        "    </scene>",
        "",
        "    <!-- Standard Zurich Coordinates for PX4 -->",
        "    <spherical_coordinates>",
        "      <surface_model>EARTH_WGS84</surface_model>",
        "      <world_frame_orientation>ENU</world_frame_orientation>",
        "      <latitude_deg>47.397971057728974</latitude_deg>",
        "      <longitude_deg>8.546163739800146</longitude_deg>",
        "      <elevation>0</elevation>",
        "    </spherical_coordinates>",
        "",
        "    <light name='sun' type='directional'>",
        "      <cast_shadows>0</cast_shadows>",
        "      <pose>0 0 30 0 0 0</pose>",
        "      <diffuse>0.9 0.9 0.9 1</diffuse>",
        "      <specular>0.2 0.2 0.2 1</specular>",
        "      <attenuation>",
        "        <range>1000</range>",
        "        <constant>0.9</constant>",
        "        <linear>0.01</linear>",
        "        <quadratic>0.001</quadratic>",
        "      </attenuation>",
        "      <direction>-0.5 0.2 -1.0</direction>",
        "    </light>",
        "",
        "    <!-- Ground Plane -->",
        "    <model name='ground_plane'>",
        "      <static>true</static>",
        "      <link name='link'>",
        "        <collision name='collision'>",
        "          <geometry>",
        "            <plane>",
        "              <normal>0 0 1</normal>",
        f"              <size>{ground_w} {ground_h}</size>",
        "            </plane>",
        "          </geometry>",
        "          <surface>",
        "            <friction>",
        "              <ode>",
        "                <mu>100</mu>",
        "                <mu2>50</mu2>",
        "              </ode>",
        "            </friction>",
        "          </surface>",
        "        </collision>",
        "        <visual name='visual'>",
        "          <geometry>",
        "            <plane>",
        "              <normal>0 0 1</normal>",
        f"              <size>{ground_w} {ground_h}</size>",
        "            </plane>",
        "          </geometry>",
        "          <material>",
        "            <ambient>0.2 0.35 0.2 1.0</ambient>",
        "            <diffuse>0.25 0.4 0.22 1.0</diffuse>",
        "            <specular>0.1 0.1 0.1 1.0</specular>",
        "          </material>",
        "        </visual>",
        "      </link>",
        "      <pose>0 0 0 0 0 0</pose>",
        "    </model>",
        "",
        f"    <!-- Trees ({len(trees)} obstacles) -->",
    ]

    z_center = tree_height / 2.0

    # Color palette for tree trunks (warm forest wood / cedar tones)
    materials = [
        {"amb": "0.35 0.22 0.14 1.0", "diff": "0.45 0.28 0.18 1.0"},
        {"amb": "0.30 0.19 0.12 1.0", "diff": "0.40 0.24 0.15 1.0"},
        {"amb": "0.28 0.22 0.16 1.0", "diff": "0.38 0.30 0.20 1.0"},
        {"amb": "0.22 0.26 0.20 1.0", "diff": "0.32 0.36 0.28 1.0"},
    ]

    for i, (tx, ty, r) in enumerate(trees):
        mat = materials[i % len(materials)]
        tree_xml = f"""    <model name='tree_{i}'>
      <static>true</static>
      <pose>{tx:.3f} {ty:.3f} {z_center:.3f} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <cylinder>
              <radius>{r:.3f}</radius>
              <length>{tree_height:.3f}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <cylinder>
              <radius>{r:.3f}</radius>
              <length>{tree_height:.3f}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{mat['amb']}</ambient>
            <diffuse>{mat['diff']}</diffuse>
            <specular>0.1 0.1 0.1 1.0</specular>
          </material>
        </visual>
      </link>
    </model>"""
        sdf_lines.append(tree_xml)

    sdf_lines.append("  </world>")
    sdf_lines.append("</sdf>\n")
    return "\n".join(sdf_lines)


def main():
    parser = argparse.ArgumentParser(
        description="Generate lightweight, evenly distributed forest worlds in SDF for PX4 SITL & MIGHTY."
    )
    parser.add_argument(
        "--world-name",
        type=str,
        default="balanced_forest",
        help="Name of the generated world (default: balanced_forest)",
    )
    parser.add_argument(
        "--size",
        type=float,
        default=40.0,
        help="Side length of the forest area in meters (default: 40.0m)",
    )
    parser.add_argument(
        "--width",
        type=float,
        default=None,
        help="Custom width in X (defaults to --size)",
    )
    parser.add_argument(
        "--height",
        type=float,
        default=None,
        help="Custom height in Y (defaults to --size)",
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=4.8,
        help="Minimum distance between trees in meters (default: 4.8m)",
    )
    parser.add_argument(
        "--clear-center",
        type=float,
        default=3.0,
        help="Safety radius around origin (0,0) left clear for takeoff/landing (default: 3.0m)",
    )
    parser.add_argument(
        "--tree-radius-min",
        type=float,
        default=0.30,
        help="Minimum tree radius in meters (default: 0.30m)",
    )
    parser.add_argument(
        "--tree-radius-max",
        type=float,
        default=0.45,
        help="Maximum tree radius in meters (default: 0.45m)",
    )
    parser.add_argument(
        "--tree-height",
        type=float,
        default=5.0,
        help="Tree trunk height in meters (default: 5.0m)",
    )
    parser.add_argument(
        "--mode",
        choices=["poisson", "grid", "rings"],
        default="poisson",
        help="Distribution mode: 'poisson' (natural uniform), 'grid' (regular grid), 'rings' (concentric rings)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducible forest generation",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Output directory (default: px4_mighty_bridge/worlds/)",
    )

    args = parser.parse_args()

    width = args.width if args.width is not None else args.size
    height = args.height if args.height is not None else args.size

    # Determine output directory
    real_script_path = os.path.realpath(__file__)
    script_dir = os.path.dirname(real_script_path)
    if args.output_dir:
        out_dir = os.path.abspath(args.output_dir)
    else:
        # Default to ../worlds relative to real script location
        candidate = os.path.abspath(os.path.join(script_dir, "..", "worlds"))
        if os.path.isdir(candidate):
            out_dir = candidate
        else:
            # Fallback to current working directory src/px4_mighty_bridge/worlds if present
            cwd_candidate = os.path.join(os.getcwd(), "src", "px4_mighty_bridge", "worlds")
            if os.path.isdir(cwd_candidate):
                out_dir = cwd_candidate
            else:
                out_dir = os.getcwd()

    os.makedirs(out_dir, exist_ok=True)

    print("==================================================")
    print(f" Generating Forest World: {args.world_name}")
    print(f" Area: {width:.1f}m x {height:.1f}m (centered at (0,0))")
    print(f" Distribution mode: {args.mode}")
    print(f" Tree spacing: >= {args.spacing:.1f}m")
    print(f" Center clear radius: {args.clear_center:.1f}m")
    print("==================================================")

    # Generate 2D sample positions
    if args.mode == "poisson":
        pts = generate_poisson_disk_samples(
            width, height, args.spacing, args.clear_center, seed=args.seed
        )
    elif args.mode == "grid":
        pts = generate_grid_samples(
            width, height, args.spacing, args.clear_center, jitter=0.5, seed=args.seed
        )
    elif args.mode == "rings":
        pts = generate_ring_samples(
            max(width, height) / 2.0, args.spacing, args.clear_center, seed=args.seed
        )

    # Assign radius to each tree
    random.seed(args.seed)
    trees = []
    for x, y in pts:
        r = random.uniform(args.tree_radius_min, args.tree_radius_max)
        trees.append((x, y, r))

    print(f"Total trees generated: {len(trees)}")

    # Ground plane slightly larger than forest area
    ground_size = (width + 10.0, height + 10.0)

    # Generate SDF content
    sdf_content = build_sdf_xml(args.world_name, trees, ground_size, args.tree_height)

    # Output file paths
    sdf_path = os.path.join(out_dir, f"{args.world_name}.sdf")
    csv_path = os.path.join(out_dir, f"{args.world_name}.csv")

    with open(sdf_path, "w", encoding="utf-8") as f:
        f.write(sdf_content)
    print(f"[+] Saved SDF world to: {sdf_path}")

    # Also save companion CSV for analysis or benchmarking
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["id", "x", "y", "radius", "height"])
        for i, (tx, ty, tr) in enumerate(trees):
            writer.writerow([i, f"{tx:.3f}", f"{ty:.3f}", f"{tr:.3f}", f"{args.tree_height:.3f}"])
    print(f"[+] Saved companion CSV to: {csv_path}")

    print("\nYou can now launch this world directly via:")
    print(f"  ./run_px4_sitl.sh {args.world_name}")
    print("==================================================")


if __name__ == "__main__":
    main()
