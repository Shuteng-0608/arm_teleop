#!/usr/bin/env python3
"""
Generate a wall-mounted peg-in-hole MuJoCo scene whose wall surface is
parallel to the robot facing direction.

Input:
    model/right_arm_peg_tool.xml

Output:
    model/right_arm_peg_tool_wall.xml

Coordinate convention used here:
    x: robot forward/back direction, also horizontal direction along the wall
    y: wall normal / insertion depth direction
    z: vertical direction

Therefore:
    - the wall surface lies in the x-z plane
    - the hole ring lies in the x-z plane
    - the insertion axis is along the y direction
    - wall_task body origin is the hole center

This differs from the previous wall generator, where the wall surface was the
y-z plane and the insertion axis was x.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


# ---------------------------------------------------------------------------
# XML utilities
# ---------------------------------------------------------------------------

def get_repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def find_or_create(root: ET.Element, tag: str) -> ET.Element:
    elem = root.find(tag)
    if elem is None:
        elem = ET.SubElement(root, tag)
    return elem


def ensure_material(root: ET.Element, name: str, rgba: str) -> None:
    asset = find_or_create(root, "asset")

    for child in asset:
        if child.tag == "material" and child.get("name") == name:
            return

    mat = ET.SubElement(asset, "material")
    mat.set("name", name)
    mat.set("rgba", rgba)


def remove_scene_objects(worldbody: ET.Element) -> None:
    """
    Remove old tabletop/wall task objects while keeping the robot, floor and light.
    """
    names_to_remove = {
        # tabletop version
        "task_table",
        "round_hole_fixture",
        "task_cam",
        "side_cam",

        # wall versions
        "wall_panel",
        "wall_task",
        "wall_task_cam",
        "wall_side_cam",
    }

    for child in list(worldbody):
        name = child.get("name")
        if name in names_to_remove:
            worldbody.remove(child)


def remove_task_cameras(worldbody: ET.Element) -> None:
    camera_names = {
        "task_cam",
        "side_cam",
        "wall_task_cam",
        "wall_side_cam",
    }

    for child in list(worldbody):
        if child.tag == "camera" and child.get("name") in camera_names:
            worldbody.remove(child)


# ---------------------------------------------------------------------------
# Scene construction
# ---------------------------------------------------------------------------

def add_wall_cameras(worldbody: ET.Element) -> None:
    """
    Fixed cameras for the parallel-wall insertion task.
    You can still rotate the free camera in the MuJoCo viewer.
    """

    cam = ET.SubElement(worldbody, "camera")
    cam.set("name", "wall_task_cam")
    cam.set("pos", "0.55 -1.20 1.15")
    cam.set("xyaxes", "1 0 0 0 0.55 0.84")

    side_cam = ET.SubElement(worldbody, "camera")
    side_cam.set("name", "wall_side_cam")
    side_cam.set("pos", "-0.40 -0.95 1.00")
    side_cam.set("xyaxes", "1 0 0 0 0.45 0.89")


def add_box(
    parent: ET.Element,
    name: str,
    pos: str,
    size: str,
    material: str = "mat_table",
    contype: str = "1",
    conaffinity: str = "1",
    friction: str = "1.0 0.005 0.0001",
    rgba: str | None = None,
) -> ET.Element:
    geom = ET.SubElement(parent, "geom")
    geom.set("name", name)
    geom.set("type", "box")
    geom.set("pos", pos)
    geom.set("size", size)

    if material:
        geom.set("material", material)
    if rgba is not None:
        geom.set("rgba", rgba)

    geom.set("contype", contype)
    geom.set("conaffinity", conaffinity)
    geom.set("friction", friction)
    return geom


def add_wall_frame(
    wall: ET.Element,
    wall_depth: float,
    half_width_x: float,
    half_height_z: float,
    bar: float,
) -> None:
    """
    Build a rectangular vertical frame in the x-z plane.

    Coordinate convention:
        x: horizontal direction along the wall
        y: wall thickness / normal direction
        z: vertical direction
    """

    # 上边框
    add_box(
        wall,
        name="wall_top",
        pos=f"0 0 {half_height_z:.6f}",
        size=f"{half_width_x:.6f} {wall_depth:.6f} {bar:.6f}",
    )

    # 下边框
    add_box(
        wall,
        name="wall_bottom",
        pos=f"0 0 {-half_height_z:.6f}",
        size=f"{half_width_x:.6f} {wall_depth:.6f} {bar:.6f}",
    )

    # 左边框，沿 x 负方向
    add_box(
        wall,
        name="wall_left",
        pos=f"{-half_width_x:.6f} 0 0",
        size=f"{bar:.6f} {wall_depth:.6f} {half_height_z:.6f}",
    )

    # 右边框，沿 x 正方向
    add_box(
        wall,
        name="wall_right",
        pos=f"{half_width_x:.6f} 0 0",
        size=f"{bar:.6f} {wall_depth:.6f} {half_height_z:.6f}",
    )


def add_wall_hole_ring(
    wall: ET.Element,
    hole_inner_radius: float,
    peg_radius: float,
    wall_thickness: float,
    hole_depth: float,
    n_segments: int,
) -> None:
    """
    Build the circular hole directly in the x-z plane.

    The insertion/depth direction is the wall normal y direction.
    The ring lies in the x-z plane.

    For each small box:
        local x-axis = radial direction in x-z plane
        local y-axis = tangential direction in x-z plane
        local z-axis = local x cross local y = approximately world -y

    The sign of the insertion axis does not matter for a through-hole collision
    approximation, but the geometry is now visually and structurally aligned
    with a wall surface parallel to the robot facing direction.
    """

    outer_radius = hole_inner_radius + wall_thickness
    mid_radius = 0.5 * (hole_inner_radius + outer_radius)
    arc_len = 2.0 * math.pi * mid_radius / n_segments

    for i in range(n_segments):
        theta = 2.0 * math.pi * i / n_segments

        # Ring center line in the x-z plane.
        x = mid_radius * math.cos(theta)
        z = mid_radius * math.sin(theta)

        # radial direction in x-z plane.
        radial = [math.cos(theta), 0.0, math.sin(theta)]

        # tangent direction in x-z plane.
        tangent = [-math.sin(theta), 0.0, math.cos(theta)]

        block = ET.SubElement(wall, "geom")
        block.set("name", f"wall_hole_ring_{i:02d}")
        block.set("type", "box")
        block.set("pos", f"{x:.6f} 0 {z:.6f}")

        # local dimensions:
        #   radial thickness
        #   tangential arc length
        #   depth along wall normal
        block.set(
            "size",
            f"{0.5 * wall_thickness:.6f} "
            f"{0.5 * arc_len:.6f} "
            f"{0.5 * hole_depth:.6f}"
        )

        block.set(
            "xyaxes",
            f"{radial[0]:.6f} {radial[1]:.6f} {radial[2]:.6f} "
            f"{tangent[0]:.6f} {tangent[1]:.6f} {tangent[2]:.6f}"
        )

        block.set("material", "mat_hole")
        block.set("contype", "1")
        block.set("conaffinity", "1")
        block.set("friction", "0.8 0.005 0.0001")

    # Invisible site for future success metric / debugging.
    site = ET.SubElement(wall, "site")
    site.set("name", "hole_center_site")
    site.set("pos", "0 0 0")
    site.set("size", "0.001")
    site.set("rgba", "0 1 0 0")

    print(f"Peg radius:          {peg_radius:.4f} m")
    print(f"Hole inner radius:   {hole_inner_radius:.4f} m")
    print(f"Radial clearance:    {hole_inner_radius - peg_radius:.4f} m")
    print(f"Hole depth:          {hole_depth:.4f} m")
    print(f"Hole ring segments:  {n_segments}")


def add_wall_task_scene(
    worldbody: ET.Element,
    wall_pos: str,
    hole_inner_radius: float,
    peg_radius: float,
    wall_thickness: float,
    hole_depth: float,
    n_segments: int,
) -> None:
    """
    Add a complete wall-mounted peg-in-hole task.

    wall_task body origin = hole center.
    """

    wall = ET.SubElement(worldbody, "body")
    wall.set("name", "wall_task")
    wall.set("pos", wall_pos)

    # Rectangular wall/frame dimensions.
    wall_depth = 0.020       # half thickness along y
    half_width_x = 0.180     # half width along x
    half_height_z = 0.140    # half height along z
    bar = 0.020              # half width of frame bars

    add_wall_frame(
        wall,
        wall_depth=wall_depth,
        half_width_x=half_width_x,
        half_height_z=half_height_z,
        bar=bar,
    )

    add_wall_hole_ring(
        wall,
        hole_inner_radius=hole_inner_radius,
        peg_radius=peg_radius,
        wall_thickness=wall_thickness,
        hole_depth=hole_depth,
        n_segments=n_segments,
    )

    print(f"Wall task body pos:  {wall_pos}")
    print("Note: wall_task body origin is the hole center.")
    print("Wall surface: x-z plane. Insertion normal: y direction.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    repo_root = get_repo_root()

    default_src = repo_root / "model" / "right_arm_peg_tool.xml"
    default_dst = repo_root / "model" / "right_arm_peg_tool_wall.xml"

    parser = argparse.ArgumentParser(
        description="Generate a wall scene whose surface is parallel to the robot facing direction."
    )
    parser.add_argument(
        "--src",
        type=str,
        default=str(default_src),
        help="Source MuJoCo XML. Default: model/right_arm_peg_tool.xml",
    )
    parser.add_argument(
        "--dst",
        type=str,
        default=str(default_dst),
        help="Output wall-scene MuJoCo XML. Default: model/right_arm_peg_tool_wall.xml",
    )
    parser.add_argument(
        "--wall-pos",
        type=str,
        default="0.080 -0.350 0.900",
        help='Wall task position "x y z". The origin is the hole center.',
    )
    parser.add_argument(
        "--hole-radius",
        type=float,
        default=0.014,
        help="Inner radius of the hole in meters.",
    )
    parser.add_argument(
        "--peg-radius",
        type=float,
        default=0.010,
        help="Radius of the peg in meters. Used only for printed clearance.",
    )
    parser.add_argument(
        "--wall-thickness",
        type=float,
        default=0.010,
        help="Radial thickness of the hole ring blocks.",
    )
    parser.add_argument(
        "--hole-depth",
        type=float,
        default=0.045,
        help="Hole depth along wall normal y direction.",
    )
    parser.add_argument(
        "--segments",
        type=int,
        default=24,
        help="Number of small boxes used to approximate the circular hole.",
    )
    args = parser.parse_args()

    src = Path(args.src).expanduser().resolve()
    dst = Path(args.dst).expanduser().resolve()

    if not src.exists():
        raise FileNotFoundError(f"Cannot find source XML: {src}")

    parser_xml = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(src, parser=parser_xml)
    root = tree.getroot()

    root.set("model", "Right_Arm_Peg_Tool_Wall_Parallel")

    ensure_material(root, "mat_table", "0.45 0.45 0.45 1")
    ensure_material(root, "mat_hole", "0.12 0.12 0.14 1")
    ensure_material(root, "mat_peg", "0.90 0.18 0.10 1")
    ensure_material(root, "mat_goal", "0.10 0.80 0.20 0.45")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Cannot find <worldbody> in source XML.")

    remove_scene_objects(worldbody)
    remove_task_cameras(worldbody)
    add_wall_cameras(worldbody)

    add_wall_task_scene(
        worldbody=worldbody,
        wall_pos=args.wall_pos,
        hole_inner_radius=args.hole_radius,
        peg_radius=args.peg_radius,
        wall_thickness=args.wall_thickness,
        hole_depth=args.hole_depth,
        n_segments=args.segments,
    )

    try:
        ET.indent(tree, space="  ")
    except AttributeError:
        pass

    dst.parent.mkdir(parents=True, exist_ok=True)
    tree.write(dst, encoding="utf-8", xml_declaration=True)

    print()
    print(f"Generated wall scene XML:")
    print(f"  {dst}")
    print()
    print("Next test:")
    try:
        rel_dst = dst.relative_to(repo_root)
        print(f"  python3 scripts/view_mujoco_model.py --model {rel_dst}")
    except ValueError:
        print(f"  python3 scripts/view_mujoco_model.py --model {dst}")


if __name__ == "__main__":
    main()
