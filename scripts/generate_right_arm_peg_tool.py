#!/usr/bin/env python3
from pathlib import Path
import math
import xml.etree.ElementTree as ET


HAND_BODY_NAMES = {
    "link_hand",
    "link_thumb_1", "link_thumb_2",
    "link_index_1", "link_index_2",
    "link_middle_1", "link_middle_2",
    "link_ring_1", "link_ring_2",
    "link_little_1", "link_little_2",
}

HAND_JOINT_NAMES = {
    "joint_hand",
    "joint_thumb_1", "joint_thumb_2",
    "joint_index_1", "joint_index_2",
    "joint_middle_1", "joint_middle_2",
    "joint_ring_1", "joint_ring_2",
    "joint_little_1", "joint_little_2",
}

HAND_MESH_NAMES = {
    "link_hand",
    "link_thumb_1", "link_thumb_2",
    "link_index_1", "link_index_2",
    "link_middle_1", "link_middle_2",
    "link_ring_1", "link_ring_2",
    "link_little_1", "link_little_2",
}


def find_first(root, tag):
    elem = root.find(tag)
    if elem is None:
        elem = ET.SubElement(root, tag)
    return elem


def find_body_by_name(elem, name):
    if elem.tag == "body" and elem.get("name") == name:
        return elem

    for child in list(elem):
        found = find_body_by_name(child, name)
        if found is not None:
            return found

    return None


def remove_body_by_name(parent, name):
    """
    Remove a body recursively.
    Return the removed body if found.
    """
    for child in list(parent):
        if child.tag == "body" and child.get("name") == name:
            parent.remove(child)
            return child

        removed = remove_body_by_name(child, name)
        if removed is not None:
            return removed

    return None


def remove_actuators_for_deleted_joints(root):
    actuator = root.find("actuator")
    if actuator is None:
        return

    for motor in list(actuator):
        joint_name = motor.get("joint")
        actuator_name = motor.get("name", "")

        if joint_name in HAND_JOINT_NAMES:
            actuator.remove(motor)
            continue

        if actuator_name.startswith("motor_joint_thumb"):
            actuator.remove(motor)
            continue
        if actuator_name.startswith("motor_joint_index"):
            actuator.remove(motor)
            continue
        if actuator_name.startswith("motor_joint_middle"):
            actuator.remove(motor)
            continue
        if actuator_name.startswith("motor_joint_ring"):
            actuator.remove(motor)
            continue
        if actuator_name.startswith("motor_joint_little"):
            actuator.remove(motor)
            continue
        if actuator_name == "motor_joint_hand":
            actuator.remove(motor)
            continue


def remove_hand_mesh_assets(root):
    asset = root.find("asset")
    if asset is None:
        return

    for elem in list(asset):
        if elem.tag == "mesh" and elem.get("name") in HAND_MESH_NAMES:
            asset.remove(elem)


def enable_contact(root):
    """
    Remove <flag contact="disable"/> if it exists.
    """
    option = root.find("option")
    if option is None:
        return

    for child in list(option):
        if child.tag == "flag" and child.get("contact") == "disable":
            option.remove(child)


def add_collision_default(root):
    """
    Disable collision for original robot geoms by default.
    Task geoms such as peg/table/hole will explicitly enable collision.
    """
    # Avoid adding duplicate default block.
    for default in root.findall("default"):
        geom = default.find("geom")
        if geom is not None and geom.get("contype") == "0" and geom.get("conaffinity") == "0":
            return

    default = ET.Element("default")
    geom = ET.SubElement(default, "geom")
    geom.set("contype", "0")
    geom.set("conaffinity", "0")

    # Insert after compiler if possible.
    compiler = root.find("compiler")
    children = list(root)
    if compiler is not None:
        idx = children.index(compiler)
        root.insert(idx + 1, default)
    else:
        root.insert(0, default)


def add_materials(root):
    asset = find_first(root, "asset")

    existing_names = {
        elem.get("name")
        for elem in asset
        if elem.tag == "material" and elem.get("name")
    }

    def add_mat(name, rgba):
        if name not in existing_names:
            mat = ET.SubElement(asset, "material")
            mat.set("name", name)
            mat.set("rgba", rgba)

    add_mat("mat_table", "0.45 0.45 0.45 1")
    add_mat("mat_hole", "0.12 0.12 0.14 1")
    add_mat("mat_peg", "0.90 0.18 0.10 1")
    add_mat("mat_goal", "0.10 0.80 0.20 0.45")


def add_peg_to_link7(root, removed_hand_body):
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Cannot find <worldbody>.")

    link7 = find_body_by_name(worldbody, "link_7")
    if link7 is None:
        raise RuntimeError("Cannot find body named 'link_7'. Please check your XML body names.")

    # Use the original link_hand pose if it existed.
    peg_body_attrs = {"name": "peg_tool"}

    if removed_hand_body is not None:
        if removed_hand_body.get("pos") is not None:
            peg_body_attrs["pos"] = removed_hand_body.get("pos")
        if removed_hand_body.get("quat") is not None:
            peg_body_attrs["quat"] = removed_hand_body.get("quat")
        if removed_hand_body.get("euler") is not None:
            peg_body_attrs["euler"] = removed_hand_body.get("euler")
    else:
        # Fallback if link_hand was not found.
        peg_body_attrs["pos"] = "0 0 0"

    # Avoid duplicate peg_tool.
    old_peg = None
    for child in list(link7):
        if child.tag == "body" and child.get("name") == "peg_tool":
            old_peg = child
            break
    if old_peg is not None:
        link7.remove(old_peg)

    peg_body = ET.SubElement(link7, "body", peg_body_attrs)

    # Cylinder in MuJoCo is along local z axis.
    # Here the peg extends along negative local z.
    peg_radius = 0.010
    peg_half_length = 0.090
    peg_center_z = -peg_half_length
    peg_tip_z = -2.0 * peg_half_length

    peg = ET.SubElement(peg_body, "geom")
    peg.set("name", "cylindrical_peg")
    peg.set("type", "cylinder")
    peg.set("pos", f"0 0 {peg_center_z:.6f}")
    peg.set("size", f"{peg_radius:.6f} {peg_half_length:.6f}")
    peg.set("material", "mat_peg")
    peg.set("mass", "0.08")
    peg.set("contype", "1")
    peg.set("conaffinity", "1")
    peg.set("friction", "1.0 0.005 0.0001")

    tip = ET.SubElement(peg_body, "site")
    tip.set("name", "peg_tip_site")
    tip.set("pos", f"0 0 {peg_tip_z:.6f}")
    tip.set("size", "0.007")
    tip.set("rgba", "0 1 0 1")

    base = ET.SubElement(peg_body, "site")
    base.set("name", "peg_base_site")
    base.set("pos", "0 0 0")
    base.set("size", "0.006")
    base.set("rgba", "0 0.3 1 1")


def add_round_hole_scene(root):
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Cannot find <worldbody>.")

    # Remove old task scene if regenerated.
    for child in list(worldbody):
        if child.get("name") in {
            "task_key_light",
            "task_cam",
            "side_cam",
            "floor",
            "task_table",
            "round_hole_fixture",
        }:
            worldbody.remove(child)

    # Light.
    light = ET.SubElement(worldbody, "light")
    light.set("name", "task_key_light")
    light.set("pos", "0.6 -1.0 2.2")
    light.set("dir", "-0.4 0.7 -1")
    light.set("diffuse", "0.8 0.8 0.8")
    light.set("specular", "0.2 0.2 0.2")

    # Cameras.
    cam = ET.SubElement(worldbody, "camera")
    cam.set("name", "task_cam")
    cam.set("pos", "0.75 -1.20 1.55")
    cam.set("xyaxes", "1 0 0 0 0.65 0.76")

    side_cam = ET.SubElement(worldbody, "camera")
    side_cam.set("name", "side_cam")
    side_cam.set("pos", "-0.15 -1.10 1.30")
    side_cam.set("xyaxes", "1 0 0 0 0.45 0.89")

    # Floor.
    floor = ET.SubElement(worldbody, "geom")
    floor.set("name", "floor")
    floor.set("type", "plane")
    floor.set("pos", "0 0 0")
    floor.set("size", "2.0 2.0 0.02")
    floor.set("rgba", "0.80 0.82 0.85 1")
    floor.set("contype", "1")
    floor.set("conaffinity", "1")
    floor.set("friction", "1.0 0.005 0.0001")

    # Table.
    table = ET.SubElement(worldbody, "body")
    table.set("name", "task_table")
    table.set("pos", "0.35 -0.45 1.05")

    table_top = ET.SubElement(table, "geom")
    table_top.set("name", "table_top")
    table_top.set("type", "box")
    table_top.set("size", "0.45 0.32 0.025")
    table_top.set("material", "mat_table")
    table_top.set("contype", "1")
    table_top.set("conaffinity", "1")
    table_top.set("friction", "1.0 0.005 0.0001")

    # Circular hole fixture.
    fixture = ET.SubElement(worldbody, "body")
    fixture.set("name", "round_hole_fixture")
    fixture.set("pos", "0.35 -0.45 1.105")

    peg_radius = 0.010
    hole_inner_radius = 0.013
    wall_thickness = 0.010
    wall_height = 0.045
    n_segments = 20

    outer_radius = hole_inner_radius + wall_thickness
    mid_radius = 0.5 * (hole_inner_radius + outer_radius)
    arc_len = 2.0 * math.pi * mid_radius / n_segments

    for i in range(n_segments):
        theta = 2.0 * math.pi * i / n_segments

        x = mid_radius * math.cos(theta)
        y = mid_radius * math.sin(theta)

        radial_x = math.cos(theta)
        radial_y = math.sin(theta)

        tang_x = -math.sin(theta)
        tang_y = math.cos(theta)

        block = ET.SubElement(fixture, "geom")
        block.set("name", f"hole_ring_{i:02d}")
        block.set("type", "box")
        block.set("pos", f"{x:.6f} {y:.6f} {0.5 * wall_height:.6f}")
        block.set(
            "size",
            f"{0.5 * wall_thickness:.6f} {0.5 * arc_len:.6f} {0.5 * wall_height:.6f}",
        )
        block.set(
            "xyaxes",
            f"{radial_x:.6f} {radial_y:.6f} 0 {tang_x:.6f} {tang_y:.6f} 0",
        )
        block.set("material", "mat_hole")
        block.set("contype", "1")
        block.set("conaffinity", "1")
        block.set("friction", "0.8 0.005 0.0001")

    hole_center = ET.SubElement(fixture, "site")
    hole_center.set("name", "hole_center_site")
    hole_center.set("pos", "0 0 0.060")
    hole_center.set("size", "0.010")
    hole_center.set("material", "mat_goal")

    hole_axis = ET.SubElement(fixture, "site")
    hole_axis.set("name", "hole_axis_site")
    hole_axis.set("pos", "0 0 0.100")
    hole_axis.set("size", "0.006")
    hole_axis.set("rgba", "0.1 0.8 0.2 0.8")

    print("Peg radius:", peg_radius)
    print("Hole inner radius:", hole_inner_radius)
    print("Radial clearance:", hole_inner_radius - peg_radius)


def main():
    repo_root = Path(__file__).resolve().parents[1]

    src = repo_root / "model" / "right_arm_stable.xml"
    dst = repo_root / "model" / "right_arm_peg_tool.xml"

    if not src.exists():
        raise FileNotFoundError(f"Cannot find source XML: {src}")

    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(src, parser=parser)
    root = tree.getroot()

    enable_contact(root)
    add_collision_default(root)
    add_materials(root)

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Cannot find <worldbody>.")

    # Remove hand subtree.
    removed_hand_body = remove_body_by_name(worldbody, "link_hand")
    if removed_hand_body is None:
        print("Warning: body 'link_hand' was not found. Peg will be attached to link_7 origin.")
    else:
        print("Removed body subtree: link_hand")

    remove_actuators_for_deleted_joints(root)
    remove_hand_mesh_assets(root)

    add_peg_to_link7(root, removed_hand_body)
    add_round_hole_scene(root)

    try:
        ET.indent(tree, space="  ")
    except AttributeError:
        pass

    tree.write(dst, encoding="utf-8", xml_declaration=True)

    print(f"\nGenerated: {dst}")
    print("Next: load this XML with MuJoCo to verify the model.")


if __name__ == "__main__":
    main()