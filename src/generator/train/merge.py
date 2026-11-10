#!/usr/bin/env python3
import os, yaml, math, numpy as np
import xml.etree.ElementTree as ET

###

with open('config/params.yaml', 'r') as file:
    config = yaml.safe_load(file)

def get_vector(text, default=(0.0, 0.0, 0.0)):
    if text is None:
        return np.array(default, dtype=float)

    vals = [float(x) for x in text.split()]
    if len(vals) != 3:
        return np.array(default, dtype=float)

    return np.array(vals, dtype=float)


def get_root_link_name(robot_elem):
    child_links = set()
    all_links = []

    for joint in robot_elem.findall("joint"):
        child = joint.find("child")
        if child is not None and "link" in child.attrib:
            child_links.add(child.attrib["link"])

    for link in robot_elem.findall("link"):
        name = link.attrib["name"]
        all_links.append(name)

    roots = [name for name in all_links if name not in child_links]
    if not roots:
        raise ValueError("No root link found in URDF.")

    return roots[0]


def compute_origin_to_transform(origin_elem):
    xyz = np.zeros(3, dtype=float)
    rpy = np.zeros(3, dtype=float)
    if origin_elem is not None:
        xyz = get_vector(origin_elem.attrib.get("xyz"), (0.0, 0.0, 0.0))
        rpy = get_vector(origin_elem.attrib.get("rpy"), (0.0, 0.0, 0.0))

    T = np.eye(4, dtype=float)
    
    roll, pitch, yaw = rpy
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ], dtype=float)

    T[:3, :3] = R
    T[:3, 3] = xyz

    return T


def compute_link_world_transforms(robot_elem):
    root_link = get_root_link_name(robot_elem)

    parent_to_joints = {}
    for joint in robot_elem.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        
        if parent is None or child is None:
            continue
        parent_link = parent.attrib.get("link")
        child_link = child.attrib.get("link")
        
        if parent_link is None or child_link is None:
            continue
        parent_to_joints.setdefault(parent_link, []).append(joint)

    link_T = {root_link: np.eye(4, dtype=float)}
    stack = [root_link]

    while stack:
        parent_link = stack.pop()
        T_parent = link_T[parent_link]

        for joint in parent_to_joints.get(parent_link, []):
            child = joint.find("child")
            if child is None:
                continue
            child_link = child.attrib.get("link")
            if not child_link:
                continue

            T_joint = compute_origin_to_transform(joint.find("origin"))
            T_child = T_parent @ T_joint

            if child_link not in link_T:
                link_T[child_link] = T_child
                stack.append(child_link)

    return link_T


def compute_urdf_xy_bounds(urdf_path, circle_samples=16):
    tree = ET.parse(urdf_path)
    robot = tree.getroot()
    link_T = compute_link_world_transforms(robot)

    bounds = [float("inf"), float("-inf"), float("inf"), float("-inf")]
    found_any = False

    for link in robot.findall("link"):
        link_name = link.attrib.get("name")
        if link_name not in link_T:
            continue

        T_link = link_T[link_name]

        for vis_or_col in list(link.findall("visual")) + list(link.findall("collision")):
            geom = vis_or_col.find("geometry")
            if geom is None:
                continue

            cyl = geom.find("cylinder")
            if cyl is None:
                continue

            length = float(cyl.attrib.get("length", "0.0"))
            radius = float(cyl.attrib.get("radius", "0.0"))

            T_geom = compute_origin_to_transform(vis_or_col.find("origin"))
            T = T_link @ T_geom

            # Sample the two circular faces of the cylinder.
            for z in (-0.5 * length, 0.5 * length):
                for k in range(circle_samples):
                    theta = 2.0 * math.pi * k / circle_samples
                    x = radius * math.cos(theta)
                    y = radius * math.sin(theta)
                    
                    q = T @ np.array([x, y, z, 1.0], dtype=float)
                    p_world = q[:3]
                    
                    xmin, xmax, ymin, ymax = bounds
                    x, y = float(p_world[0]), float(p_world[1])
                    bounds = (min(xmin, x), max(xmax, x), min(ymin, y), max(ymax, y))
                    found_any = True

    if not found_any:
        return None

    return tuple(bounds)


def compute_urdf_xy_center(urdf_paths):
    global_bounds = [float("inf"), float("-inf"), float("inf"), float("-inf")]
    found_any = False

    for urdf_path in urdf_paths:
        b = compute_urdf_xy_bounds(urdf_path)
        if b is None:
            continue

        xmin, xmax, ymin, ymax = b
        global_bounds[0] = min(global_bounds[0], xmin)
        global_bounds[1] = max(global_bounds[1], xmax)
        global_bounds[2] = min(global_bounds[2], ymin)
        global_bounds[3] = max(global_bounds[3], ymax)
        found_any = True

    if not found_any:
        return 0.0, 0.0

    xmin, xmax, ymin, ymax = global_bounds
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)

    return cx, cy

###

if __name__ == "__main__":
    datapath = os.path.abspath(os.path.join(config["TREES_DIR"], "urdf", "tree"))
    parent = os.path.join(os.path.dirname(datapath), "forest")
    os.makedirs(parent, exist_ok=True)

    for folder in sorted(os.listdir(datapath)):
        folder_path = os.path.join(datapath, folder)
        if not os.path.isdir(folder_path):
            continue

        urdfs = []
        for root, _, files in os.walk(folder_path):
            for f in files:
                if f.lower().endswith(".urdf"):
                    urdfs.append(os.path.join(root, f))

        urdf_files = sorted(urdfs)
        if not urdf_files:
            continue

        cx, cy = compute_urdf_xy_center(urdf_files)
        out_path = os.path.join(parent, f"{folder}.urdf")
        try:
            merged = ET.Element("robot", name="forest")

            ET.SubElement(merged, "link", name="base_link")
            used_names = set(["base_link"])

            ox, oy, oz = (cx, cy, 0)

            for idx, urdf_path in enumerate(urdf_files):
                tree = ET.parse(urdf_path)
                robot = tree.getroot()

                stem = os.path.splitext(os.path.basename(urdf_path))[0]
                prefix = f"{idx:04d}_{stem}"

                link_name_map = {}

                links = robot.findall("link")
                joints = robot.findall("joint")

                if not links:  continue

                root_link_old = get_root_link_name(robot)

                for link in links:
                    old_name = link.attrib["name"]
                    new_name = f"{prefix}__{old_name}"
                    while new_name in used_names:
                        new_name += "_x"
                    used_names.add(new_name)
                    link_name_map[old_name] = new_name

                    new_link = ET.fromstring(ET.tostring(link, encoding="unicode"))
                    new_link.attrib["name"] = new_name
                    merged.append(new_link)

                for joint in joints:
                    new_joint = ET.fromstring(ET.tostring(joint, encoding="unicode"))

                    old_joint_name = new_joint.attrib.get("name", "joint")
                    new_joint_name = f"{prefix}__{old_joint_name}"
                    while new_joint_name in used_names:
                        new_joint_name += "_x"
                    used_names.add(new_joint_name)
                    new_joint.attrib["name"] = new_joint_name

                    parent = new_joint.find("parent")
                    if parent is not None and "link" in parent.attrib:
                        parent.attrib["link"] = link_name_map[parent.attrib["link"]]

                    child = new_joint.find("child")
                    if child is not None and "link" in child.attrib:
                        child.attrib["link"] = link_name_map[child.attrib["link"]]

                    merged.append(new_joint)

                root_link_new = link_name_map[root_link_old]

                attach_joint = ET.SubElement(merged, "joint", name=f"{prefix}__attach_to_world", type="fixed")
                ET.SubElement(attach_joint, "parent", link="base_link")
                ET.SubElement(attach_joint, "child", link=root_link_new)

                # Center the whole scene here.
                ET.SubElement(attach_joint, "origin", xyz=f"{-ox:.8f} {-oy:.8f} {-oz:.8f}", rpy="0 0 0")

            tree = ET.ElementTree(merged)
            
            ET.indent(tree, space="  ", level=0)
            tree.write(out_path, encoding="utf-8", xml_declaration=True)

            print(f"OK: merged {len(urdf_files)} URDFs -> {out_path}")
            print(f"    scene center = ({cx:.6f}, {cy:.6f})")

        except Exception as e:
            print(f"FAILED: {folder_path}")
            print(f"  {e}")