#!/usr/bin/env python3
import os, yaml, math, numpy as np
import xml.etree.ElementTree as ET

from plyfile import PlyData
from collections import defaultdict

###

with open('config/params.yaml', 'r') as file:
    config = yaml.safe_load(file)

def normalize(v, eps=1e-12):
    n = np.linalg.norm(v)
    if n < eps:
        return v, 0.0
    return v / n, n


def rpy_rotation_z(direction):
    direction, n = normalize(np.asarray(direction, dtype=float))
    if n == 0.0:
        return np.eye(3)

    z = np.array([0.0, 0.0, 1.0], dtype=float)
    c = float(np.clip(np.dot(z, direction), -1.0, 1.0))

    if abs(c - 1.0) < 1e-10:
        return np.eye(3)

    if abs(c + 1.0) < 1e-10:
        return np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ])

    axis = np.cross(z, direction)
    axis, _ = normalize(axis)
    x, y, zc = axis
    s = math.sqrt(1.0 - c * c)
    C = 1.0 - c

    R = np.array([
        [x * x * C + c,      x * y * C - zc * s, x * zc * C + y * s],
        [y * x * C + zc * s,  y * y * C + c,     y * zc * C - x * s],
        [zc * x * C - y * s,  zc * y * C + x * s, zc * zc * C + c],
    ], dtype=float)

    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-12

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return roll, pitch, yaw

###s

def edge_key(a, b):
    return (a, b) if a < b else (b, a)


def build_adjacency(edges):
    adj = defaultdict(set)
    for a, b in edges:
        adj[a].add(b)
        adj[b].add(a)
    return adj


def extract_segments(edges):
    adj = build_adjacency(edges)
    visited = set()
    segments = []

    special_nodes = [n for n in adj.keys() if len(adj[n]) != 2]

    def walk_path(start, nxt):
        path = [start, nxt]
        visited.add(edge_key(start, nxt))

        prev = start
        curr = nxt

        while len(adj[curr]) == 2:
            candidates = [x for x in adj[curr] if x != prev]
            if not candidates:
                break

            nxt2 = candidates[0]
            ek = edge_key(curr, nxt2)
            if ek in visited:
                break

            path.append(nxt2)
            visited.add(ek)
            prev, curr = curr, nxt2

        return path

    # Paths that start from leaves or junctions
    for s in special_nodes:
        for n in adj[s]:
            ek = edge_key(s, n)
            if ek not in visited:
                segments.append(walk_path(s, n))

    # Catch any leftover edges, including cycles
    for a, b in edges:
        ek = edge_key(a, b)
        if ek not in visited:
            segments.append(walk_path(a, b))

    return segments


def fit_pca_segment(points, segment):
    pts = points[np.array(segment, dtype=int)]
    if len(pts) == 1:
        return pts[0], pts[0], np.array([0.0, 0.0, 1.0], dtype=float)

    centroid = np.mean(pts, axis=0)
    X = pts - centroid

    if len(pts) == 2:
        axis = pts[1] - pts[0]
        axis, _ = normalize(axis)
        t = X @ axis
    else:
        cov = (X.T @ X) / max(len(pts), 1)
        vals, vecs = np.linalg.eigh(cov)
        axis = vecs[:, int(np.argmax(vals))]
        axis, _ = normalize(axis)
        t = X @ axis

    t_min = float(np.min(t))
    t_max = float(np.max(t))

    p1 = centroid + axis * t_min
    p2 = centroid + axis * t_max

    return p1, p2, axis

###

def add_cylinder(robot, name, p1, p2, radius):
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)

    d = p2 - p1
    length = float(np.linalg.norm(d))
    if length < 1e-10:  return

    midpoint = (p1 + p2) * 0.5
    rpy = rpy_rotation_z(d)

    link = ET.SubElement(robot, "link", name=name)

    for tag in ("visual", "collision"):
        elem = ET.SubElement(link, tag)
        ET.SubElement(elem, "origin", xyz="0 0 0", rpy="0 0 0")

        geom = ET.SubElement(elem, "geometry")
        scale_length = length * config["TREE_URDF_SCALE"]
        scale_radius = radius * config["TREE_URDF_SCALE"]
        ET.SubElement(geom, "cylinder", length=f"{scale_length:.8f}", radius=f"{scale_radius:.8f}")

    joint = ET.SubElement(robot, "joint", name=f"{name}_joint", type="fixed")
    ET.SubElement(joint, "parent", link="base_link")
    ET.SubElement(joint, "child", link=name)
    midpoint = midpoint * config["TREE_URDF_SCALE"]
    ET.SubElement(joint, "origin", xyz=f"{midpoint[0]:.8f} {midpoint[1]:.8f} {midpoint[2]:.8f}", rpy=f"{rpy[0]:.8f} {rpy[1]:.8f} {rpy[2]:.8f}")

###

if __name__ == "__main__":
    datapath = os.path.abspath(os.path.join(config["TREES_DIR"], "adtree"))
    parent = os.path.join(os.path.dirname(datapath), "urdf", "tree")
    os.makedirs(parent, exist_ok=True)

    for group_name in sorted(os.listdir(datapath)):
        group_dir = os.path.join(datapath, group_name)
        if not os.path.isdir(group_dir):
            continue

        out_group_dir = os.path.join(parent, group_name)
        os.makedirs(out_group_dir, exist_ok=True)

        nested_dirs = [d for d in sorted(os.listdir(group_dir)) if os.path.isdir(os.path.join(group_dir, d))]
        processed_any = False

        for sample_name in nested_dirs:
            sample_dir = os.path.join(group_dir, sample_name)

            ply_files = [f for f in os.listdir(sample_dir) if f.lower().endswith(".ply")]
            if not ply_files:  continue

            skeleton_plys = [f for f in ply_files if f.lower().endswith("_skeleton.ply")]
            if skeleton_plys:
                ply_path = os.path.join(sample_dir, sorted(skeleton_plys)[0])

            ply_path = os.path.join(sample_dir, sorted(ply_files)[0])
            if ply_path is None:  continue

            out_path = os.path.join(out_group_dir, f"{sample_name}.urdf")
            try:
                ply = PlyData.read(ply_path)

                v = ply["vertex"].data
                points = np.stack([np.asarray(v["x"], dtype=float), np.asarray(v["y"], dtype=float), np.asarray(v["z"], dtype=float)], axis=1)

                radii = None
                if "radius" in v.dtype.names:
                    radii = np.asarray(v["radius"], dtype=float)

                e = ply["edge"].data
                edges = []
                for row in e:
                    idx = row["vertex_indices"]
                    if len(idx) != 2:
                        raise ValueError("Each edge must contain exactly two vertex indices.")
                    edges.append((int(idx[0]), int(idx[1])))

                segments = extract_segments(edges)

                robot = ET.Element("robot", name="tree_skeleton")
                ET.SubElement(robot, "link", name="base_link")

                for i, segment in enumerate(segments):
                    p1, p2, _axis = fit_pca_segment(points, segment)

                    if radii is not None:
                        seg_radii = radii[np.array(segment, dtype=int)]
                        radius = float(np.max(seg_radii))
                    else:
                        radius = float(config["TREE_URDF_RADIUS"])

                    add_cylinder(robot, f"branch_{i:05d}", p1, p2, radius)

                tree = ET.ElementTree(robot)
                ET.indent(tree, space="  ", level=0)
                tree.write(out_path, encoding="utf-8", xml_declaration=True)

                print(f"OK: {ply_path} -> {out_path}")
                processed_any = True
                
            except Exception as e:
                print(f"FAILED: {ply_path}")
                print(f"  {e}")