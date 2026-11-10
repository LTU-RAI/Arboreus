
import os, yaml, math, numpy as np, open3d as o3d
import matplotlib.pyplot as plt

from pathlib import Path
from scipy.spatial import Delaunay, QhullError

###

with open('config/params.yaml', 'r') as file:
    config = yaml.safe_load(file)

def natural_key(path: Path):
    try:                 return (0, int(path.stem))
    except ValueError:   return (1, path.name.lower())


def tetra_volumes(points: np.ndarray, tetrahedra: np.ndarray) -> np.ndarray:
    p = points[tetrahedra]
    return np.abs(np.linalg.det(np.stack((p[:, 1] - p[:, 0], p[:, 2] - p[:, 0], p[:, 3] - p[:, 0]), axis=1))) / 6.0


def boundary_mesh(points: np.ndarray, tetrahedra: np.ndarray) -> o3d.geometry.TriangleMesh:
    faces = np.vstack((tetrahedra[:, [0, 1, 2]], tetrahedra[:, [0, 1, 3]], tetrahedra[:, [0, 2, 3]], tetrahedra[:, [1, 2, 3]]))
    ordered = np.sort(faces, axis=1)

    _, first, counts = np.unique(ordered, axis=0, return_index=True, return_counts=True)
    surface = faces[first[counts == 1]]
    
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(points), o3d.utility.Vector3iVector(surface))
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()

    return mesh


def measure(path: Path):
    cloud = o3d.io.read_point_cloud(str(path))
    cloud.remove_non_finite_points()

    points = np.asarray(cloud.points)
    if len(points) < 4:
        raise ValueError("fewer than four valid points")
    
    extent = np.ptp(points, axis=0)
    width = float(extent[0])
    height = float(extent[2])

    try:
        delaunay = Delaunay(points, qhull_options="QJ Qbb Qc")
    except QhullError as exc:
        raise ValueError(f"Delaunay failed: {exc}") from exc

    tetrahedra = delaunay.simplices
    volumes = tetra_volumes(points, tetrahedra)
    valid = volumes > np.finfo(float).eps
    method = "delaunay_convex_hull"

    kept = tetrahedra[valid]
    volume = float(volumes[valid].sum())

    return width, height, volume, len(points), len(kept), method

###

if __name__ == "__main__":
    folders = []
    for rdx in range(config["FOREST_VARIANTS"]):
        folders.append(Path(os.path.join(config["TREES_DIR"], "ply", f"{rdx:02d}")))

    labels = ("Pine", "Spruce", "Birch")
    all_rows = [];  limit = 60
    for folder_number, (folder, label) in enumerate(zip(folders, labels), 1):
        files = sorted(folder.glob("*.ply"), key=natural_key)
        if not files:
            parser.error(f"no files matching .ply in {folder}")

        files = files[:limit]

        print(f"\nFolder {folder_number}/3: {folder} ({label})")
        rows = []
        
        max_height_id = ""
        max_height = -math.inf
        
        min_height_id = ""
        min_height = math.inf

        for index, path in enumerate(files, 1):
            print(f"[{index}/{len(files)}] {path.name}", flush=True)
            try:
                width, height, volume, points, tetrahedra, method = measure(path)
                
                if height > max_height: max_height = height; max_height_id = path
                if height < min_height: min_height = height; min_height_id = path

            except Exception as exc:
                print(f"  SKIPPED: {exc}")
                continue

            rows.append((path.name, width, height, volume, points, tetrahedra, method))
            print(f"  width = {width:.9g}; height = {height:.9g}; volume = {volume:.9g}")

        print(f"{label}: max_height({max_height}) max_height_id({max_height_id}) min_height({min_height}) min_height_id({min_height_id})")

        if not rows:
            raise SystemExit(f"No point clouds could be measured in {folder}.")
        all_rows.append(rows)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    colors = ("tab:blue", "tab:orange", "tab:green")
    for label, rows, color in zip(labels, all_rows, colors):
        widths = [row[1] for row in rows]
        heights = [row[2] for row in rows]
        volumes = [row[3] for row in rows]
        ax.scatter(widths, heights, volumes, s=38, alpha=0.75, color=color, label=label, depthshade=True)

    ax.set_xlabel(f"Width (m)")
    ax.set_ylabel(f"Height (m)")
    ax.set_zlabel("Volume (m³)")
    ax.legend()
    
    fig.tight_layout()
    fig.savefig("imgs/synthtree.png", dpi=200)
    plt.close(fig)
