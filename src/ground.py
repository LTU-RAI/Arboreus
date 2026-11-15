import os, yaml, argparse, numpy as np, open3d as o3d

###

with open('config/params.yaml', 'r') as file:
    config = yaml.safe_load(file)

def sample_rectangular_boundary(pcd, boundary=2.0, spacing=0.25, z_neighbors=8):
    points = np.asarray(pcd.points)

    min_x, min_y = points[:, :2].min(axis=0)
    max_x, max_y = points[:, :2].max(axis=0)

    # Expanded rectangular bounds
    x0 = min_x - boundary
    x1 = max_x + boundary
    y0 = min_y - boundary
    y1 = max_y + boundary

    xs = np.arange(x0, x1 + spacing, spacing)
    ys = np.arange(y0, y1 + spacing, spacing)

    # Generate points over the entire added rectangular area, excluding points already inside the original bounding rectangle.
    xx, yy = np.meshgrid(xs, ys)
    xy = np.column_stack([xx.ravel(), yy.ravel()])

    boundary_xy = xy[((xy[:, 0] < min_x) | (xy[:, 0] > max_x) | (xy[:, 1] < min_y) | (xy[:, 1] > max_y))]
    print(f"Adding {len(boundary_xy)} boundary points")

    # KD tree containing original points
    tree = o3d.geometry.KDTreeFlann(pcd)

    boundary_points = []

    # Estimate Z from nearby original ground points
    for x, y in boundary_xy:
        # Start the query at median terrain height.
        query_z = np.median(points[:, 2])

        query = np.array([x, y, query_z], dtype=np.float64)

        _, idx, dist2 = tree.search_knn_vector_3d(query, z_neighbors)

        if len(idx) == 0:
            z = query_z
        else:
            neighbor_points = points[idx]

            # Inverse-distance weighted Z
            distances = np.sqrt(np.asarray(dist2))
            weights = 1.0 / np.maximum(distances, 0.001)

            z = np.average(neighbor_points[:, 2], weights=weights)

        boundary_points.append([x, y, z])

    boundary_points = np.asarray(boundary_points)

    # Combine original and generated points
    all_points = np.vstack([points, boundary_points])

    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(all_points)

    return result, (x0, x1, y0, y1)


def densify_point_cloud(pcd, spacing=0.20, k=4, max_distance=1.0):
    points = np.asarray(pcd.points)

    xy = points[:, :2]
    z = points[:, 2]

    min_x, min_y = xy.min(axis=0)
    max_x, max_y = xy.max(axis=0)

    # KD tree in XY only. Open3D's KDTree works in 3D, so create another cloud where Z=0.
    xy_points_3d = np.column_stack([xy, np.zeros(len(xy))])

    xy_pcd = o3d.geometry.PointCloud()
    xy_pcd.points = o3d.utility.Vector3dVector(xy_points_3d)

    tree = o3d.geometry.KDTreeFlann(xy_pcd)

    xs = np.arange(min_x, max_x + spacing, spacing)
    ys = np.arange(min_y, max_y + spacing, spacing)

    new_points = []

    for y in ys:
        for x in xs:

            query = np.array([x, y, 0.0])
            count, idx, dist2 = tree.search_knn_vector_3d(query, k)

            if count == 0:
                continue

            distances = np.sqrt(np.asarray(dist2))

            # Don't fill areas that are too far from real data
            if distances[0] > max_distance:
                continue

            neighbor_z = z[np.asarray(idx)]

            # If we're almost exactly on an existing point, just use its Z.
            if distances[0] < 1e-6:
                interpolated_z = neighbor_z[0]
            else:
                # Inverse distance weighting
                weights = 1.0 / np.maximum(distances, 1e-6) ** 2
                interpolated_z = np.sum(weights * neighbor_z) / np.sum(weights)

            new_points.append([x, y, interpolated_z])

    if not new_points:
        return pcd

    new_points = np.asarray(new_points)

    # Keep original points AND interpolated points
    combined = np.vstack([points, new_points])

    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(combined)

    print(f"Densification: {len(points)} original + {len(new_points)} interpolated = {len(combined)} points")

    return result


def generate_base_ground(width, height, spacing=0.5, seed=None):
    if width <= 0 or height <= 0 or spacing <= 0:
        raise ValueError("width, height, and spacing must be positive")
    if not 0.0 <= config["GROUND_FILTER_PROBABILITY"] <= 1.0:
        raise ValueError("application_probability must be between 0 and 1")

    def _smooth_patch_window(axis_values, fade_fraction):
        """Return a smooth 1D envelope that reaches zero at both patch edges."""
        span = float(axis_values[-1] - axis_values[0])
        if span <= 0:
            return np.ones_like(axis_values, dtype=np.float64)

        distance_to_edge = np.minimum(axis_values - axis_values[0], axis_values[-1] - axis_values)
        fade_distance = max(span * fade_fraction, np.finfo(np.float64).eps)
        t = np.clip(distance_to_edge / fade_distance, 0.0, 1.0)

        # Quintic smoothstep has zero slope at both ends, avoiding visible creases.
        return t**3 * (t * (t * 6.0 - 15.0) + 10.0)

    rng = np.random.default_rng(seed)
    
    xs = np.arange(0.0, width + spacing * 0.5, spacing)
    ys = np.arange(0.0, height + spacing * 0.5, spacing)

    xx, yy = np.meshgrid(xs, ys)
    z = np.zeros_like(xx, dtype=np.float64)

    for fraction_x, fraction_y in config["GROUND_FILTERS"]:
        patch_cols = max(1, min(len(xs), int(round(fraction_x * (len(xs) - 1))) + 1))
        patch_rows = max(1, min(len(ys), int(round(fraction_y * (len(ys) - 1))) + 1))
        stride_cols, stride_rows = patch_cols, patch_rows

        for row0 in range(0, len(ys), stride_rows):
            for col0 in range(0, len(xs), stride_cols):
                row1 = min(row0 + patch_rows, len(ys))
                col1 = min(col0 + patch_cols, len(xs))

                if rng.random() <= config["GROUND_FILTER_PROBABILITY"]:
                    continue

                patch_x = xx[row0:row1, col0:col1]
                patch_y = yy[row0:row1, col0:col1]

                window_x = _smooth_patch_window(xs[col0:col1], config["GROUND_EDGE_FADE_FRACTION"])
                window_y = _smooth_patch_window(ys[row0:row1], config["GROUND_EDGE_FADE_FRACTION"])
                window = window_y[:, None] * window_x[None, :]

                angle = np.deg2rad(rng.uniform(*config["GROUND_ANGLE_RANGE"]))
                axis = patch_x if rng.random() < 0.5 else patch_y
                axis_center = 0.5 * (float(axis.min()) + float(axis.max()))
                z[row0:row1, col0:col1] += ((axis - axis_center) * np.tan(angle) * window)

    points = np.column_stack((xx.ravel(), yy.ravel(), z.ravel()))
    return points, xs, ys

###

if __name__ == "__main__":
    parent = os.path.join(config["TREES_DIR"], "ground")
    if not os.path.exists(parent):  os.mkdir(parent)

    for rdx in range(config["FOREST_VARIANTS"]):

        pcd = o3d.io.read_point_cloud(os.path.join(config["TREES_DIR"], "ply", f"{rdx:02d}", "ground.ply"))
        if len(pcd.points) == 0:
            raise ValueError("PLY contains no points.")
        print(f"Original points: {len(pcd.points)}")

        # Fill gaps between original points
        pcd = densify_point_cloud(pcd, spacing=0.20, k=4, max_distance=50.0)

        # Add the rectangular 2-meter boundary
        pcd, bounds = sample_rectangular_boundary(pcd, boundary=2.0, spacing=0.20)
        x0, x1, y0, y1 = bounds

        print(f"Points with boundary: {len(pcd.points)}")
        print(f"Output XY bounds: X={x0:.2f}..{x1:.2f}, Y={y0:.2f}..{y1:.2f}")

        # Optional voxel downsample
        if config["GROUND_VOXEL_SIZE"] is not None and config["GROUND_VOXEL_SIZE"] > 0:
            pcd = pcd.voxel_down_sample(config["GROUND_VOXEL_SIZE"])

        # Normals
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=50))
        normals = np.asarray(pcd.normals)

        flip = normals[:, 2] < 0
        normals[flip] *= -1

        pcd.normals = o3d.utility.Vector3dVector(normals)

        # Poisson reconstruction
        mesh, densities = (o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9))
        densities = np.asarray(densities)

        threshold = np.quantile(densities, 0.02)
        mesh.remove_vertices_by_mask(densities < threshold)

        # Crop exactly to expanded rectangular XY bounds
        points = np.asarray(pcd.points)

        min_z = points[:, 2].min() - 10.0
        max_z = points[:, 2].max() + 10.0

        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=np.array([x0, y0, min_z]), max_bound=np.array([x1, y1, max_z]))
        mesh = mesh.crop(bbox)

        # Cleanup
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        mesh.remove_duplicated_vertices()
        mesh.remove_non_manifold_edges()

        mesh.compute_vertex_normals()

        print(f"Mesh: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles")

        success = o3d.io.write_triangle_mesh(os.path.join(parent, f"{rdx:02d}.obj"), mesh, write_vertex_normals=True)
        if not success:
            raise RuntimeError("Failed to write OBJ.")