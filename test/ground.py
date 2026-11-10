import os, time, json, csv, yaml, requests, open3d as o3d, numpy as np

import matplotlib, matplotlib.pyplot as plt
matplotlib.use("Agg")

from requests.auth import HTTPBasicAuth

###

with open('config/params.yaml', 'r') as file:
    config = yaml.safe_load(file)

session = requests.Session()
session.auth = HTTPBasicAuth(config["LANTMATERIET_USERNAME"], config["LANTMATERIET_PASSWORD"])
session.headers.update({ "Accept": "application/json", })

def sample_lantmateriet_height(easting, northing):
    # EPSG:3006 -> Geotorget

    max_height_retries = 3

    params = {
        "srid": 3006,
        "e": float(easting),
        "n": float(northing),
    }

    for attempt in range(max_height_retries):
        try:
            response = session.get(config["LANTMATERIET_URL"], params=params, timeout=30)
            response.raise_for_status()

            data = response.json()
            height = data["geometry"]["coordinates"][2]
            nodata = data["properties"]["nodatavalue"]

            if height == nodata:  return None

            return float(height)

        except (requests.RequestException, KeyError, TypeError, ValueError) as exc:

            if attempt == max_height_retries - 1:
                print("\nHeight lookup failed at E={easting:.3f}, N={northing:.3f}: {exc}")
                return None

            time.sleep(0.1)

    return None


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

    return result, new_points


def validate_interpolated_points(interpolated_points, origin_e, origin_n, output_csv, request_delay=0.0, max_points=None):
    candidates = interpolated_points
    if max_points is not None and max_points > 0 and len(candidates) > max_points:
        # Evenly spread validation samples over the generated grid
        rng = np.random.default_rng(42)
        candidates = candidates[rng.choice(len(candidates), size=max_points, replace=False)]

    rows, errors = [], []
    print(f"Validating {len(candidates)} interpolated points against Lantmateriet")

    for number, (local_x, local_y, interpolated_z) in enumerate(candidates, start=1):
        easting = origin_e + local_x
        northing = origin_n + local_y
        api_z = sample_lantmateriet_height(easting, northing)

        if api_z is None:
            signed_error = absolute_error = None
        else:
            signed_error = interpolated_z - api_z
            absolute_error = abs(signed_error)
            errors.append(signed_error)

        rows.append((local_x, local_y, easting, northing, interpolated_z, api_z, signed_error, absolute_error))

        if number % 100 == 0 or number == len(candidates):
            print(f"  API validation: {number}/{len(candidates)}")
        if request_delay > 0:
            time.sleep(request_delay)

    os.makedirs(os.path.dirname(os.path.abspath(output_csv)), exist_ok=True)
    with open(output_csv, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["local_x", "local_y", "easting_epsg3006", "northing_epsg3006", "interpolated_z", "api_z", "signed_error_m", "absolute_error_m"])
        writer.writerows(rows)

    if not errors:
        print(f"No valid API heights returned; wrote: {output_csv}")
        return None

    errors = np.asarray(errors, dtype=np.float64)
    valid_rows = np.asarray([row for row in rows if row[6] is not None])
    rmse = float(np.sqrt(np.mean(errors ** 2)))

    print(f"Validation: valid={len(errors)}/{len(candidates)}, MAE={np.mean(np.abs(errors)):.4f} m, RMSE={rmse:.4f} m, bias={np.mean(errors):.4f} m, max_abs={np.max(np.abs(errors)):.4f} m")
    print(f"Saved validation rows: {output_csv}")

    return {"rmse": rmse, "x": valid_rows[:, 0], "y": valid_rows[:, 1], "error": valid_rows[:, 6]}


def plot_validation_results(results):
    if not results:
        print("No valid results to plot.");  return

    rmse_values = [result["rmse"] for result in results]

    fig, ax = plt.subplots(figsize=(5, 6))
    
    ax.boxplot(rmse_values, labels=["All grounds"], showmeans=True)
    ax.scatter(np.ones(len(rmse_values)), rmse_values, alpha=0.7)
    ax.set_ylabel("RMSE (m)")
    ax.set_title("Interpolation RMSE")
    ax.grid(axis="y", alpha=0.25)
    
    fig.tight_layout()
    fig.savefig(os.path.join("imgs", "ground_rmse_boxplot.png"), dpi=200)
    plt.close(fig)

    shown = results[:4]
    error_limit = max(np.max(np.abs(result["error"])) for result in shown)

    if error_limit == 0:  error_limit = 1e-9

    fig, axes = plt.subplots(2, 2, figsize=(15, 9))
    fig.subplots_adjust(right=0.82, hspace=0.30, wspace=0.22)
    axes = axes.ravel()

    height_map = error_points = None
    for ax, result in zip(axes, shown):
        ground = result["ground"]

        height_map = ax.scatter(ground[:, 0], ground[:, 1], c=ground[:, 2], cmap="viridis", marker="s", s=24, linewidths=0)
        height_colorbar = fig.colorbar(height_map, ax=ax, fraction=0.046, pad=0.04)
        height_colorbar.set_label("Ground height (m)")
        
        error_points = ax.scatter(result["x"], result["y"], c=result["error"], cmap="RdBu_r", vmin=-error_limit, vmax=error_limit, s=38, edgecolors="black", linewidths=0.35)
        
        ax.set_title(f"RMSE {result['rmse']:.3f} m")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.set_aspect("equal", adjustable="box")
        ax.set_box_aspect(1)

    for ax in axes[len(shown):]:
        ax.set_visible(False)

    error_cax = fig.add_axes([0.86, 0.22, 0.018, 0.56])
    error_colorbar = fig.colorbar(error_points, cax=error_cax)
    error_colorbar.set_label("Signed error: interpolated API (m)")

    fig.savefig(os.path.join("imgs", "ground_rmse_map.png"), dpi=200)
    plt.close(fig)

###

if __name__ == "__main__":
    parent = os.path.join(config["TREES_DIR"], "ground")
    if not os.path.exists(parent):  os.mkdir(parent)

    validation_results = []

    for rdx in range(config["FOREST_VARIANTS"]):
        input_ply = os.path.join(config["TREES_DIR"], "ply", f"{rdx:02d}", "ground.ply")
        origin_path = os.path.join(os.path.dirname(input_ply), "origin.json")

        try:
            with open(origin_path, "r", encoding="utf-8") as origin_file:
                metadata = json.load(origin_file)
            
            origin_e, origin_n = float(metadata["origin_e"]), float(metadata["origin_n"])

        except FileNotFoundError as exc:
            raise FileNotFoundError(f"Missing {origin_path}") from exc

        pcd = o3d.io.read_point_cloud(input_ply)

        if len(pcd.points) == 0:
            raise ValueError("PLY contains no points.")

        print(f"Original points: {len(pcd.points)}")

        # Fill gaps between original points
        pcd, interpolated_points = densify_point_cloud(pcd, spacing=0.20, k=4, max_distance=5.0)

        validation = {
            "origin_e": origin_e,
            "origin_n": origin_n,
            "output_csv": os.path.join(parent, f"{rdx:02d}_validation.csv"),
            "max_points": 100,
        }

        result = validate_interpolated_points(interpolated_points, **validation)
        if result is not None:
            result["ground_id"] = rdx
            result["ground"] = np.asarray(pcd.points).copy()
            validation_results.append(result)

    plot_validation_results(validation_results)
