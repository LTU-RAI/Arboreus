import os, yaml, random, time, math, json, rasterio, requests, numpy as np

from tqdm import tqdm
from pyproj import Transformer
from requests.auth import HTTPBasicAuth

from synthtree.synthtree import gen_simtree
from synthtree.ply import export_points_ply, export_xyz_ply

from ground import generate_base_ground
from utils import visualize_window

###

with open('config/params.yaml', 'r') as file:
    config = yaml.safe_load(file)

os.makedirs(os.path.join(config["TREES_DIR"], "ply"), exist_ok=True)

session = requests.Session()
session.auth = HTTPBasicAuth(config["LANTMATERIET_USERNAME"], config["LANTMATERIET_PASSWORD"])
session.headers.update({ "Accept": "application/json", })

###

def sample_random_tree_model(dominant_leaf_type=None, rng=None):
    if dominant_leaf_type is None:
        species = rng.choice(("pine", "birch", "spruce"), p=(config["TREE_PROB_PINE"], config["TREE_PROB_BIRCH"], config["TREE_PROB_SPRUCE"]))
    else:
        if dominant_leaf_type == 1:  species = "birch"
        else:                        species = "spruce" if random.random() <= config["TREE_PROB_SPRUCE"] else "pine"

    if species == "spruce":
        scale = config["TREE_SCALE_SPRUCE"]
        params = {
            "height_range": [12.1, 28.6], "diam_range": [0.13, 0.37],
            "num_branches": [160, 200], "min_can_height": [0.1, 0.1],
            "max_can_width": 10.3, "max_can_width_height": [0.0, 0.0],
            "tree_top_dist": 1.0, "tree_mid_dist": 0.5, "foliage_noise": 0.5,
        }

    elif species == "pine":
        scale = config["TREE_SCALE_PINE"]
        params = {
            "height_range": [11.8, 30.8], "diam_range": [0.14, 0.43],
            "num_branches": [80, 120], "min_can_height": [0.03, 0.03],
            "max_can_width": 7.14, "max_can_width_height": [0.5, 0.5],
            "tree_top_dist": 1.0, "tree_mid_dist": 0.5, "foliage_noise": 0.5,
        }

    else:
        scale = config["TREE_SCALE_BIRCH"]
        params = {
            "height_range": [14.5, 28.7], "diam_range": [0.12, 0.36],
            "num_branches": [250, 350], "min_can_height": [0.007, 0.007],
            "max_can_width": 8.25, "max_can_width_height": [0.5, 0.5],
            "tree_top_dist": 0.5, "tree_mid_dist": 0.0, "foliage_noise": 0.1,
        }

    return species, scale, params


def sample_grid_height(x, y, xs, ys, z_grid):
    col = int(np.clip(np.searchsorted(xs, x, side="right") - 1, 0, len(xs) - 2))
    row = int(np.clip(np.searchsorted(ys, y, side="right") - 1, 0, len(ys) - 2))

    tx = (x - xs[col]) / max(xs[col + 1] - xs[col], 1e-12)
    ty = (y - ys[row]) / max(ys[row + 1] - ys[row], 1e-12)
    
    return float(
        (1 - tx) * (1 - ty) * z_grid[row, col] + \
             tx  * (1 - ty) * z_grid[row, col + 1] + \
        (1 - tx) *      ty  * z_grid[row + 1, col] + \
             tx  *      ty  * z_grid[row + 1, col + 1]
    )


def generate_base_forest():
    if config["FOREST_SIZE_M2"] <= 0 or config["FOREST_DENSITY"] < 0 or config["FOREST_VARIANTS"] <= 0:
        raise ValueError("size_m2 and variants must be positive; density cannot be negative")

    os.makedirs(os.path.join(config["TREES_DIR"], "ply"), exist_ok=True)

    side = float(np.sqrt(config["FOREST_SIZE_M2"]))
    for rdx in range(config["FOREST_VARIANTS"]):
        variant_seed = config["SEED"] + rdx * 13
        rng = np.random.default_rng(variant_seed)

        variant_dir = os.path.join(config["TREES_DIR"], "ply", f"{rdx:02d}")
        os.makedirs(variant_dir, exist_ok=True)

        ground_points, xs, ys = generate_base_ground(side, side, spacing=config["GROUND_SPACING_M"], seed=variant_seed)
        export_xyz_ply(os.path.join(variant_dir, "ground.ply"), ground_points)

        tree_count = np.random.binomial(n=int(config["FOREST_DENSITY"] * config["FOREST_SIZE_M2"]), p=rng.uniform(0.0, 1.0))
        print(f"rdx {rdx:02d}: {tree_count} trees; ground has {len(ground_points)} points")

        z_grid = ground_points[:, 2].reshape(len(ys), len(xs))
        positions = rng.uniform(0.0, side, size=(tree_count, 2))

        for tree_id, (x, y) in enumerate(positions):
            species, scale, model_params = sample_random_tree_model(rng=rng)
            tree = np.asarray(gen_simtree(Np=config["TREE_POINTS"], model_params=model_params), dtype=np.float64) * scale

            if tree.ndim != 2 or tree.shape[1] < 3:
                raise ValueError("gen_simtree must return an Nx3 point array")

            tree = tree[:, :3].copy()
            tree[:, 2] -= float(tree[:, 2].min())
            tree[:, 0] += x
            tree[:, 1] += y

            ground_z = sample_grid_height(x, y, xs, ys, z_grid)
            tree[:, 2] += ground_z

            export_xyz_ply(os.path.join(variant_dir, f"{tree_id:04d}_{species}.ply"), tree)

###

def sample_lantmateriet_height(easting, northing):
    # EPSG:3006 -> Geotorget

    max_height_retries = 3
    height_retry_delay = 1.0

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

            time.sleep(height_retry_delay)

    return None


def sample_satellite_data(): 

    with rasterio.open(config["SATELLITE_TCD_TIF_PATH"]) as src:

        density = src.read(1).astype(np.float64)

        raster_crs = src.crs
        raster_transform = src.transform
        raster_nodata = src.nodata

        raster_width = src.width
        raster_height = src.height

        raster_resolution_x = abs(src.transform.a)
        raster_resolution_y = abs(src.transform.e)

    if raster_crs is None:
        raise RuntimeError("GeoTIFF does not contain CRS information.")

    # Mask valid pixels (bounded and existing)
    valid_mask = np.isfinite(density) 
    if raster_nodata is not None:
        valid_mask &= density != raster_nodata

    # Check if any pixel is valid
    valid_values = density[valid_mask]
    if valid_values.size == 0:
        raise RuntimeError("No valid Copernicus pixels found.")

    # Normalize density to 0-1 from 0-100
    density_normalized = density / 100.0 if np.max(valid_values) > 1.0 else density.copy()
    density_normalized = np.clip(density_normalized, 0.0, 1.0)
    density_normalized[~valid_mask] = 0.0  # Set non valid pixels to 0

    with rasterio.open(config["SATELLITE_DLT_TIF_PATH"]) as src:
        leaf_type = src.read(1).astype(np.float64)

    # Number of Copernicus pixels required to represent the requested forest size.
    window_cols = window_rows = int(np.ceil(math.sqrt(config["FOREST_SIZE_M2"] / (raster_resolution_x * raster_resolution_y))))

    if window_cols > raster_width:
        raise RuntimeError("FOREST_SIZE_M2 is wider than the raster.")
    if window_rows > raster_height:
        raise RuntimeError("FOREST_SIZE_M2 is taller than the raster.")

    # From EPSG:4326 to raster CRS EPSG:3005
    to_3035 = Transformer.from_crs("EPSG:4326", raster_crs, always_xy=True)

    nw_x, nw_y = to_3035.transform(config["SATELLITE_WEST_TARGET"], config["SATELLITE_NORTH_TARGET"])  # NW corner
    se_x, se_y = to_3035.transform(config["SATELLITE_EAST_TARGET"], config["SATELLITE_SOUTH_TARGET"])  # SE corner

    # Raster coordinates -> pixel coordinates
    row_min, col_min = rasterio.transform.rowcol(raster_transform, nw_x, nw_y)
    row_max, col_max = rasterio.transform.rowcol(raster_transform, se_x, se_y)

    return (raster_crs, raster_transform), (window_cols, window_rows), (row_min, col_min, row_max, col_max), density_normalized, leaf_type


def generate_satellite_forest():
    (raster_crs, raster_transform), (window_cols, window_rows), (row_min, col_min, row_max, col_max), density_normalized, leaf_type = sample_satellite_data()

    for rdx in range (config["FOREST_VARIANTS"]):
        os.makedirs(os.path.join(config["TREES_DIR"], "ply", f"{rdx:02d}"), exist_ok=True)

        variant_seed = config["SEED"] + rdx * 13
        random.seed(variant_seed); np.random.seed(variant_seed)

        window_density = window_leaf_type = None
        for attempt in range(1000):

            start_row = random.randint(row_min, row_max - window_rows)
            start_col = random.randint(col_min, col_max - window_cols)

            candidate = density_normalized[
                start_row:start_row + window_rows,
                start_col:start_col + window_cols
            ]

            # Reject windows containing no tree cover.
            if candidate.sum() <= 0:
                continue

            window_density = candidate
            window_leaf_type = leaf_type[
                start_row:start_row + window_rows,
                start_col:start_col + window_cols
            ]
            
            break

        if window_density is None:
            raise RuntimeError("Could not find a forest window containing tree cover.")

        total_weight = window_density.copy().ravel().sum()
        if total_weight <= 0:
            raise RuntimeError("Selected forest window contains no tree density.")

        # Get all four window corners in the TIFF's coordinate system.
        corner_pixels = [
            (start_col, start_row),
            (start_col + window_cols, start_row),
            (start_col, start_row + window_rows),
            (start_col + window_cols, start_row + window_rows),
        ]

        # From raster_crs EPSG:3005 to EPSG:3006
        to_3006 = Transformer.from_crs(raster_crs, "EPSG:3006", always_xy=True)
        corner_3006 = []

        for col, row in corner_pixels:
            raster_x, raster_y = raster_transform * (col, row)
            e, n = to_3006.transform(raster_x, raster_y)
            corner_3006.append((e, n))

        corner_e = np.array([p[0] for p in corner_3006])
        corner_n = np.array([p[1] for p in corner_3006])

        forest_min_e = float(corner_e.min())
        forest_max_e = float(corner_e.max())

        forest_min_n = float(corner_n.min())
        forest_max_n = float(corner_n.max())

        # The lower bound of the selected FOREST WINDOW becomes zero.
        origin_e = forest_min_e
        origin_n = forest_min_n

        with open(os.path.join(config["TREES_DIR"], "ply", f"{rdx:02d}", "origin.json"), "w", encoding="utf-8") as file:
            json.dump({"origin_e": origin_e, "origin_n": origin_n}, file, ensure_ascii=False, indent=4)

        # Tree generation 

        # Generate tree locations from per-pixel density
        tree_id = 0
        ground_world_points = []
        for local_row in range(window_rows):
            for local_col in range(window_cols):

                density_value = float(window_density[local_row, local_col])
                if density_value <= 0.0:
                    continue

                leaf_type_value = float(window_leaf_type[local_row, local_col])

                # Determine number of trees in THIS 10x10 m pixel.
                # Binomial keeps the absolute maximum at 10.
                num_trees = np.random.binomial(n=int(config["FOREST_DENSITY"] * 100), p=density_value)
                if num_trees == 0:
                    continue

                # Position every generated tree independently within this Copernicus 10x10 pixel.
                raster_row = (start_row + local_row)
                raster_col = (start_col + local_col)

                for _ in range(num_trees):

                    # Random location inside this 10x10 m pixel.
                    subpixel_col = (raster_col + random.random())
                    subpixel_row = (raster_row + random.random())

                    # Pixel coordinate -> GeoTIFF CRS.
                    raster_x, raster_y = raster_transform * (subpixel_col, subpixel_row)

                    # GeoTIFF CRS -> EPSG:3006.
                    easting, northing = to_3006.transform(raster_x, raster_y)

                    # Every individual tree receives its own Lantmateriet ground elevation.
                    ground_height = sample_lantmateriet_height(easting, northing)

                    if ground_height is None:
                        continue

                    ground_world_points.append([easting, northing, ground_height])

                    species, scale, model_params = (sample_random_tree_model(dominant_leaf_type=leaf_type_value))

                    # Generate synthetic tree.
                    points = np.asarray(gen_simtree(Np=config["TREE_POINTS"], model_params=model_params), dtype=np.float64) * scale

                    # Lowest point of THIS tree becomes zero before terrain elevation is applied.
                    points[:, 2] -= points[:, 2].min()

                    # Set local coordinate in meters
                    points[:, 0] += (float(easting) - origin_e) 
                    points[:, 1] += (float(northing) - origin_n)
                    points[:, 2] += (float(ground_height))  # - origin_z

                    filename = os.path.join(config["TREES_DIR"], "ply", f"{rdx:02d}", f"{tree_id:03d}_{species}.ply")
                    export_points_ply(filename, points)
                    tree_id += 1

        # Ground generation
        ground_world_points = np.asarray(ground_world_points, dtype=np.float64)
        if len(ground_world_points) == 0:
            raise RuntimeError("No valid Lantmateriet terrain points obtained.")

        # Convert ground to local coordinates
        ground_points = ground_world_points.copy()

        ground_points[:, 0] -= origin_e
        ground_points[:, 1] -= origin_n

        export_xyz_ply(os.path.join(config["TREES_DIR"], "ply", f"{rdx:02d}", "ground.ply"), ground_points)

        to_4326 = Transformer.from_crs("EPSG:3006", "EPSG:4326", always_xy=True)
        origin_lon, origin_lat = to_4326.transform(origin_e, origin_n)

        forest_size_m2 = config["FOREST_SIZE_M2"]

        print()
        print("Forest:")
        print(f"  requested size: {forest_size_m2:.1f} m2")
        print(f"  actual EPSG:3006 bounds: {forest_max_e - forest_min_e:.2f} x {forest_max_n - forest_min_n:.2f} m")

        print()
        print("World origin:")
        print(f"  LON = {origin_lon:.4f}")
        print(f"  LAT = {origin_lat:.4f}")

        print()
        print("Local ground bounds:")
        print(f"  X: {ground_points[:, 0].min():.2f} -> {ground_points[:, 0].max():.2f} m")
        print(f"  Y: {ground_points[:, 1].min():.2f} -> {ground_points[:, 1].max():.2f} m")
        print(f"  Z: {ground_points[:, 2].min():.2f} -> {ground_points[:, 2].max():.2f} m")

        visualize_window(rdx, origin_lon, origin_lat, forest_max_e - forest_min_e, forest_max_n - forest_min_n)

###

if __name__ == "__main__":
    if config["FOREST_TYPE"] == "base":  
        generate_base_forest()
    elif config["FOREST_TYPE"] == "satellite":          
        generate_satellite_forest()
    else:
        print("The specified forest type does not exist.")
        exit(0)