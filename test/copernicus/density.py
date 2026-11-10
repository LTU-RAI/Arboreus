import csv, yaml, math, rasterio, matplotlib.pyplot as plt

from pyproj import Transformer
from matplotlib.colors import Normalize

from rasterio.warp import transform
from rasterio.windows import from_bounds

###

with open('config/params.yaml', 'r') as file:
    config = yaml.safe_load(file)

def load_points():
    with open("data/mjölkuddsberget.csv", newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)

        available_columns = set(reader.fieldnames or [])
        required_columns = {"i", "LAT", "LON", "Nij", "Hij", "Gij_1", "Gij_2", "Gij_3"}

        if not required_columns.issubset(available_columns):
            missing = sorted(required_columns - available_columns)
            raise ValueError(f"Missing CSV columns: {', '.join(missing)}")

        rows = list(reader)

    if not rows:
        raise ValueError(f"No points found in CSV path")

    data = {
        "ids": [row["i"] for row in rows],
        "latitudes": [float(row["LAT"]) for row in rows],
        "longitudes": [float(row["LON"]) for row in rows],
        "Nij": [float(row["Nij"]) for row in rows],
        "Hij": [],
        "Gij_1": [float(row["Gij_1"]) for row in rows],
        "Gij_2": [float(row["Gij_2"]) for row in rows],
        "Gij_3": [float(row["Gij_3"]) for row in rows],
    }

    n_max_trees = 30  # 0.1662 RMSE

    # Sum the first two Gij columns for the second plot.
    data["Gij_1_2_sum"] = [first + second for first, second in zip(data["Gij_1"], data["Gij_2"])]
    data["Hij"] = [min(nij / n_max_trees, 1.0) for nij in data["Nij"]]

    return data


def sample_satellite_target(tif_path):
    with rasterio.open(tif_path) as dataset:
        raster_crs = dataset.crs

        xs, ys = transform("EPSG:4326", raster_crs, \
                            [config["SATELLITE_WEST_TARGET"], config["SATELLITE_EAST_TARGET"]], \
                            [config["SATELLITE_NORTH_TARGET"], config["SATELLITE_SOUTH_TARGET"]])

        x_west, x_east = xs
        y_north, y_south = ys

        display_window = from_bounds(x_west, y_south, x_east, y_north, transform=dataset.transform)
        display_window = (display_window.round_offsets().round_lengths())

        subset = dataset.read(1, window=display_window)
        subset_bounds = rasterio.windows.bounds(display_window, dataset.transform)

    return subset, subset_bounds, raster_crs


def calculate_rmse(tif_path, point_data):
    with rasterio.open(tif_path) as dataset:
        transformer = Transformer.from_crs("EPSG:4326", dataset.crs, always_xy=True)
        point_x, point_y = transformer.transform(point_data["longitudes"], point_data["latitudes"])
        
        tcd_values = [(value[0]) / 100.0 for value in dataset.sample(zip(point_x, point_y))]

    return  math.sqrt(sum((hij - tcd)**2 for hij, tcd in zip(point_data["Hij"], tcd_values)) / len(tcd_values))

###

if __name__ == "__main__":
    point_data = load_points()

    tcd_rmse = calculate_rmse(config["SATELLITE_TCD_TIF_PATH"], point_data)
    print(f"TCD RMSE (manual Hij vs assigned pixel): {tcd_rmse:.4f}")

    tcd_subset, tcd_bounds, tcd_crs = sample_satellite_target(config["SATELLITE_TCD_TIF_PATH"])
    dlt_subset, dlt_bounds, dlt_crs = sample_satellite_target(config["SATELLITE_DLT_TIF_PATH"])

    value_norm = Normalize(vmin=0.0, vmax=1.0)

    panels = [
        {
            "column": "Hij",
            "title": "Tree Cover Density",
            "cmap": "Greens",
            "raster": tcd_subset,
            "bounds": tcd_bounds,
            "crs": tcd_crs,
            "raster_name": "TCD",
        },
        {
            "column": "Gij_1_2_sum",
            "title": "Dominant Leaf Type",
            "cmap": "Blues",
            "raster": dlt_subset,
            "bounds": dlt_bounds,
            "crs": dlt_crs,
            "raster_name": "DLT",
        },
        {
            "column": "Gij_3",
            "title": "Dominant Leaf Type",
            "cmap": "Oranges",
            "raster": dlt_subset,
            "bounds": dlt_bounds,
            "crs": dlt_crs,
            "raster_name": "DLT",
        },
    ]


    fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(20, 8), constrained_layout=True)

    for ax, panel in zip(axes, panels):
        raster = panel["raster"]
        raster_crs = panel["crs"]

        left, bottom, right, top = panel["bounds"]

        extent = [left, right, bottom, top]

        # Convert point coordinates from longitude/latitude to raster
        coordinate_transformer = Transformer.from_crs("EPSG:4326", raster_crs, always_xy=True)

        point_x, point_y = coordinate_transformer.transform(point_data["longitudes"], point_data["latitudes"])

        # Display the raster background.
        ax.imshow(raster, extent=extent, origin="upper", cmap=panel["cmap"], alpha=1.0)

        # Display the annotation markers.
        markers = ax.scatter(point_x, point_y, c=point_data[panel["column"]], cmap=panel["cmap"], norm=value_norm, s=100, edgecolors="black", linewidths=0.7, zorder=5, label="Manual annotations")

        # Add point IDs next to the markers.
        for point_id, x, y in zip(point_data["ids"], point_x, point_y):
            ax.annotate(point_id, (x, y), xytext=(4, 4), textcoords="offset points", fontsize=7, color="white", zorder=6)

        # Add an independent color bar for each annotation variable.
        fig.colorbar(markers, ax=ax, label=panel["title"], fraction=0.046, pad=0.04)

        ax.set_xlabel(f"Easting ({raster_crs.to_string()})")
        ax.set_ylabel(f"Northing ({raster_crs.to_string()})")
        ax.legend(loc="best")

    plt.savefig("imgs/density.png", bbox_inches="tight", dpi=300)
