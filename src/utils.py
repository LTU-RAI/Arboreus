import os, yaml, rasterio, matplotlib.pyplot as plt

from pyproj import Transformer
from rasterio.warp import transform
from rasterio.windows import from_bounds
from matplotlib.patches import Rectangle

###

with open('config/params.yaml', 'r') as file:
    config = yaml.safe_load(file)

def visualize_window(rdx, window_lon, window_lat, window_width, window_height):

    with rasterio.open(config["SATELLITE_TCD_TIF_PATH"]) as ds:

        crs_rio = ds.crs
        xs, ys = transform("EPSG:4326", crs_rio, \
                            [config["SATELLITE_WEST_TARGET"], config["SATELLITE_EAST_TARGET"]], \
                            [config["SATELLITE_NORTH_TARGET"], config["SATELLITE_SOUTH_TARGET"]])

        x_west_m, x_east_m = xs
        y_north_m, y_south_m = ys

        display_window = from_bounds(x_west_m, y_south_m, x_east_m, y_north_m, transform=ds.transform)
        display_window = display_window.round_offsets().round_lengths()

        subset = ds.read(1, window=display_window)
        subset_transform = ds.window_transform(display_window)
        subset_bounds = rasterio.windows.bounds(display_window, ds.transform)

    to_3035 = Transformer.from_crs("EPSG:4326", crs_rio, always_xy=True)
    window_x, window_y = to_3035.transform(window_lon, window_lat)

    fig, ax = plt.subplots(figsize=(12, 10))
    left, bottom, right, top = subset_bounds

    image = ax.imshow(subset, extent=[left, right, bottom, top], origin="upper", cmap="viridis")
    plt.colorbar(image, ax=ax, label="Tree Cover Density")

    allowed_rectangle = Rectangle(
        (x_west_m, y_south_m),
        x_east_m - x_west_m,
        y_north_m - y_south_m,
        linewidth=3,
        edgecolor="red",
        facecolor="none",
        label="Allowed area",
    )
    ax.add_patch(allowed_rectangle)

    forest_rectangle = Rectangle(
        (window_x, window_y),
        window_width,
        window_height,
        linewidth=3,
        edgecolor="blue",
        facecolor="none",
        label=f"Forest window ({window_width:.0f} x {window_height:.0f} m)",
    )
    ax.add_patch(forest_rectangle)

    ax.scatter(window_x, window_y, s=120, marker="x", linewidth=3, color="blue", label="Window start")

    plt.tight_layout()
    plt.savefig(os.path.join(config["TREES_DIR"], "ply", f"{rdx:02d}", "satellite.png"), bbox_inches="tight", dpi=300)
    plt.close()