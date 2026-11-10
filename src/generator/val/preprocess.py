import os, yaml, laspy, numpy as np, argparse, open3d as o3d

from plyfile import PlyData

###

with open('config/params.yaml', 'r') as file:
    config = yaml.safe_load(file)

if __name__ == "__main__":
    for rdx in range(config["FOREST_VARIANTS"]):
        input_folder, output_folder = os.path.join(config["TREES_DIR"], f"ply/{rdx:02d}"), os.path.join(config["TREES_DIR"], f"xyz/{rdx:02d}")
        os.makedirs(output_folder, exist_ok=True)

        for fname in os.listdir(input_folder):
            if fname == "ground.ply":  continue

            if fname.lower().endswith(".ply"):
                ply = PlyData.read(os.path.join(input_folder, fname))

                vertex = ply['vertex']
                xyz = np.vstack((vertex['x'], vertex['y'], vertex['z'])).T

                np.savetxt(os.path.join(output_folder, os.path.splitext(fname)[0] + ".xyz"), xyz, fmt="%.6f")
