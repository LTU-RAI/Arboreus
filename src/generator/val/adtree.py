import os, yaml, subprocess, argparse

###

with open('config/params.yaml', 'r') as file:
    config = yaml.safe_load(file)

if __name__ == "__main__":
    parent = os.path.join(config["TREES_DIR"], "adtree")
    if not os.path.exists(parent):  os.mkdir(parent)

    for rdx in range(config["FOREST_VARIANTS"]):
        os.makedirs(os.path.join(parent, f"{rdx:02d}"), exist_ok=True)

        for file in os.listdir(os.path.join(config["TREES_DIR"], "xyz", f"{rdx:02d}")):
            outpath = os.path.join(parent, f"{rdx:02d}", file.split('.')[0])

            if ("pine" in file):
                command = ['src/adtree/pine/Release/bin/AdTree', os.path.join(config["TREES_DIR"], "xyz", f"{rdx:02d}", file), outpath, '-s']
            elif ("spruce" in file) or ("birch" in file):
                command = ['src/adtree/spruce-birch/Release/bin/AdTree', os.path.join(config["TREES_DIR"], "xyz", f"{rdx:02d}", file), outpath, '-s']

            result = subprocess.run(command, capture_output=True, text=True)
            print("Output:\n", result.stdout)
            print("Error:\n", result.stderr)