#!/bin/bash

rm -rf data/adtree data/ground data/obj data/xyz data/ply data/urdf

# Generate synthetic forest with tree trunks
python3 src/forest.py

# Generate ground mesh from geo-data
python3 src/ground.py

# Transform .ply format to .xyz
python3 src/generator/val/preprocess.py

# Estimate trunk from point cloud
python3 src/generator/val/adtree.py

## https://urdf-viewer-pi.vercel.app/

# Build URDF skeletons
# python3 src/generator.train/skeleton.py

# Join URDF trees
# python3 src/generator.train/merge.py

# Import estimated tree meshes
python3 src/generator/val/blender.py