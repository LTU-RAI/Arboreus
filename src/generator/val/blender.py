import os, bpy, cv2, yaml, math, shutil, mathutils, argparse

###

with open('config/params.yaml', 'r') as file:
    config = yaml.safe_load(file)

def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    for _ in range(3):
        try:
            bpy.ops.outliner.orphans_purge(do_recursive=True)
        except Exception:
            pass


def create_image_material(name, image_path, use_alpha=False):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    mat.use_fake_user = True

    if use_alpha:
        mat.blend_method = 'BLEND'
        mat.shadow_method = 'HASHED'

    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    for node in list(nodes):
        nodes.remove(node)

    output_node = nodes.new(type="ShaderNodeOutputMaterial")
    output_node.location = (400, 0)

    bsdf_node = nodes.new(type="ShaderNodeBsdfPrincipled")
    bsdf_node.location = (150, 0)

    tex_node = nodes.new(type="ShaderNodeTexImage")
    tex_node.location = (-200, 0)
    tex_node.image = bpy.data.images.load(image_path, check_existing=False)

    links.new(tex_node.outputs["Color"], bsdf_node.inputs["Base Color"])

    if use_alpha and "Alpha" in tex_node.outputs and "Alpha" in bsdf_node.inputs:
        links.new(tex_node.outputs["Alpha"], bsdf_node.inputs["Alpha"])

    links.new(bsdf_node.outputs["BSDF"], output_node.inputs["Surface"])

    return mat


def assign_material(obj, material):
    if obj.type != "MESH" or material is None:  return

    if obj.data.materials:
        obj.data.materials.clear()

    obj.data.materials.append(material)


def import_obj_and_apply_x_rotation(filepath):
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.wm.obj_import(filepath=filepath)

    imported_objects = [obj for obj in bpy.context.selected_objects if obj.type == "MESH"]

    for obj in imported_objects:
        obj.rotation_euler[0] = 0  # keep X rotation as requested

    if imported_objects:
        bpy.context.view_layer.objects.active = imported_objects[0]
        bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

    return imported_objects


def shift_meshes_to_ground_center(objects, ground_obj):
    if ground_obj is None:
        raise ValueError("Ground object is None.")

    if ground_obj.type != "MESH":
        raise ValueError(f"Ground object '{ground_obj.name}' is not a mesh.")

    # Calculate WORLD-SPACE bounding box of ground only
    ground_coords = [ground_obj.matrix_world @ mathutils.Vector(corner) for corner in ground_obj.bound_box]

    min_x = min(v.x for v in ground_coords)
    max_x = max(v.x for v in ground_coords)

    min_y = min(v.y for v in ground_coords)
    max_y = max(v.y for v in ground_coords)

    min_z = min(v.z for v in ground_coords)
    max_z = max(v.z for v in ground_coords)

    center_x = (min_x + max_x) * 0.5
    center_y = (min_y + max_y) * 0.5
    center_z = (min_z + max_z) * 0.5

    print(f"Ground center before shift: ({center_x:.4f}, {center_y:.4f})")

    # Translation required to put ground center at XY = 0, 0
    offset = mathutils.Vector((-center_x, -center_y, -center_z))
    print(f"Translation: ({offset.x:.4f}, {offset.y:.4f}, {offset.z:.4f})")

    # Apply SAME world-space translation to all meshes
    mesh_objects = [obj for obj in objects if obj.type == "MESH" and obj.visible_get()]

    for obj in mesh_objects:
        matrix = obj.matrix_world.copy()
        matrix.translation += offset
        obj.matrix_world = matrix

    bpy.context.view_layer.update()

    return offset

###

if __name__ == "__main__":
    bpy.ops.wm.read_homefile(use_empty=True)

    ground_material, branch_material, leaf_material = None, None, None

    mesh_root = os.path.join(config["TREES_DIR"], "obj")
    os.makedirs(mesh_root, exist_ok=True)

    for rdx in range(config["FOREST_VARIANTS"]):

        out_dir = os.path.join(mesh_root, f"{rdx:02d}")
        os.makedirs(out_dir, exist_ok=True)
        shutil.copytree("./data/textures", os.path.join(out_dir, "textures"), dirs_exist_ok=True)

        ground_material = create_image_material(name="GroundMaterial", image_path=os.path.join(out_dir, "textures/ground.jpg"), use_alpha=False)
        
        pine_branch_material = create_image_material(name="PineBranchMaterial", image_path=os.path.join(out_dir, "textures/pine_branches.jpg"), use_alpha=False)
        pine_leaf_material   = create_image_material(name="PineLeafMaterial", image_path=os.path.join(out_dir, "textures/pine_leaves.png"), use_alpha=True)
        
        spruce_branch_material = create_image_material(name="SpruceBranchMaterial", image_path=os.path.join(out_dir, "textures/spruce_branches.jpg"), use_alpha=False)
        spruce_leaf_material   = create_image_material(name="SpruceLeafMaterial", image_path=os.path.join(out_dir, "textures/spruce_leaves.jpg"), use_alpha=True)
        
        birch_branch_material = create_image_material(name="BirchBranchMaterial", image_path=os.path.join(out_dir, "textures/birch_branches.jpg"), use_alpha=False)
        birch_leaf_material   = create_image_material(name="BirchLeafMaterial", image_path=os.path.join(out_dir, "textures/birch_leaves.jpg"), use_alpha=True)

        rdx_dir = os.path.join(config["TREES_DIR"], "adtree", f"{rdx:02d}")
        if not os.path.isdir(rdx_dir):
            continue

        clear_scene()
        batch_objects = []

        ground_path = os.path.join(config["TREES_DIR"], "ground", f"{rdx:02d}.obj")
        ground_obj = import_obj_and_apply_x_rotation(ground_path)
        for obj in ground_obj:
            assign_material(obj, ground_material)
        batch_objects.extend(ground_obj)

        for folder in sorted(os.listdir(rdx_dir)):
            folder_dir = os.path.join(rdx_dir, folder)
            if not os.path.isdir(folder_dir):
                continue

            if   "pine"   in folder:  branch_material, leaf_material = pine_branch_material, pine_leaf_material
            elif "spruce" in folder:  branch_material, leaf_material = spruce_branch_material, spruce_leaf_material
            elif "birch"  in folder:  branch_material, leaf_material = birch_branch_material, birch_leaf_material

            branch_path = os.path.join(folder_dir, f"{folder}_branches.obj")
            leaves_path = os.path.join(folder_dir, f"{folder}_leaves.obj")

            print(f"Importing {branch_path}")
            branch_objs = import_obj_and_apply_x_rotation(branch_path)
            for obj in branch_objs:
                assign_material(obj, branch_material)
            batch_objects.extend(branch_objs)

            print(f"Importing {leaves_path}")
            leaf_objs = import_obj_and_apply_x_rotation(leaves_path)
            for obj in leaf_objs:
                assign_material(obj, leaf_material)
            batch_objects.extend(leaf_objs)

        if not batch_objects:
            print(f"No meshes found for {rdx_dir}")
            continue

        output_path = os.path.join(out_dir, "arboreus.obj")
        print(f"Exporting combined mesh to: {output_path}")

        valid_objects = [obj for obj in batch_objects if obj and obj.name in bpy.data.objects]
        bpy.ops.object.select_all(action='DESELECT')
        for obj in valid_objects:
            obj.select_set(True)

        if valid_objects:
            bpy.context.view_layer.objects.active = valid_objects[0]

        shift_meshes_to_ground_center(valid_objects, bpy.data.objects[f"{rdx:02d}"])
        bpy.ops.wm.obj_export(filepath=output_path, export_selected_objects=True)

        clear_scene()