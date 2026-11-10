import os, bpy, math, argparse, mathutils

###

def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    for _ in range(3):
        try:
            bpy.ops.outliner.orphans_purge(do_recursive=True)
        except Exception:
            pass


def compute_world_bounds(objects):
    coords = []

    for obj in objects:
        if obj.type != "MESH" or not obj.visible_get():
            continue

        for corner in obj.bound_box:
            coords.append(obj.matrix_world @ mathutils.Vector(corner))

    if not coords:
        raise ValueError("No mesh bounds found.")

    min_x = min(v.x for v in coords)
    max_x = max(v.x for v in coords)
    min_y = min(v.y for v in coords)
    max_y = max(v.y for v in coords)
    min_z = min(v.z for v in coords)
    max_z = max(v.z for v in coords)

    center = mathutils.Vector(((min_x + max_x) * 0.5, (min_y + max_y) * 0.5, (min_z + max_z) * 0.5))

    size_x = max_x - min_x
    size_y = max_y - min_y
    size_z = max_z - min_z

    return center, size_x, size_y, size_z


def shift_meshes_to_bbox_center(objects, extra_xy=(-50.0, -50.0)):
    center, _, _, _ = compute_world_bounds(objects)

    offset = mathutils.Vector((-center.x + extra_xy[0], -center.y + extra_xy[1], -center.z))

    for obj in objects:
        if obj.type == "MESH":
            obj.matrix_world.translation += offset

    bpy.context.view_layer.update()

###

if __name__ == "__main__":
    bpy.ops.wm.read_homefile(use_empty=True)

    for rdx in range(50):
        input_obj = os.path.join(f"{rdx:02d}", f"{rdx:02d}.obj")
        if not os.path.exists(input_obj):
            print(f"Skipping missing file: {input_obj}")
            continue

        clear_scene()

        print(f"Importing {input_obj}")

        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.wm.obj_import(filepath=input_obj)

        imported_objects = [obj for obj in bpy.context.selected_objects if obj.type == "MESH"]

        if not imported_objects:
            print(f"No mesh objects found in {input_obj}")
            continue

        shift_meshes_to_bbox_center(imported_objects)

        output_path = os.path.join(f"{rdx:02d}", f"{rdx:02d}.png")
            
        scene = bpy.context.scene
        if scene.world is None:
            scene.world = bpy.data.worlds.new("World")

        scene.world.use_nodes = True
        bg_nodes = scene.world.node_tree.nodes
        bg = bg_nodes.get("Background")
        if bg is not None:
            bg.inputs[0].default_value = (1, 1, 1, 1)

        center, sx, sy, sz = compute_world_bounds(imported_objects)
        size = max(sx, sy, sz)

        cam = bpy.data.objects.get("AngleCam")
        if not cam:
            cam_data = bpy.data.cameras.new("AngleCam")
            cam = bpy.data.objects.new("AngleCam", cam_data)
            scene.collection.objects.link(cam)

        scene.camera = cam
        cam.data.type = 'PERSP'

        # Angle view
        cam.location = (center.x + size * 1.25, center.y - size * 1.25, center.z + size * 1.25)
        
        direction = center - cam.location
        rot_quat = direction.to_track_quat('-Z', 'Y')
        cam.rotation_euler = rot_quat.to_euler()

        scene.render.resolution_x = 1000
        scene.render.resolution_y = 1000
        scene.render.resolution_percentage = 100
        scene.render.image_settings.file_format = 'PNG'
        scene.render.filepath = output_path

        bpy.ops.render.render(write_still=True)
        print(f"Saved render to: {output_path}")

        clear_scene()