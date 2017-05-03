#!/usr/bin/env python3

import _init_paths
import time
import os
import contextlib
from math import radians
import numpy as np
from PIL import Image
from tempfile import TemporaryFile
from lib.utils import stdout_redirected
from lib.config import cfg
from lib.voxel2obj import voxel2mesh, write_obj
import bpy


class BaseRenderer:
    model_idx   = 0

    def __init__(self):
        # bpy.data.scenes['Scene'].render.engine = 'CYCLES'
        # bpy.context.scene.cycles.device = 'GPU'
        # bpy.context.user_preferences.system.compute_device_type = 'CUDA'
        # bpy.context.user_preferences.system.compute_device = 'CUDA_1'

        # changing these values does affect the render.

        # remove the default cube

        bpy.ops.object.select_pattern(pattern="Cube")
        bpy.ops.object.delete()

        render_context = bpy.context.scene.render
        world  = bpy.context.scene.world
        camera = bpy.data.objects['Camera']
        light_1  = bpy.data.objects['Lamp']
        light_1.data.type = 'HEMI'

        # set the camera postion and orientation so that it is in
        # the front of the object
        camera.location       = (1, 0, 0)
        camera.rotation_mode  = 'ZXY'
        camera.rotation_euler = (0, radians(90), radians(90))

        # parent camera with a empty object at origin
        org_obj                = bpy.data.objects.new("RotCenter", None)
        org_obj.location       = (0, 0, 0)
        org_obj.rotation_euler = (0, 0, 0)
        bpy.context.scene.objects.link(org_obj)

        camera.parent = org_obj  # setup parenting

        # render setting
        render_context.resolution_percentage = 100
        world.horizon_color = (1, 1, 1)  # set background color to be white

        # set file name for storing rendering result
        self.result_fn = '%s/render_result_%d.png' % (cfg.DIR.RENDERING_PATH, os.getpid())
        bpy.context.scene.render.filepath = self.result_fn

        self.render_context = render_context
        self.org_obj = org_obj
        self.camera = camera
        self.light = light_1
        self._set_lighting()

    def initialize(self, models_fn, viewport_size_x, viewport_size_y):
        self.models_fn = models_fn
        self.render_context.resolution_x = viewport_size_x
        self.render_context.resolution_y = viewport_size_y

    def _set_lighting(self):
        pass

    def setViewpoint(self, azimuth, altitude, yaw, distance_ratio, fov):
        self.org_obj.rotation_euler = (0, 0, 0)
        self.light.location = (distance_ratio *
                               (cfg.RENDERING.MAX_CAMERA_DIST + 2), 0, 0)
        self.camera.location = (distance_ratio *
                                cfg.RENDERING.MAX_CAMERA_DIST, 0, 0)
        self.org_obj.rotation_euler = (radians(-yaw),
                                       radians(-altitude),
                                       radians(-azimuth))

    def setTransparency(self, transparency):
        """ transparency is either 'SKY', 'TRANSPARENT'
        If set 'SKY', render background using sky color."""
        self.render_context.alpha_mode = transparency

    def selectModel(self):
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.select_pattern(pattern="RotCenter")
        bpy.ops.object.select_pattern(pattern="Lamp*")
        bpy.ops.object.select_pattern(pattern="Camera")
        bpy.ops.object.select_all(action='INVERT')

    def printSelection(self):
        print(bpy.context.selected_objects)

    def clearModel(self):
        self.selectModel()
        bpy.ops.object.delete()

        # The meshes still present after delete
        for item in bpy.data.meshes:
            bpy.data.meshes.remove(item)
        for item in bpy.data.materials:
            bpy.data.materials.remove(item)

    def setModelIndex(self, model_idx):
        self.model_idx = model_idx

    def _get_bottom(self, file_path):
        z_bottom = 10000000 #infty
        with open(file_path, 'r') as f:
            for row in f:
                if row.startswith('v '):
                    z = float(row.split()[3])
                    if z < z_bottom:
                        z_bottom = z
        return z_bottom

    def loadModel(self, file_path=None):
        bottom = None
        if file_path is None:
            file_path = self.models_fn[self.model_idx]

        if file_path.endswith('obj'):
            bottom = self._get_bottom(file_path)
            bpy.ops.import_scene.obj(filepath=file_path)
        elif file_path.endswith('3ds'):
            bpy.ops.import_scene.autodesk_3ds(filepath=file_path)
        elif file_path.endswith('dae'):
            # Must install OpenCollada. Please read README.md
            bpy.ops.wm.collada_import(filepath=file_path)
        else:
            raise Exception("Loading failed: %s Model loading for type %s not Implemented" %
                            (file_path, file_path[-4:]))
        bpy.ops.mesh.primitive_plane_add(location=(0,0,bottom))
        plane = bpy.context.active_object
        plane.scale.xy = (100, 100)

    def render(self, load_model=True, clear_model=True, resize_ratio=None,
               return_image=True, image_path=os.path.join(cfg.RENDERING.BLENDER_TMP_DIR, 'tmp.png')):
        """ Render the object """
        if load_model:
            self.loadModel()

        # resize object
        self.selectModel()
        if resize_ratio:
            bpy.ops.transform.resize(value=resize_ratio)

        # self.result_fn = image_path
        bpy.context.scene.render.filepath = image_path

        bpy.context.scene.use_nodes = True
        tree = bpy.context.scene.node_tree
        links = tree.links
        for n in tree.nodes:
            tree.nodes.remove(n)
        rl = tree.nodes.new('CompositorNodeRLayers')
        map = tree.nodes.new(type="CompositorNodeMapValue")
        map.size = [0.5]
        map.use_min = True
        map.min = [0]
        map.use_max = True
        map.max = [1000000]
        invert = tree.nodes.new(type="CompositorNodeInvert")
        links.new(map.outputs[0], invert.inputs[1])
        out_basepath, out_filename = os.path.split(image_path)
        out_filelist = out_filename.split('.')
        out_filelist.insert(-1, 'depth')
        out_filename = '.'.join(out_filelist)

        # The viewer can come in handy for inspecting the results in the GUI
        depthViewer = tree.nodes.new(type="CompositorNodeViewer")
        links.new(invert.outputs[0], depthViewer.inputs[0])
        # Use alpha from input.
        links.new(rl.outputs[1], depthViewer.inputs[1])
        links.new(rl.outputs[2], map.inputs[0])
        fileOutput = tree.nodes.new(type="CompositorNodeOutputFile")
        fileOutput.base_path = out_basepath
        links.new(invert.outputs[0], fileOutput.inputs[0])
        # v = tree.nodes.new('CompositorNodeViewer')
        # v.use_alpha = False
        # tree.links.new(rl.outputs['Z'], v.inputs[0])
        bpy.ops.render.render(write_still=True)  # save straight to file
        os.rename(os.path.join(out_basepath, 'Image0001.png'),
                os.path.join(out_basepath, out_filename))

        if resize_ratio:
            bpy.ops.transform.resize(value=(1/resize_ratio[0],
                1/resize_ratio[1], 1/resize_ratio[2]))

        if clear_model:
            self.clearModel()

        if return_image:
            im = np.array(Image.open(self.result_fn))  # read the image

            # Last channel is the alpha channel (transparency)
            return im[:, :, :3], im[:, :, 3]


class ShapeNetRenderer(BaseRenderer):

    def __init__(self):
        super().__init__()
        self.setTransparency('TRANSPARENT')

    def _set_lighting(self):
        # Create new lamp datablock
        light_data = bpy.data.lamps.new(name="New Lamp", type='HEMI')

        # Create new object with our lamp datablock
        light_2 = bpy.data.objects.new(name="New Lamp", object_data=light_data)
        bpy.context.scene.objects.link(light_2)

        # put the light behind the camera. Reduce specular lighting
        self.light.location       = (0, -2, 2)
        self.light.rotation_mode  = 'ZXY'
        self.light.rotation_euler = (radians(45), 0, radians(90))
        self.light.data.energy = 0.7

        light_2.location       = (0, 2, 2)
        light_2.rotation_mode  = 'ZXY'
        light_2.rotation_euler = (-radians(45), 0, radians(90))
        light_2.data.energy = 0.7


class VoxelRenderer(BaseRenderer):

    def __init__(self):
        super().__init__()
        self.setTransparency('SKY')

    def _set_lighting(self):
        self.light.location       = (0, 3, 3)
        self.light.rotation_mode  = 'ZXY'
        self.light.rotation_euler = (-radians(45), 0, radians(90))
        self.light.data.energy = 0.7

        # Create new lamp datablock
        light_data = bpy.data.lamps.new(name="New Lamp", type='HEMI')

        # Create new object with our lamp datablock
        light_2 = bpy.data.objects.new(name="New Lamp", object_data=light_data)
        bpy.context.scene.objects.link(light_2)

        light_2.location       = (4, 1, 6)
        light_2.rotation_mode  = 'XYZ'
        light_2.rotation_euler = (radians(37), radians(3), radians(106))
        light_2.data.energy = 0.7

    def render_voxel(self, pred, thresh=0.4,
                     image_path=os.path.join(cfg.RENDERING.BLENDER_TMP_DIR, 'tmp.png')):
        # Cleanup the scene
        self.clearModel()
        out_f = os.path.join(cfg.RENDERING.BLENDER_TMP_DIR, 'tmp.obj')
        occupancy = pred > thresh
        vertices, faces = voxel2mesh(occupancy)
        with contextlib.suppress(IOError):
            os.remove(out_f)
        write_obj(out_f, vertices, faces)

        # Load the obj
        bpy.ops.import_scene.obj(filepath=out_f)
        bpy.context.scene.render.filepath = image_path
        bpy.ops.render.render(write_still=True)  # save straight to file

        im = np.array(Image.open(image_path))  # read the image

        # Last channel is the alpha channel (transparency)
        return im[:, :, :3], im[:, :, 3]


def main():
    """Test function"""
    dn = '/cvgl/group/ShapeNet/ShapeNetCore.v1/02958343/'
    model_id = [line.strip('\n') for line in open(dn + 'models.txt')]
    file_paths = [os.path.join(dn, line, 'model.obj') for line in model_id]
    sum_time = 0
    renderer = ShapeNetRenderer()
    renderer.initialize(file_paths, 500, 500)
    for i, curr_model_id in enumerate(model_id):
        start = time.time()
        image_path = '%s/%s.png' % ('./tmp', curr_model_id[:-4])

        az, el, depth_ratio = list(
            *([360, 5, 0.3] * np.random.rand(1, 3) + [0, 25, 0.65]))

        renderer.setModelIndex(i)
        renderer.setViewpoint(30, 30, 0, 0.7, 25)
        renderer.result_fn = image_path

        # with TemporaryFile() as f, stdout_redirected(f):
        rendering, alpha = renderer.render(load_model=True,
            clear_model=True, image_path=image_path)

        print('Saved at %s' % image_path)
        import ipdb; ipdb.set_trace()

        end = time.time()
        sum_time += end - start
        if i % 10 == 0:
            print(sum_time/(10))
            sum_time = 0


if __name__ == "__main__":
    main()
