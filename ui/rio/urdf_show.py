
from functools import reduce

import numpy as np
import os.path as osp



from ..s3d import Mesh, Lines, Spherecloud
from ..s3d.behaviours.misc import LightToCamera
from ..s3d.renderables import Renderable, Lines
from ..s3d.behaviours import Behaviour, SceneInit
from ..s3d.behaviours.mouse import MouseRotate, MouseZoom, MousePan
from ..s3d.behaviours.keyboard import CameraReport, SortTriangles

import sys
sys.path.append('../')
from core.urdf_parser.robot_from_urdf import Robot


try:
    from .urdf_frame import Window
except ImportError:
    raise ImportError("No supported gui library was found. Install wxPython.")


def simple_window(init, info, robot, size=(512, 512)):
    """Return a window with the expected behaviours added.

    Arguments
    ---------
        init: callable that sets up the scene
        size: (w, h) the size of the window

    Returns
    -------
        Window instance
    """
    w = Window(info, robot, size)
    w.add_behaviours([SceneInit(init), MouseRotate(), MouseZoom(),
                      MousePan(), CameraReport(), SortTriangles()])
    return w


def show(meshes, axes, info, robot, size=(512, 580), background=(0., 0., 0., 1), title="Scene",
         camera_position=(-2, -2, -2), camera_target=(0, 0, 0),
         up_vector=(0, 0, 1), light=None, behaviours=[], ):
    """Creates a simple window that displays the renderables.

    Arguments
    ---------
        renderables: list[Renderable] the renderables to be displayed in the
                     scene
        size: (w, h) the size of the window
        background: (r, g, b, a) the rgba tuple for the background
        title: str the title of the window
        camera_position: (x, y, z) the position of the camera
        camera_target: (x, y, z) the point that the camera looks at
        up_vector: (x, y, z) defines the floor and sky
        light: (x, y, z) defines the position of the light source
    """
    

    def init(scene):
        # scene.add(Lines.axes(size=0.06, width=0.006, origin=[0, 0, 0.245]))
        # scene.add(Lines.axes(size=0.06, width=0.006, origin=[0, 0, 0.245+0.195]))
        for r in axes+meshes:
            scene.add(r)
        scene.background = background
        scene.camera_position = camera_position
        scene.camera_target = camera_target
        scene.up_vector = up_vector
        if light is not None:
            scene.light = light

    w = simple_window(init, info, robot, size=size)
    w.add_behaviours(behaviours)
    w.show(title)

def scene_init(camera_position, camera_target):
    def inner(scene):
        scene.camera_position = camera_position
        scene.camera_target = camera_target
        scene.light = scene.camera_position
    return inner

def get_all_from_robot(robot):
    # test = list(robot.robotjoints.keys())[1]
    # print("test=", test)
    # robot.invert_joint_z(test)
    # robot.export_to_urdf()

    meshes = []
    mesh_names = []
    # axes list, such as link frame, CoM, remember CoM shound be appended at the last
    # axes = [Lines.axes(size=0.2, width=0.008, name='origin')]
    axes = []
    for robotlink in robot.robotlinks.values():
        mesh_filename = robotlink.mesh_fileName

        mesh = Mesh.from_file(mesh_filename, color=(0.89804, 0.91765, 0.92941, 0.2), name=robotlink.linkname)
        mesh_names.append(robotlink.linkname)

        m = np.eye(4)
        m[:3, :3] = robotlink.abs_tf_visual[:3, :3]
        m[:3, 3] = robotlink.abs_tf_visual[:3, 3]

        mesh.affine_transform(R=m[:3, :3].T, t=m[:3, 3])
        meshes.append(mesh)
        # axis
        axis = Lines.axes(size=0.06, width=0.006, origin=robotlink.abs_tf_link, name=robotlink.linkname)
        axes.append(axis)
        # CoM to the last
        axes.append(Spherecloud(name=robotlink.linkname, centers=[robotlink.abs_com], colors=[0.8, 0, 0, 0.8], sizes=[0.01]))

    return meshes, axes, mesh_names

def urdf_show(path):
    file_name = osp.basename(path)
    robot = Robot(path)

    meshes, axes, mesh_names = get_all_from_robot(robot)

    # auto adjust camera
    bbox_min = reduce(
        np.minimum,
        (m.bbox[0] for m in meshes),
        meshes[0].bbox[0]
    )
    bbox_max = reduce(
        np.maximum,
        (m.bbox[1] for m in meshes),
        meshes[0].bbox[1]
    )
    bbox = [bbox_min, bbox_max]
    center = (bbox[1]-bbox[0])/2 + bbox[0]
    camera_target = center
    camera_position = center + (bbox[1]-center)*2
    # In case the camera is too far, then scale the objects so that the
    # camera distance is not very large.
    # NOTE: This probably only works with a single model
    D = np.sqrt(np.sum((camera_position - camera_target)**2))
    if D > 100:
        s = 100. / D
        camera_target *= s
        camera_position *= s
        for m in meshes:
            m.scale(s)

    show(meshes, axes, mesh_names, robot, size=(800, 770), title=file_name, camera_position=camera_position, camera_target=camera_target, 
            behaviours=[SceneInit(scene_init(camera_position, camera_target)), LightToCamera()],
            )
