
from functools import reduce

import numpy as np
import os.path as osp

import sys
sys.path.append('../')
from simple_3dviz.window import show
from simple_3dviz import Mesh, Lines
from urdf_parser.robot_from_urdf import Robot



def urdf_show(path):
    name = osp.basename(path)
    dir_name = osp.dirname(path)

    robot = Robot(path)
    meshes = []

    # origin axes
    axes = [Lines.axes(size=0.2, width=0.008)]

    for robotlink in robot.robotlinks.values():
        mesh_filename = robotlink.mesh_fileName

        mesh = Mesh.from_file(osp.join(dir_name, mesh_filename), color=(0.89804, 0.91765, 0.92941, 0.2))
        
        mesh.affine_transform(R=robotlink.abs_tf[:3, :3].T, t=robotlink.abs_tf[:3, 3])
        meshes.append(mesh)
        # axis
        axis = Lines.axes(size=0.06, width=0.006, origin=robotlink.abs_tf)
        axes.append(axis)

    # make renderables
    meshes = axes + meshes

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

    show(meshes, title=name, camera_position=camera_position, camera_target=camera_target)