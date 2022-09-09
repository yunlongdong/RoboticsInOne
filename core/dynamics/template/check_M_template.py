from sympy import symbols, sin, cos, lambdify
from sympy.matrices import Matrix, eye
import sys
sys.path.append(r'../')
from urdf_parser.robot_from_urdf import *
from urdf_parser.utils import *
import os.path as osp
import pybullet as p
from numpy.random import random
import numpy as np

def return_elements(inertia_matrix):
    return inertia_matrix[0, 0], inertia_matrix[0, 1], inertia_matrix[0, 2], inertia_matrix[1, 1], inertia_matrix[1, 2], inertia_matrix[2, 2]

def return_inertia(qs, robot):
    # print([robotlink.mass for robotlink in list(robot.robotlinks.values())])
    _, $m_index = [robotlink.mass for robotlink in list(robot.robotlinks.values())]
    $q_index = qs
    inertia_list = [robotlink.inertia_MDH for robotlink in list(robot.robotlinks.values())]
    $com_code
    $inertia_code

    $symoro_M_code

    return np.array($Matrix)

def check_fk(filename=''):
    file_full_path = osp.dirname(osp.abspath(__file__))
    # robot
    robot = Robot(fileName=osp.join(file_full_path, filename))
    robot.show_MDH_frame(log=True)
    # randomly set joint angles
    qs =list(random(robot.num_robotjoints))
    robot.set_joint_angle(qs)
    print("set random joint angle=", qs)
    # calculated by symoro
    symoro_inertia = return_inertia(qs, robot)
    print("symoro inertia matrix=", symoro_inertia)
    # calculated by pybullet
    phy_Client = p.connect(p.DIRECT)
    p_robot = p.loadURDF(osp.join(file_full_path, filename), useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE) # pybullet problem: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12936
    p_inertia = np.array(p.calculateMassMatrix(p_robot, objPositions=qs))
    print("pybullet inertia matrix=", p_inertia)
    print("error=", np.sum(np.abs(p_inertia - symoro_inertia), axis=(1, 0)))

if __name__ == "__main__":
    check_fk(filename=r'$filename')
