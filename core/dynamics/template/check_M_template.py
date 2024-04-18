import sys
import os.path as osp
sys.path.append(osp.join(osp.dirname(osp.abspath(__file__)), "../../core/"))
from urdf_parser.robot_from_urdf import *
from urdf_parser.utils import *
import pybullet as p
from numpy.random import random
import numpy as np
from numpy import sin, cos

def return_elements(inertia_matrix):
    return inertia_matrix[0, 0], inertia_matrix[0, 1], inertia_matrix[0, 2], inertia_matrix[1, 1], inertia_matrix[1, 2], inertia_matrix[2, 2]

def return_M(qs, robot):
    links_in_order = robot.chain.ordered_links
    root_link = robot.chain.root_link
    _, $m_index = [robotlink.mass for robotlink in links_in_order]
    $q_index = qs
    inertia_list = [robotlink.inertia_MDH for robotlink in links_in_order]
    $com_code
    $inertia_code

    $symoro_M_code

    return np.array($Matrix)

def check_M(filename=''):
    # robot
    robot = Robot(filename)
    robot.change_chain($ee_link_name)
    robot.show_MDH_frame(verbose=True)
    # randomly set joint angles
    qs =list(random(robot.chain.num_chain_joints))
    robot.set_joint_angle(qs)
    print("set random joint angle=", qs)
    # calculated by symoro
    symoro_inertia = return_M(qs, robot)
    print("symoro inertia matrix=", symoro_inertia)
    # calculated by pybullet
    phy_Client = p.connect(p.DIRECT)
    p_robot = p.loadURDF(filename, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE) # pybullet problem: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12936
    
    joint_name_list = robot.chain.ordered_chain_joints_name
    joint_indices = []
    for i in range(p.getNumJoints(p_robot)):
        joint_info = p.getJointInfo(p_robot, i)
        if joint_info[1].decode("utf-8") in joint_name_list:
            joint_indices.append(i)
    p_qs = np.zeros(p.getNumJoints(p_robot))
    p_qs[joint_indices] = np.array(qs)

    p_inertia = np.array(p.calculateMassMatrix(p_robot, objPositions=p_qs.tolist()))
    print(f"p_inertia shape={p_inertia.shape}")
    p_inertia = p_inertia[joint_indices, :][:, joint_indices]
    print("pybullet inertia matrix=", p_inertia)
    print("error=", np.sum(np.abs(p_inertia - symoro_inertia), axis=(1, 0)))
    p.disconnect()
if __name__ == "__main__":
    check_M(filename=r'$filename')
