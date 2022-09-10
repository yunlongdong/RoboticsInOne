import sys
sys.path.append(r'../')
from urdf_parser.robot_from_urdf import *
from urdf_parser.utils import *
import pybullet as p
from numpy.random import random
import numpy as np
from numpy import sin, cos, sign

def return_elements(inertia_matrix):
    return inertia_matrix[0, 0], inertia_matrix[0, 1], inertia_matrix[0, 2], inertia_matrix[1, 1], inertia_matrix[1, 2], inertia_matrix[2, 2]

def return_jointtorque(qs, dqs, ddqs, robot):
    _, $m_index = [robotlink.mass for robotlink in list(robot.robotlinks.values())]
    $q_index = qs
    $dq_index = dqs
    $ddq_index = ddqs
    g = -9.81
    base_rotation = get_extrinsic_rotation(list(robot.robotlinks.values())[0].rpy_MDH)[:3, :3]
    gx, gy, gz = np.matmul(base_rotation.T, np.array([0, 0, g]))
    $set_Fs
    $set_Fv
    inertia_list = [robotlink.inertia_MDH for robotlink in list(robot.robotlinks.values())]
    $com_code
    $inertia_code

    $symoro_idm_code

    return np.array($Matrix)

def check_jointtorque(filename=''):
    # robot
    robot = Robot(filename)
    robot.show_MDH_frame(log=True)
    # randomly set joint angles
    qs =list(random(robot.num_robotjoints))
    robot.set_joint_angle(qs)
    dqs = list(random(robot.num_robotjoints))
    ddqs = list(random(robot.num_robotjoints))
    print("set random joint angle=", qs)
    print("set random joint velocity=", dqs)
    print("set random joint acceleration=", ddqs)
    # calculated by symoro
    symoro_jointtorque = return_jointtorque(qs, dqs, ddqs, robot)
    print("symoro joint torque=", symoro_jointtorque)
    # calculated by pybullet
    phy_Client = p.connect(p.DIRECT)
    p_robot = p.loadURDF(filename, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE) # pybullet problem: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12936
    p.setGravity(0, 0, -9.81)
    p_jointtorque = np.array(p.calculateInverseDynamics(p_robot, objPositions=qs, objVelocities=dqs, objAccelerations=ddqs))
    print("pybullet joint torque=", p_jointtorque)
    print("error=", np.sum(np.abs(p_jointtorque - symoro_jointtorque)))

if __name__ == "__main__":
    check_jointtorque(filename=r'$filename')
