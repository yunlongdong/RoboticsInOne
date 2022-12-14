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


class FK_SYM:
    def __init__(self, base2world_rpy, base2world_xyz, MDHs) -> None:
        self.num_joints = len(MDHs)
        self.qs = [symbols('q{}'.format(i+1)) for i in range(self.num_joints)]
        self.mdhs = MDHs
        self.global_tf_list = []
        self.global_tf_list.append(self.get_extrinsic_tf(base2world_rpy, base2world_xyz)*self.tf(0))
        self.global_pos = None
        # self.return_global_pos =  self.calulate_global_pos()
        self.return_global_pos_rot = self.calculate_global_pos_rot()

    def tf(self, index):
        """
        https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
        """
        mdh = self.mdhs[index]
        alpha, a, theta, d = mdh
        theta += self.qs[index]

        T = eye(4)
        T[0, 0] = cos(theta)
        T[0, 1] = -sin(theta)
        T[0, 3] = a

        T[1, 0] = sin(theta)*cos(alpha)
        T[1, 1] = cos(theta)*cos(alpha)
        T[1, 2] = -sin(alpha)
        T[1, 3] = -d*sin(alpha)

        T[2, 0] = sin(theta)*sin(alpha)
        T[2, 1] = cos(theta)*sin(alpha)
        T[2, 2] = cos(alpha)
        T[2, 3] = d*cos(alpha)

        return T
    
    def calulate_global_pos(self):
        last_global_tf = self.global_tf_list[-1]
        for i in range(1, self.num_joints):
            self.global_tf_list.append(last_global_tf * self.tf(i))
            last_global_tf = self.global_tf_list[-1]
        self.global_pos = self.global_tf_list[-1] * Matrix([symbols('x'), symbols('y'), symbols('z'), 1.])
        self.global_pos = self.global_pos[0:3]
        return_global_pos = lambdify([self.qs, ['x', 'y', 'z']], self.global_pos, "numpy")
        return return_global_pos
    
    def calculate_global_pos_rot(self):
        last_global_tf = self.global_tf_list[-1]
        for i in range(1, self.num_joints):
            self.global_tf_list.append(last_global_tf * self.tf(i))
            last_global_tf = self.global_tf_list[-1]
        final_tf = eye(4)
        final_tf[:3, -1] = [symbols('x'), symbols('y'), symbols('z')]
        final_tf[:3, :3] = self.get_extrinsic_rot([symbols('angle_z'), symbols('angle_y'), symbols('angle_x')])
        self.global_pos_rot = self.global_tf_list[-1] * final_tf   
        return_global_pos_rot = lambdify([self.qs, ['x', 'y', 'z'], ['angle_z', 'angle_y', 'angle_x']], self.global_pos_rot, "numpy")
        return return_global_pos_rot
    
    # the followings are utility functions
    def get_extrinsic_tf(self, rpy, xyz):
        tf = eye(4)
        x_rot = self.create_from_x_rotation(rpy[0])
        y_rot = self.create_from_y_rotation(rpy[1])
        z_rot = self.create_from_z_rotation(rpy[2])
        tf[:3, :3] = z_rot * y_rot * x_rot
        tf[:3, 3] = xyz
        return tf
    
    def get_extrinsic_rot(self, rpy):
        x_rot = self.create_from_x_rotation(rpy[0])
        y_rot = self.create_from_y_rotation(rpy[1])
        z_rot = self.create_from_z_rotation(rpy[2])
        return z_rot * y_rot * x_rot

    def create_from_x_rotation(self, theta):
        return Matrix(
            [[1.0, 0.0, 0.0],
             [0.0, cos(theta),-sin(theta)],
             [0.0, sin(theta), cos(theta)]]
        )
    def create_from_y_rotation(self, theta):
        return Matrix(
            [[cos(theta), 0.0, sin(theta)],
            [0.0, 1.0, 0.0],
            [-sin(theta), 0.0, cos(theta)]]
        )
    def create_from_z_rotation(self, theta):
        return Matrix(
            [[cos(theta),-sin(theta), 0.0],
            [sin(theta), cos(theta), 0.0],
            [0.0, 0.0, 1.0]]
        )



def check_fk(filename=''):
    robot = Robot(fileName=filename)
    robot.show_MDH_frame(log=False)

    # parameters
    root_joint = robot.return_root_joint()
    leave_link = robot.return_leave_link()
    base2world_rpy = root_joint.rpy_MDH
    base2world_xyz = root_joint.xyz_MDH
    local_pos = leave_link.com_MDH
    local_rpy = leave_link.rpy_MDH
    
    MDHs = robot.MDH_params
    fk = FK_SYM(base2world_rpy, base2world_xyz, MDHs)
    # print("base2world_rpy=", base2world_rpy)
    # print("base2world_xyz=", base2world_xyz)
    # print("MDHs=", MDHs)
    # print("local_pos=", local_pos)
    # print("local_rpy=", local_rpy)

    # randomly set joint angles
    qs = list(random(fk.num_joints)) # [0.] * fk.num_joints
    print("set random joint angle=", qs)
    robot.set_joint_angle(qs)

    # calculated by fk
    global_pos_rot = fk.return_global_pos_rot(qs, local_pos, local_rpy)
    global_pos, global_rot = global_pos_rot[:3, -1], global_pos_rot[:3, :3]
    print("*"*50)
    print("fk global_pos=", global_pos)
    
    # calculated by RIO
    print("RIO global_pos=", robot.return_leave_link().abs_com)

    # calculated with pybullet
    phy_Client = p.connect(p.DIRECT)
    p_robot = p.loadURDF(filename, useFixedBase=True)
    # pybullet simulation
    for _ in range(100):
        p.setJointMotorControlArray(p_robot, range(fk.num_joints), p.POSITION_CONTROL, targetPositions=qs)
        p.stepSimulation()
    link_pos, link_orientation, _, _, _, _ = p.getLinkState(p_robot, fk.num_joints-1, computeForwardKinematics=True)
    print("pybullet global_pos=", np.array(link_pos))
    print("pos error=", np.sum(np.abs(np.array(link_pos) - global_pos)))

    p_global_rot = np.array(p.getMatrixFromQuaternion(link_orientation)).reshape(3, 3)
    print("*"*50)
    print("pybullet global_rot=", p_global_rot)
    print("fk global_rot = ", global_rot)
    print("rotation error=", np.sum(np.abs(p_global_rot-global_rot)))
    p.disconnect()


if __name__ == "__main__":
    check_fk(filename=r'$filename')
