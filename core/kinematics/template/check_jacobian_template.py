from sympy import symbols, sin, cos, lambdify, zeros
from sympy.matrices import Matrix, eye
import sys
sys.path.append(r'../')
from urdf_parser.robot_from_urdf import *
from urdf_parser.utils import *
import os.path as osp
import pybullet as p
from numpy.random import random


class JAC_SYM:
    def __init__(self, base2world_rpy, base2world_xyz, MDHs) -> None:
        self.num_joints = len(MDHs)
        self.qs = [symbols('q{}'.format(i+1)) for i in range(self.num_joints)]
        self.mdhs = MDHs
        self.jacobian = zeros(6, self.num_joints)

        self.global_tf_list = []
        self.global_tf_list.append(self.get_extrinsic_tf(base2world_rpy, base2world_xyz)*self.tf(0))
        self.z_list = [self.global_tf_list[-1][:3, 2]]
        self.o_oc = []
        self.return_jacobian = self.calulate_global_jacobian_symbol()
        
    def get_modified_dh_frame(self, index):
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
    
    def calulate_global_jacobian_symbol(self):
        local_pos = Matrix([symbols('x'), symbols('y'), symbols('z'), 1.])
        last_global_tf = self.global_tf_list[-1]
        for i in range(1, self.num_joints):
            # update global tf
            global_tf = last_global_tf * self.tf(i)
            self.global_tf_list.append(global_tf)
            self.z_list.append(global_tf[:3, 2])
            last_global_tf = self.global_tf_list[-1]
        oc = self.global_tf_list[-1] * local_pos
        for i in range(self.num_joints):
            o_oc = oc - self.global_tf_list[i][:, 3]
            o_oc = o_oc[:3, :]
            self.o_oc.append(o_oc)
            self.jacobian[:3, i] = self.skew_symmetric_matrix(self.z_list[i]) * self.o_oc[i]
            self.jacobian[3:, i] = self.z_list[i]
        calulate_global_jacobian = lambdify([self.qs, ['x', 'y', 'z']], self.jacobian, "numpy")
        return calulate_global_jacobian

    # The followings are utility functions
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
    
    def skew_symmetric_matrix(self, vec):
        a1, a2, a3 = vec[0], vec[1], vec[2]
        return Matrix([[0, -a3, a2],
                    [a3, 0, -a1],
                    [-a2, a1, 0]])

    def get_extrinsic_tf(self, rpy, xyz):
        tf = eye(4)
        x_rot = self.create_from_x_rotation(rpy[0])
        y_rot = self.create_from_y_rotation(rpy[1])
        z_rot = self.create_from_z_rotation(rpy[2])
        tf[:3, :3] = z_rot * y_rot * x_rot
        tf[:3, 3] = xyz
        return tf
    
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
    

def check_jacobian(filename=''):
    robot = Robot(fileName=filename)
    robot.show_MDH_frame(log=True)

    # parameters
    base2world_rpy = list(robot.robotjoints.values())[0].rpy_MDH
    base2world_xyz = list(robot.robotjoints.values())[0].xyz_MDH
    local_pos = list(robot.robotlinks.values())[-1].com_MDH
    MDHs = robot.MDH_params
    jac = JAC_SYM(base2world_rpy, base2world_xyz, MDHs)
    # randomly set joint angles
    qs = list(random(jac.num_joints)) # [0.] * jac.num_joints#
    print("set random joint angle=", qs)
    # calculated by jac
    jacobian = jac.return_jacobian(qs, local_pos)
    print("generated jacobian =", jacobian)

    # calculated with pybullet
    phy_Client = p.connect(p.DIRECT)
    p_robot = p.loadURDF(filename, useFixedBase=True)
    # pybullet simulation
    linearJacobian, angularJacobian = p.calculateJacobian(p_robot, linkIndex=jac.num_joints-1, localPosition=list(robot.robotlinks.values())[-1].com, objPositions=qs, objVelocities=[0.]*jac.num_joints, objAccelerations=[0.]*jac.num_joints)
    p_jacobian = np.concatenate((np.array(linearJacobian), np.array(angularJacobian)), axis=0)
    print("pybullet jacobian=", p_jacobian)
    print("jacobian error=", np.sum(np.abs(p_jacobian-jacobian), axis=(1, 0)))


if __name__ == "__main__":
    check_jacobian(filename=r'$filename')
