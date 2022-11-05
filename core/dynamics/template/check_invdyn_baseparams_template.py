import os.path as osp
import pybullet as p
from numpy.random import random
import numpy as np
from numpy import sin, cos, sign

import sys
sys.path.append(r'../')
from urdf_parser.robot_from_urdf import *
from urdf_parser.utils import *

import symengine
from symengine import var, sin, cos, sign, sqrt, zeros, eye, ones, Matrix
from sympy import simplify, nsimplify, lambdify
import sympy as sym
from sympy.utilities.codegen import codegen
from copy import deepcopy
import time

class invdyn_baseparams:
    def __init__(self, fileName):
        self.fileName = fileName
        self.robot = Robot(fileName=self.fileName)
        self.M = None
        self.C = None
        self.G = None
        self.fric = None
    
    def return_elements(self, inertia_matrix):
        return inertia_matrix[0, 0], inertia_matrix[0, 1], inertia_matrix[0, 2], inertia_matrix[1, 1], inertia_matrix[1, 2], inertia_matrix[2, 2]

    def split_M_C_G_fric(self, theta, tol=1e-10):
        # theta from numpy array to sympy matrix
        theta = Matrix(sym.Matrix(theta))
        num_joints = $num_joints
        qs = [var('q{}'.format(i+1)) for i in range(num_joints)]
        dqs = [var('dq{}'.format(i+1)) for i in range(num_joints)]
        ddqs = [var('ddq{}'.format(i+1)) for i in range(num_joints)]

        X = self.returnA(qs, dqs, ddqs)
        fric = X[:, -2*num_joints:] * theta[-2*num_joints:, 0]
        Mdqq_C_G = X[:, :-2*num_joints] * theta[:-2*num_joints, 0]

        # M+G 矩阵
        Mdqq_G = Mdqq_C_G.subs(dqs, [0.]*num_joints)
        # G矩阵
        G = Mdqq_G.subs(ddqs, [0.]*num_joints)
        # Mdqq
        Mdqq = Mdqq_G - G
        # C
        C = Mdqq_C_G.subs(ddqs, [0.]*num_joints) - G

        # M*ddq矩阵 -> M矩阵
        M = zeros(num_joints)
        start_time = time.time()
        Mdqq = Mdqq.expand()
        print("expanding time={0}...".format(time.time()-start_time))
        for i in range(Mdqq.shape[0]):
            start_time = time.time()
            Mdqq_ij = Mdqq[i, 0]
            print("num_term of joint {0} = {1}".format(i+1, len(Mdqq_ij.args)))
            for index, term in enumerate(Mdqq_ij.args):
                clean_term = 0.
                if abs(term.args[0]) > tol:
                    clean_term = term
                else:
                    continue
                
                for k in range(num_joints):
                    if ddqs[k] in clean_term.free_symbols:
                        M[i, k] += clean_term.subs(ddqs[k], 1.)
                        break
            print("time for calculating M of joint {0} = {1}...".format(i+1, time.time()-start_time))
        M = 0.5*(M + M.T)

        self.M, self.C, self.G, self.fric = M, C, G, fric
        
        return self.M, self.C, self.G, self.fric

    def returnTheta(self):
        links_in_order = self.robot.return_links_in_order()
        root_link = self.robot.return_root_link()
        _, $m_index = [robotlink.mass for robotlink in links_in_order]
        $q_index = [0.] * $num_joints
        $set_Fs
        $set_Fv
        inertia_list = [robotlink.inertia_MDH for robotlink in links_in_order]
        $com_code
        $inertia_code


        $symoro_dim_code

        # $num_theta parameters
        $theta_para
        return theta

    def returnA(self, qs, dqs, ddqs):
        $q_index = qs
        $dq_index = dqs
        $ddq_index = ddqs
        g = -9.81
        root_link = self.robot.return_root_link()
        base_rotation = get_extrinsic_rotation(root_link.rpy_MDH)[:3, :3]
        gx, gy, gz = np.matmul(base_rotation.T, np.array([0, 0, g]))

        $symoro_regp_code

        
        # $num_theta parameters
        $theta_name

        X = zeros($num_joints, len(theta_name))
        for index, name in enumerate(theta_name):
            for i in range(1, $num_joints+1):
                tmp = 0
                try:
                    tmp = eval("DG"+str(i)+name)
                except:
                    pass
                X[i-1, index] = tmp

        return X

    def compareTorque(self):
        # randomly set joint angles
        qs =list(random(self.robot.num_robotjoints))
        dqs = list(random(self.robot.num_robotjoints))
        ddqs = list(random(self.robot.num_robotjoints))
        
        print("set random joint angle=", qs)
        print("set random joint velocity=", dqs)
        print("set random joint acceleration=", ddqs)

        # calculated by symoro
        Theta = self.returnTheta()
        M, C, G, fric = self.split_M_C_G_fric(Theta)
        
        qs_sym = [var('q{}'.format(i+1)) for i in range(self.robot.num_robotjoints)]
        dqs_sym = [var('dq{}'.format(i+1)) for i in range(self.robot.num_robotjoints)]
        ddqs_sym = [var('ddq{}'.format(i+1)) for i in range(self.robot.num_robotjoints)]
        M_ddq = np.matmul(np.array(simplify(M.subs(qs_sym, qs))), np.array(ddqs))
        C = np.array(simplify(C.subs(qs_sym+dqs_sym, qs+dqs))).flatten()
        G = np.array(simplify(G.subs(qs_sym, qs))).flatten()
        fric = np.array(simplify(fric.subs(qs_sym+dqs_sym, qs+dqs))).flatten()
        symoro_jointtorque =  M_ddq + C + G + fric
        print("M_ddq=", M_ddq)
        print("C=", C)
        print("G=", G)
        print("fric=", fric)
        print("symoro joint torque=", symoro_jointtorque)

        # calculated by pybullet
        phy_Client = p.connect(p.DIRECT)
        p_robot = p.loadURDF(self.fileName, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE) # pybullet problem: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12936
        p.setGravity(0, 0, -9.81)
        p_jointtorque = np.array(p.calculateInverseDynamics(p_robot, objPositions=qs, objVelocities=dqs, objAccelerations=ddqs))
        print("pybullet joint torque=", p_jointtorque)
        print("error=", np.sum(np.abs(p_jointtorque - symoro_jointtorque)))
        p.disconnect()


if __name__ =="__main__":
    my_invdyn_baseparams = invdyn_baseparams(fileName=r'$fileName')
    my_invdyn_baseparams.compareTorque()
