import os.path as osp
import pybullet as p
from numpy.random import random
import numpy as np
from numpy import sin, cos, sign

import sys
import os.path as osp
sys.path.append(osp.join(osp.dirname(osp.abspath(__file__)), "../../core/"))
from urdf_parser.robot_from_urdf import *
from urdf_parser.utils import *

class systemID:
    def __init__(self, fileName):
        self.fileName = fileName
        self.robot = Robot(fileName=self.fileName)
        self.robot.change_chain($ee_link_name)
    
    def return_elements(self, inertia_matrix):
        return inertia_matrix[0, 0], inertia_matrix[0, 1], inertia_matrix[0, 2], inertia_matrix[1, 1], inertia_matrix[1, 2], inertia_matrix[2, 2]

    def returnTheta(self):
        links_in_order = self.robot.chain.ordered_links
        root_link = self.robot.chain.root_link
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
        root_link = self.robot.chain.root_link
        base_rotation = get_extrinsic_rotation(root_link.rpy_MDH)[:3, :3]
        gx, gy, gz = np.matmul(base_rotation.T, np.array([0, 0, g]))

        $symoro_regp_code

        
        # $num_theta parameters
        $theta_name

        X = np.zeros(($num_joints, len(theta_name)))
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
        qs =list(random(self.robot.chain.num_chain_joints))
        dqs = list(1000 * random(self.robot.chain.num_chain_joints))
        ddqs = list(1000 * random(self.robot.chain.num_chain_joints))
        
        print("set random joint angle=", qs)
        print("set random joint velocity=", dqs)
        print("set random joint acceleration=", ddqs)
        # calculated by symoro
        Theta = self.returnTheta()
        A = self.returnA(qs, dqs, ddqs)
        symoro_jointtorque = np.matmul(A, Theta)
        print("symoro joint torque=", symoro_jointtorque)
        # calculated by pybullet
        phy_Client = p.connect(p.DIRECT)
        p_robot = p.loadURDF(self.fileName, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE) # pybullet problem: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12936
        p.setGravity(0, 0, -9.81)

        joint_name_list = self.robot.chain.ordered_chain_joints_name
        joint_indices = []
        for i in range(p.getNumJoints(p_robot)):
            joint_info = p.getJointInfo(p_robot, i)
            if joint_info[1].decode("utf-8") in joint_name_list:
                joint_indices.append(i)
        p_qs = np.zeros(p.getNumJoints(p_robot))
        p_dqs = np.zeros(p.getNumJoints(p_robot))
        p_ddqs = np.zeros(p.getNumJoints(p_robot))
        p_qs[joint_indices] = np.array(qs)
        p_dqs[joint_indices] = np.array(dqs)
        p_ddqs[joint_indices] = np.array(ddqs)

        p_jointtorque = np.array(p.calculateInverseDynamics(p_robot, objPositions=p_qs.tolist(), objVelocities=p_dqs.tolist(), objAccelerations=p_ddqs.tolist()))
        p_jointtorque = p_jointtorque[joint_indices]
        print("pybullet joint torque=", p_jointtorque)
        print("error=", np.sum(np.abs(p_jointtorque - symoro_jointtorque)))
        p.disconnect()


if __name__ =="__main__":
    my_systemID = systemID(fileName=r'$fileName')
    my_systemID.compareTorque()
