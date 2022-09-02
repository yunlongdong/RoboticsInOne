import xml.etree.ElementTree as ET
from anytree import AnyNode, LevelOrderIter, RenderTree
import numpy as np
import os.path as osp
from pandas import DataFrame
from .utils import *

class Robotlink:
    def __init__(self):
        self.mass = 0.
        self.com = np.zeros(3)
        self.rpy = np.zeros(3)
        self.inertia = np.zeros((3, 3))
        self.mesh_fileName = ''
        self.linkname = ''
        self.abs_tf = np.eye(4) # absolute tranformation matrix from link to world
        self.rel_tf = np.eye(4) # tranfromation matrix from this link to last link

    @property
    def linkRPY(self):
        yaw = np.arctan2(self.abs_tf[1, 0],self.abs_tf[0, 0])
        pitch = np.arctan2(-self.abs_tf[2, 0], np.sqrt(self.abs_tf[2, 1]**2 + self.abs_tf[2, 2]**2))
        roll = np.arctan2(self.abs_tf[2, 1], self.abs_tf[2, 2])
        return np.array([roll, pitch, yaw])

    @property
    def linkPos(self):
        return self.abs_tf[0:3, 3]
    
    @property
    def linkRotation(self):
        return self.abs_tf[0:3, 0:3]
    
    def log_link_info(self):
        print("link {0}: mass {1}, com {2}, rpy {3}".format(self.linkname, self.mass, self.com, self.rpy))

class Robotjoint:
    def __init__(self):
        self.jointname = ''
        self.angle = 0
        self.axis = np.zeros(3)
        self.xyz = np.zeros(3)
        self.rpy = np.zeros(3)
        self.parent_link = ''   # name of parent link
        self.child_link = ''    # name of child link
    
    def log_joint_info(self):
        print("joint {0}: axis {1}, xyz {2}, rpy {3}".format(self.jointname, self.axis, self.xyz, self.rpy))

class Robot:
    def __init__(self, fileName='../urdf_examples/exo/exo.urdf'):
        self.urdf_file = fileName
        self.root_link_node = None
        self.urdf_tree_nodes = []
        self.robotlinks = {}
        self.robotjoints = {}
        # load link and joint info from urdf file
        self.parse_urdf()
        self.calculate_tfs_in_world_frame()

    def log_urdf_info(self):
        print("URDF Tree:")
        for pre, _, node in RenderTree(self.root_link_node):
            print('%s%s' % (pre, node.id))
        print("links info:")
        for link in self.robotlinks.values():
            link.log_link_info()
        print("joints info:")
        for joint in self.robotjoints.values():
            joint.log_joint_info()

    def calculate_modified_dh_params(self, log=True):
        if log:
            print("\ncalculate_dh_params...\n")
        point_list= []
        zaxis_list = []
        for node in LevelOrderIter(self.root_link_node):
            if node.type == 'link' and node.parent != None:
                point_list.append(self.robotlinks[node.id].abs_tf[0:3, 3])
                zaxis_list.append(np.matmul(self.robotlinks[node.id].abs_tf[0:3, 0:3], self.robotjoints[node.parent.id].axis))
        
        if log:
            print("point_list=", point_list)
            print("zaxis_list=", zaxis_list)
        robot_dh_params = get_modified_dh_params(point_list, zaxis_list, epsilon=1e-6)



        pd_frame = DataFrame(robot_dh_params, columns=['alpha', 'd', 'theta', 'r'])
        # print("\nModified DH Parameters: (csv)")
        # print(pd_frame.to_csv())
        print("\nModified DH Parameters: (markdown)")
        print(pd_frame.to_markdown())

    def set_joint_angle(self, jointangles):
        for index, robotjoint in enumerate(self.robotjoints.values()):
            robotjoint.angle = jointangles[index]
        # update
        self.calculate_tfs_in_world_frame()

    """The followings are utility functions"""
    def calculate_tfs_in_world_frame(self):
        for node in LevelOrderIter(self.root_link_node):
            if node.type == 'link' and node.parent != None:
                parent_tf_world = self.robotlinks[node.parent.parent.id].abs_tf
                xyz = self.robotjoints[node.parent.id].xyz
                rpy = self.robotjoints[node.parent.id].rpy
                tf = get_extrinsic_rotation(rpy)
                tf[0:3, 3] = xyz
                angle = self.robotjoints[node.parent.id].angle
                tf = np.matmul(tf, matrix44.create_from_z_rotation(angle))
                self.robotlinks[node.id].rel_tf = tf
                
                abs_tf = np.matmul(parent_tf_world, tf)
                
                self.robotlinks[node.id].abs_tf = abs_tf
    
    def parse_urdf(self):
        urdf_root = self.get_urdf_root()
        # deal with link node
        for child in urdf_root:
            if child.tag == 'link':
                self.process_link(child)
        # deal with joint node
        for child in urdf_root:
            if child.tag == 'joint':
                self.process_joint(child)
                
        # find root link
        num_nodes_no_parent = 0
        for node in self.urdf_tree_nodes:
            if node.parent == None:
                num_nodes_no_parent += 1
                self.root_link_node = node
        if num_nodes_no_parent != 1:
            print("Error: Should only be one root link!!!")
    
    def get_urdf_root(self):
        try:
            tree = ET.parse(self.urdf_file)
        except ET.ParseError:
            print('ERROR: Could not parse urdf file.')

        return tree.getroot()
    
    def process_link(self, link):
        robotlink = Robotlink()
        robotlink.linkname = link.get('name')
        robotlink.mesh_fileName =  osp.join(osp.dirname(self.urdf_file), link.find("visual").find("geometry").find("mesh").get("filename"))

        for child in link:
            if child.tag == 'inertial':
                for item in child:
                    if item.tag == "origin":
                        robotlink.com = np.array(item.get('xyz').split(), dtype=float)
                        robotlink.rpy = np.array(item.get('rpy').split(), dtype=float)
                    elif item.tag == "mass":
                        robotlink.mass = float(item.get('value'))
                    elif item.tag == "inertia":
                        robotlink.inertia = np.array([[float(item.get('ixx')), float(item.get('ixy')), float(item.get      ('ixz'))],
                                            [float(item.get('ixy')), float(item.get('iyy')), float(item.get('iyz'))],
                                            [float(item.get('ixz')), float(item.get('iyz')), float(item.get('izz'))]])
                
        self.robotlinks[robotlink.linkname] = robotlink
        # add link node to urdf tree
        node = AnyNode(id=robotlink.linkname, parent=None, children=None, type='link')
        self.urdf_tree_nodes.append(node)
        return robotlink

    def process_joint(self, joint):
        robotjoint = Robotjoint()
        robotjoint.jointname = joint.get('name')

        for child in joint:
            if child.tag == 'axis':
                robotjoint.axis = np.array(child.get('xyz').split(), dtype=float)
                try:
                    np.testing.assert_allclose(robotjoint.axis, np.array([0, 0, 1.]))
                except:
                    print("joint axis:", robotjoint.axis)
                    print("Error: all axes of joints should be [0, 0, 1]...")
            elif child.tag == 'origin':
                robotjoint.xyz = np.array(child.get('xyz').split(), dtype=float)
                robotjoint.rpy = np.array(child.get('rpy').split(), dtype=float)
            elif child.tag == 'parent':
                robotjoint.parent_link = child.get('link')
            elif child.tag == 'child':
                robotjoint.child_link = child.get('link')
        self.robotjoints[robotjoint.jointname] = robotjoint
        jointnode = AnyNode(id=robotjoint.jointname, parent=None, children=None, type='joint')
        self.urdf_tree_nodes.append(jointnode)
        # Find parent and child link
        for node in self.urdf_tree_nodes:
            if node.id == robotjoint.parent_link:
                jointnode.parent = node
            if node.id == robotjoint.child_link:
                node.parent = jointnode
        return robotjoint


if __name__ == "__main__":
    robot = Robot(fileName='../urdf_examples/half_exo/half_exo.urdf')
    robot.log_urdf_info()
    robot.calculate_modified_dh_params()
    