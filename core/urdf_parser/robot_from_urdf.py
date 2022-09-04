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
        self.abs_com = np.zeros(3)
        self.abs_tf_reverse = np.eye(4)


    @property
    def linkRPY(self):
        return get_rpy_from_rotation(self.abs_tf)

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
        self.reverse = False
    
    def log_joint_info(self):
        print("joint {0}: axis {1}, xyz {2}, rpy {3}".format(self.jointname, self.axis, self.xyz, self.rpy))

class Robot:
    def __init__(self, fileName='../urdf_examples/estun/estun.urdf'):
        self.urdf_file = fileName
        self.urdf_tree = None
        self.root_link_node = None
        self.urdf_tree_nodes = []
        self.robotlinks = {}
        self.robotjoints = {}
        self.num_robotjoints = 0
        self.num_robotlinks = 0
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
        try:
            assert len(jointangles)==self.num_robotjoints
        except:
            print("Number of joints mismatched with joint angles...")
        for index, robotjoint in enumerate(self.robotjoints.values()):
            robotjoint.angle = jointangles[index]
        # update
        self.calculate_tfs_in_world_frame()

    def invert_joint_z(self, jointname):
        self.robotjoints[jointname].reverse = not self.robotjoints[jointname].reverse
        self.robotjoints[jointname].angle *= -1.
        last2old = get_extrinsic_tf(self.robotjoints[jointname].rpy, self.robotjoints[jointname].xyz)
        old2new = np.eye(4)
        old2new[:3, :3] = matrix33.create_from_x_rotation(np.pi)
        last2new = np.matmul(last2old, old2new)
        last2new_rpy = get_rpy_from_rotation(last2new)
        self.set_joint_rpy_xyz(jointname, last2new_rpy, last2new[:3, 3])
        # change joint pos
        for robotjoint in self.robotjoints.values():
            if robotjoint.parent_link == self.robotjoints[jointname].child_link:
                old2next = get_extrinsic_tf(robotjoint.rpy, robotjoint.xyz)
                last2next = np.matmul(last2old, old2next)
                new2next = np.matmul(inv_tf(last2new), last2next)
                new2next_rpy = get_rpy_from_rotation(new2next)
                self.set_joint_rpy_xyz(robotjoint.jointname, new2next_rpy, new2next[:3, 3])
        # change link pos
        linkname = self.robotjoints[jointname].child_link
        link = self.robotlinks[self.robotjoints[jointname].child_link]
        old2link = get_extrinsic_tf(robotjoint.rpy, np.zeros(3))
        link_rpy = get_rpy_from_rotation(np.matmul(inv_tf(old2new), old2link))
        link_com = tf_coordinate(inv_tf(old2new), link.com)
        # print("inv_tf(old2new)=", inv_tf(old2new))
        # print("link_com=", link_com)
        # print("original link_com=", link.com)
        self.set_link_rpy_com(linkname, link_rpy, link_com)
        self.calculate_tfs_in_world_frame()
        pass
    
    def export_to_urdf(self):
        self.urdf_tree.write(osp.join(osp.dirname(self.urdf_file), "generate_" + osp.basename(self.urdf_file)))
        pass

    """The followings are utility functions"""
    def calculate_tfs_in_world_frame(self):
        for node in LevelOrderIter(self.root_link_node):
            if node.type == 'link' and node.parent != None:
                parent_tf_world = self.robotlinks[node.parent.parent.id].abs_tf
                xyz = self.robotjoints[node.parent.id].xyz
                rpy = self.robotjoints[node.parent.id].rpy
                tf = get_extrinsic_tf(rpy, xyz)
                angle = self.robotjoints[node.parent.id].angle
                tf_reverse = tf.copy()
                # tf for joint
                tf = np.matmul(tf, matrix44.create_from_z_rotation(angle))
                # tf for link
                if self.robotjoints[node.parent.id].reverse:
                    old2new = np.eye(4)
                    old2new[:3, :3] = matrix33.create_from_x_rotation(np.pi)
                    tf_reverse = np.matmul(tf_reverse, matrix44.create_from_z_rotation(angle))
                    tf_reverse = np.matmul(tf_reverse, old2new)
                else:
                    tf_reverse = tf
                
                self.robotlinks[node.id].rel_tf = tf
                abs_tf = np.matmul(parent_tf_world, tf)
                self.robotlinks[node.id].abs_tf = abs_tf
                self.robotlinks[node.id].abs_tf_reverse = np.matmul(parent_tf_world, tf_reverse)
                # print("joint ", self.robotjoints[node.parent.id].jointname, abs_tf)
                self.robotlinks[node.id].abs_com = tf_coordinate(abs_tf, self.robotlinks[node.id].com)
    
    def set_joint_rpy_xyz(self, jointname, rpy, xyz):
        self.robotjoints[jointname].rpy = rpy
        self.robotjoints[jointname].xyz = xyz
        # update urdf tree
        a = self.urdf_tree.findall("./*[@name='{0}']/origin".format(jointname))
        assert len(a) == 1
        a[0].set('rpy', str(self.robotjoints[jointname].rpy).replace("[", "").replace("]", ""))
        a[0].set('xyz', str(self.robotjoints[jointname].xyz).replace("[", "").replace("]", ""))

    def set_link_rpy_com(self, linkname, rpy, com):
        self.robotlinks[linkname].rpy = rpy
        self.robotlinks[linkname].com = com
        # update urdf tree
        a = self.urdf_tree.findall("./*[@name='{0}']/inertial/origin".format(linkname))
        assert len(a) == 1
        a[0].set('rpy', str(self.robotlinks[linkname].rpy).replace("[", "").replace("]", ""))
        a[0].set('xyz', str(self.robotlinks[linkname].com).replace("[", "").replace("]", ""))

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
        
        # num of joints and links
        self.num_robotjoints = len(self.robotjoints)
        self.num_robotlinks = len(self.robotlinks)
    
    def get_urdf_root(self):
        try:
            tree = ET.parse(self.urdf_file)
            self.urdf_tree = tree
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
        jointtype = joint.get('type')
        try:
            assert jointtype=='revolute'
        except:
            print("Can only deal with revolute joints now!!! The urdf contains {0} joints.".format(jointtype))

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
    robot = Robot(fileName='../urdf_examples/estun/estun.urdf')
    robot.log_urdf_info()
    robot.calculate_modified_dh_params()
    