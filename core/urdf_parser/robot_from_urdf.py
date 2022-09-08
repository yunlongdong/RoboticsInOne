import xml.etree.ElementTree as ET
from anytree import AnyNode, LevelOrderIter, RenderTree
import numpy as np
import os.path as osp
from pandas import DataFrame
from .utils import *

class Robotlink:
    def __init__(self):
        self.linkname = ''
        self.mesh_fileName = ''
        self.parent_jointname = ''
        self.parent_linkname = ''
        self.mass = 0.
        self.com = np.zeros(3)
        self.rpy = np.zeros(3)
        self.inertia = np.zeros((3, 3)) # origin: CoM
        self.xyz_visual = np.zeros(3)
        self.rpy_visual = np.zeros(3)

        self.inertia_joint_frame = np.zeros((3, 3)) # origin: parent joint frame origin
        self.abs_tf_link = np.eye(4) # absolute tranformation matrix from child link to world
        # self.rel_tf_link = np.eye(4) # tranfromation matrix from child link to parent link
        self.abs_com = np.zeros(3)  # absolute CoM position
        self.abs_tf_visual = np.eye(4)   # absolute mesh transform

        # MDH frame
        self.com_MDH = np.zeros(3)
        self.rpy_MDH = np.zeros(3)
        self.inertia_MDH = np.zeros((3, 3)) # origin: CoM
        self.xyz_visual_MDH = np.zeros(3)
        self.rpy_visual_MDH = np.zeros(3)

        self.inertia_joint_frame_MDH = np.zeros((3, 3)) # origin: parent joint frame origin
        self.abs_tf_link_MDH = np.eye(4) # absolute tranformation matrix from child link to world
        # self.rel_tf_link = np.eye(4) # tranfromation matrix from child link to parent link
        self.abs_com_MDH = np.zeros(3)  # absolute CoM position
        self.abs_tf_visual_MDH = np.eye(4)   # absolute mesh transform
        self.rel_tf_MDH = np.eye(4)


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
        self.jointtype = None
        self.angle = 0
        self.axis = np.zeros(3)
        self.xyz = np.zeros(3)
        self.rpy = np.zeros(3)
        self.parent_link = ''   # name of parent link
        self.child_link = ''    # name of child link

        self.xyz_MDH = np.zeros(3)
        self.rpy_MDH = np.zeros(3)
    
    def log_joint_info(self):
        print("joint {0}: axis {1}, xyz {2}, rpy {3}".format(self.jointname, self.axis, self.xyz, self.rpy))

class Robot:
    def __init__(self, fileName='../../urdf_examples/estun/estun.urdf'):
        self.urdf_file = fileName
        self.urdf_tree = None   # 解析得到的urdf
        self.root_link_node = None
        self.leave_link_node = []
        self.urdf_tree_nodes = []
        self.robotlinks = {}
        self.robotjoints = {}
        self.num_robotjoints = 0
        self.num_robotlinks = 0
        self.MDH_params = None
        # load link and joint info from urdf file
        self.parse_urdf()
        self.calculate_tfs_in_world_frame()
        self.show_MDH_frame()

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

    def set_joint_angle(self, jointangles):
        try:
            assert len(jointangles)==self.num_robotjoints
        except:
            print("Number of joints mismatched with joint angles...")
        for index, robotjoint in enumerate(self.robotjoints.values()):
            robotjoint.angle = jointangles[index]
        # update
        self.calculate_tfs_in_world_frame()
        self.calculate_MDH_tfs_in_world_frame()

    def invert_joint_z(self, jointname):
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
        link = self.robotlinks[linkname]
        old2link = get_extrinsic_tf(robotjoint.rpy, np.zeros(3))
        link_rpy = get_rpy_from_rotation(np.matmul(inv_tf(old2new), old2link))
        link_com = tf_coordinate(inv_tf(old2new), link.com)
        self.set_link_rpy_com(linkname, link_rpy, link_com)
        # set visual
        old2visual = get_extrinsic_tf(self.robotlinks[linkname].rpy_visual, self.robotlinks[linkname].xyz_visual)
        new2visual = np.matmul(inv_tf(old2new), old2visual)
        rpy_visual = get_rpy_from_rotation(new2visual)
        xyz_visual = new2visual[:3, 3]
        self.set_link_rpy_xyz_visual(linkname, rpy_visual, xyz_visual)
        self.calculate_tfs_in_world_frame()
        self.show_MDH_frame()
    
    def export_to_urdf(self):
        self.urdf_tree.write(osp.join(osp.dirname(self.urdf_file), "generated_" + osp.basename(self.urdf_file)))

    def show_MDH_frame(self, log=False):
        # TODO:注意可能需要先reset joint angle，避免abs_tf_link的影响
        self.set_joint_angle(np.zeros(self.num_robotjoints))
        MDH_tf_list_total = []
        MDH_param_list_total = [np.array([0, 0, 0, 0.])]
        jointname_list_subtree = self.get_urdf_subtree()
        for jointname_list in jointname_list_subtree:
            point_list= []
            zaxis_list = []
            for jointname in jointname_list:
                joint = self.robotjoints[jointname]
                link = self.robotlinks[joint.child_link]
                point_list.append(link.abs_tf_link[0:3, 3])
                zaxis_list.append(np.matmul(link.abs_tf_link[0:3, 0:3], joint.axis))
            
            origin_list, xaxis_list, zaxis_list = find_common_vertical_line(point_list, zaxis_list, epsilon=1e-6)
            MDH_param_list_total += get_MDH_params(origin_list, xaxis_list, zaxis_list)
            MDH_tf_list = get_MDH_frame(origin_list, xaxis_list, zaxis_list)
            MDH_tf_list_total += MDH_tf_list
            self.update_MDH_frame(MDH_tf_list, jointname_list)
        
        MDH_pd_frame = DataFrame(MDH_param_list_total, columns=['alpha', 'a', 'theta', 'd'])
        # print("\nModified DH Parameters: (csv)")
        # print(pd_frame.to_csv())
        if log:
            print("\nModified DH Parameters: (markdown)")
            print(MDH_pd_frame.to_markdown())
        self.MDH_params = MDH_pd_frame.to_numpy()
        return MDH_tf_list_total

    """The followings are utility functions"""
    def calculate_tfs_in_world_frame(self):
        for node in LevelOrderIter(self.root_link_node):
            if node.type == 'link' and node.parent != None:
                # last link to world
                parent_tf_world = self.robotlinks[node.parent.parent.id].abs_tf_link
                parent_joint = self.robotjoints[node.parent.id]
                xyz = parent_joint.xyz
                rpy = parent_joint.rpy
                # current link to last link
                tf = get_extrinsic_tf(rpy, xyz)
                # joint angle
                angle = self.robotjoints[node.parent.id].angle
                tf = np.matmul(tf, matrix44.create_from_z_rotation(angle))
                # link to world
                current_tf_world = np.matmul(parent_tf_world, tf)
                self.robotlinks[node.id].abs_tf_link = current_tf_world
                self.robotlinks[node.id].abs_com = tf_coordinate(current_tf_world, self.robotlinks[node.id].com)

                # visual mesh tf
                current_link2visual = get_extrinsic_tf(self.robotlinks[node.id].rpy_visual, self.robotlinks[node.id].xyz_visual)
                # print("error=", current_link2visual1 - current_link2visual)
                self.robotlinks[node.id].abs_tf_visual = np.matmul(current_tf_world, current_link2visual)      
    
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

    def set_link_rpy_xyz_visual(self, linkname, rpy_visual, xyz_visual):
        self.robotlinks[linkname].rpy_visual = rpy_visual
        self.robotlinks[linkname].xyz_visual = xyz_visual
        # update urdf tree
        a = self.urdf_tree.findall("./*[@name='{0}']/visual/origin".format(linkname))
        assert len(a) == 1
        a[0].set('rpy', str(self.robotlinks[linkname].rpy_visual).replace("[", "").replace("]", ""))
        a[0].set('xyz', str(self.robotlinks[linkname].xyz_visual).replace("[", "").replace("]", ""))
    
    def calculate_MDH_tfs_in_world_frame(self):
        for node in LevelOrderIter(self.root_link_node):
            if node.type == 'link' and node.parent != None:
                # last link to world
                parent_tf_world = self.robotlinks[node.parent.parent.id].abs_tf_link_MDH
                parent_joint = self.robotjoints[node.parent.id]
                xyz_MDH = parent_joint.xyz_MDH
                rpy_MDH = parent_joint.rpy_MDH
                # current link to last link
                tf = get_extrinsic_tf(rpy_MDH, xyz_MDH)
                
                # joint angle
                angle = self.robotjoints[node.parent.id].angle
                tf = np.matmul(tf, matrix44.create_from_z_rotation(angle))
                self.robotlinks[node.id].rel_tf_MDH = tf
                # link to world
                current_tf_world = np.matmul(parent_tf_world, tf)
                self.robotlinks[node.id].abs_tf_link_MDH = current_tf_world
                self.robotlinks[node.id].abs_com_MDH = tf_coordinate(current_tf_world, self.robotlinks[node.id].com_MDH)

                # visual mesh tf
                current_link2visual = np.eye(4)
                current_link2visual = get_extrinsic_tf(self.robotlinks[node.id].rpy_visual_MDH, self.robotlinks[node.id].xyz_visual_MDH)
                # print("error=", current_link2visual1 - current_link2visual)
                self.robotlinks[node.id].abs_tf_visual_MDH = np.matmul(current_tf_world, current_link2visual)
                # print("{0} com error=".format(node.id), self.robotlinks[node.id].abs_com_MDH - self.robotlinks[node.id].abs_com)

    def update_MDH_frame(self, MDH_tf_list, jointname_list):
        self.calculate_tfs_in_world_frame()
        last2world_MDH = np.eye(4)
        for MDH_tf, jointname in zip(MDH_tf_list, jointname_list):
            current2world_MDH = MDH_tf
            # last2world_MDH * current2last_MDH = current2world_MDH
            current2last_MDH = np.matmul(inv_tf(last2world_MDH), current2world_MDH)
            last2world_MDH = current2world_MDH
            # change joint property
            self.robotjoints[jointname].xyz_MDH = current2last_MDH[:3, 3]
            self.robotjoints[jointname].rpy_MDH = get_rpy_from_rotation(current2last_MDH)
            # change link property
            linkname = self.robotjoints[jointname].child_link
            robotlink = self.robotlinks[linkname]
            current2world = self.robotlinks[linkname].abs_tf_link
            # current2world * get_extrinsic_rotation(rpy) = current2world_MDH * get_extrinsic_rotation(rpy_MDH)
            rpy_MDH_matrix = np.matmul(np.matmul(inv_tf(current2world_MDH), current2world), get_extrinsic_rotation(robotlink.rpy))
            self.robotlinks[linkname].rpy_MDH = get_rpy_from_rotation(rpy_MDH_matrix)
            # current2world * com = current2world_MDH * com_MDH
            self.robotlinks[linkname].com_MDH = tf_coordinate(np.matmul(inv_tf(current2world_MDH), current2world), self.robotlinks[linkname].com)

            current_link2visual = get_extrinsic_tf(self.robotlinks[linkname].rpy_visual, self.robotlinks[linkname].xyz_visual)
            current_link2visual_MDH = np.matmul(np.matmul(inv_tf(current2world_MDH), self.robotlinks[linkname].abs_tf_link), current_link2visual)
            self.robotlinks[linkname].xyz_visual_MDH = current_link2visual_MDH[:3, 3]
            self.robotlinks[linkname].rpy_visual_MDH = get_rpy_from_rotation(current_link2visual_MDH)
        self.calculate_MDH_tfs_in_world_frame()

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
        self.leave_link_node = list(self.robotlinks.keys())
        num_nodes_no_parent = 0
        for node in self.urdf_tree_nodes:
            # find root nodes
            if node.parent == None:
                num_nodes_no_parent += 1
                self.root_link_node = node
            # find leave nodes
            if node.id in self.robotlinks and self.robotlinks[node.id].parent_linkname in self.leave_link_node:
                self.leave_link_node.remove(self.robotlinks[node.id].parent_linkname)
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

    def get_urdf_subtree(self):
        subtree_list = [] 
        for leave_linkname in self.leave_link_node:
            subtree = []
            while True:
                parent_jointname = self.robotlinks[leave_linkname].parent_jointname
                parent_linkname = self.robotlinks[leave_linkname].parent_linkname
                if parent_jointname == '':
                    break
                subtree.insert(0, parent_jointname)
                leave_linkname = parent_linkname
            subtree_list.append(subtree)
        # print("subtree_list=", subtree_list)
        return subtree_list

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
                        robotlink.inertia = np.array([[float(item.get('ixx')), float(item.get('ixy')), float(item.get('ixz'))],
                                            [float(item.get('ixy')), float(item.get('iyy')), float(item.get('iyz'))],
                                            [float(item.get('ixz')), float(item.get('iyz')), float(item.get('izz'))]])
            if child.tag == 'visual':
                for item in child:
                    if item.tag == "origin":
                        robotlink.xyz_visual = np.array(item.get('xyz').split(), dtype=float)
                        robotlink.rpy_visual = np.array(item.get('rpy').split(), dtype=float)
                        break
        self.robotlinks[robotlink.linkname] = robotlink
        # add link node to urdf tree
        node = AnyNode(id=robotlink.linkname, parent=None, children=None, type='link')
        self.urdf_tree_nodes.append(node)
        return robotlink

    def process_joint(self, joint):
        robotjoint = Robotjoint()
        robotjoint.jointname = joint.get('name')
        jointtype = joint.get('type')
        if jointtype in ["revolute", "fixed"]:
            robotjoint.jointtype = jointtype
        else:
            raise Exception('Can only deal with revolute or fixed joints now!!! The urdf contains {0} joints...".format(jointtype)')

        for child in joint:
            if child.tag == 'axis':
                assert robotjoint.jointtype == "revolute"
                robotjoint.axis = np.array(child.get('xyz').split(), dtype=float)
                try:
                    np.testing.assert_allclose(robotjoint.axis, np.array([0, 0, 1.]))
                except:
                    print("joint axis:", robotjoint.axis)
                    raise Exception("Error: all axes of joints should be [0, 0, 1]...")
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
        self.robotlinks[robotjoint.child_link].parent_linkname = robotjoint.parent_link
        self.robotlinks[robotjoint.child_link].parent_jointname = robotjoint.jointname
        return robotjoint


if __name__ == "__main__":
    file_full_path = osp.dirname(osp.abspath(__file__))
    robot = Robot(fileName=osp.join(file_full_path, '../../urdf_examples/estun/estun.urdf'))
    # robot = Robot(fileName=osp.join(file_full_path, '../../urdf_examples/kuka iiwa/model.urdf'))
    robot.log_urdf_info()
    robot.show_MDH_frame(log=True)
    robotlinks = list(robot.robotlinks.values())
    print("absolute com=", robotlinks[-1].abs_com)
    robot.set_joint_angle(np.random.random(robot.num_robotjoints))
    