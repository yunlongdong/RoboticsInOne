from urdfpy import URDF
from anytree import AnyNode, LevelOrderIter, RenderTree
import numpy as np
import os.path as osp
from pandas import DataFrame
from .utils import *


class Robotlink:
    def __init__(self):
        self.linkname = ""
        self.mesh_fileName = ""
        self.mass = 0.0
        self.origin = np.eye(4)
        self.inertia = np.zeros((3, 3))  # origin: CoM, frame: urdf link frame
        self.inertia_joint_frame = np.zeros((3, 3))  # origin: parent joint frame origin
        self.visual_origin = np.eye(4)

        self.abs_tf_link = np.eye(
            4
        )  # absolute tranformation matrix from child link to world
        self.abs_com = np.zeros(3)  # absolute CoM position
        self.abs_tf_visual = np.eye(4)  # absolute mesh transform

        # MDH frame
        self.com_MDH = np.zeros(3)
        self.rpy_MDH = np.zeros(3)
        self.inertia_MDH = np.zeros((3, 3))  # origin: CoM, frame: MDH frame
        self.inertia_joint_frame_MDH = np.zeros(
            (3, 3)
        )  # origin: parent joint frame origin
        self.xyz_visual_MDH = np.zeros(3)
        self.rpy_visual_MDH = np.zeros(3)

        self.abs_tf_link_MDH = np.eye(
            4
        )  # absolute tranformation matrix from child link to world
        self.abs_com_MDH = np.zeros(3)  # absolute CoM position
        self.abs_tf_visual_MDH = np.eye(4)  # absolute mesh transform

    @property
    def com(self):
        return self.origin[0:3, 3]

    @property
    def pure_rotation_tf(self):
        tf = np.eye(4)
        tf[0:3, 0:3] = self.origin[0:3, 0:3]
        return tf

    @property
    def rotation_matrix(self):
        return self.origin[0:3, 0:3]

    @property
    def rpy(self):
        return get_rpy_from_rotation(self.origin)

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
        print(
            f"\tlink {self.linkname}: mass {self.mass}, com {self.com}, rpy {self.rpy}"
        )


class Robotjoint:
    def __init__(self):
        self.jointname = ""
        self.jointtype = None
        self.angle = 0
        self.axis = np.zeros(3)
        self.origin = np.eye(4)
        self.parent_link = ""  # name of parent link
        self.child_link = ""  # name of child link

        self.xyz_MDH = np.zeros(3)
        self.rpy_MDH = np.zeros(3)

    def log_joint_info(self):
        print(
            f"\tjoint {self.jointname}: axis {self.axis}, {self.parent_link}<->{self.child_link}"
        )


class Chain:
    def __init__(self):
        self.chain_links = {}
        self.chain_joints = {}
        self.ordered_chain_joints_name = []
        self.ordered_chain_links_name = []

        # urdf tree and nodes
        self.tree = []
        self.tree_root = None
        self.root_joint_name = ""
        self.root_link_name = ""
        self.ee_link_name = ""
        # MDH
        self.MDH_params = None

    @property
    def num_chain_links(self):
        return len(self.chain_links)

    @property
    def num_chain_joints(self):
        return len(self.chain_joints)

    @property
    def ordered_links(self):
        return [
            self.chain_links[link_name] for link_name in self.ordered_chain_links_name
        ]

    @property
    def root_link(self):
        return self.chain_links[self.root_link_name]

    @property
    def root_joint(self):
        return self.chain_joints[self.root_joint_name]

    @property
    def ee_link(self):
        return self.chain_links[self.ee_link_name]

    # update
    def update_tf(self):
        self.calculate_tfs_in_world_frame()
        self.calculate_MDH_tfs_in_world_frame()

    def log_chain_info(self):
        print("links info:")
        for link in self.chain_links.values():
            link.log_link_info()
        print("joints info:")
        for joint in self.chain_joints.values():
            joint.log_joint_info()

    def calculate_tfs_in_world_frame(self):
        for node in LevelOrderIter(self.tree_root):
            if node.type == "link" and node.parent != None:
                # last link to world
                parent_tf_world = self.chain_links[node.parent.parent.id].abs_tf_link
                parent_joint = self.chain_joints[node.parent.id]
                # joint angle
                angle = self.chain_joints[node.parent.id].angle
                tf = np.matmul(
                    parent_joint.origin, matrix44.create_from_z_rotation(angle)
                )
                # link to world
                current_tf_world = np.matmul(parent_tf_world, tf)
                self.chain_links[node.id].abs_tf_link = current_tf_world
                self.chain_links[node.id].abs_com = tf_coordinate(
                    current_tf_world, self.chain_links[node.id].com
                )

                # visual mesh tf
                current_link2visual = self.chain_links[node.id].visual_origin
                self.chain_links[node.id].abs_tf_visual = np.matmul(
                    current_tf_world, current_link2visual
                )

    def show_MDH_frame(self, verbose=False):
        MDH_tf_list_total = []
        MDH_param_list_total = [np.array([0, 0, 0, 0.0])]

        joint_name_list = []
        for node in LevelOrderIter(self.tree_root):
            if node.type == "joint":
                joint_name_list.append(node.id)
        point_list = []
        zaxis_list = []
        for joint_name in joint_name_list:
            joint = self.chain_joints[joint_name]
            link = self.chain_links[joint.child_link]
            point_list.append(link.abs_tf_link[0:3, 3])
            zaxis_list.append(np.matmul(link.abs_tf_link[0:3, 0:3], joint.axis))

        origin_list, xaxis_list, zaxis_list = find_common_vertical_line(
            point_list, zaxis_list, epsilon=1e-6
        )
        MDH_param_list_total += get_MDH_params(origin_list, xaxis_list, zaxis_list)
        MDH_tf_list = get_MDH_frame(origin_list, xaxis_list, zaxis_list)
        MDH_tf_list_total += MDH_tf_list
        self.update_MDH_frame(MDH_tf_list, joint_name_list)

        MDH_pd_frame = DataFrame(
            MDH_param_list_total, columns=["alpha", "a", "theta", "d"]
        )
        if verbose:
            print("\nModified DH Parameters: (markdown)")
            print(MDH_pd_frame.to_markdown())
        self.MDH_params = MDH_pd_frame.to_numpy()
        return MDH_tf_list_total

    # calculate tfs
    def calculate_MDH_tfs_in_world_frame(self):
        for node in LevelOrderIter(self.tree_root):
            if node.type == "link" and node.parent != None:
                # last link to world
                parent_tf_world = self.chain_links[
                    node.parent.parent.id
                ].abs_tf_link_MDH
                parent_joint = self.chain_joints[node.parent.id]
                xyz_MDH = parent_joint.xyz_MDH
                rpy_MDH = parent_joint.rpy_MDH
                # current link to last link
                tf = get_extrinsic_tf(rpy_MDH, xyz_MDH)

                # joint angle
                angle = self.chain_joints[node.parent.id].angle
                tf = np.matmul(tf, matrix44.create_from_z_rotation(angle))
                # link to world
                current_tf_world = np.matmul(parent_tf_world, tf)
                self.chain_links[node.id].abs_tf_link_MDH = current_tf_world
                self.chain_links[node.id].abs_com_MDH = tf_coordinate(
                    current_tf_world, self.chain_links[node.id].com_MDH
                )

                # visual mesh tf
                current_link2visual = np.eye(4)
                current_link2visual = get_extrinsic_tf(
                    self.chain_links[node.id].rpy_visual_MDH,
                    self.chain_links[node.id].xyz_visual_MDH,
                )
                # print("error=", current_link2visual1 - current_link2visual)
                self.chain_links[node.id].abs_tf_visual_MDH = np.matmul(
                    current_tf_world, current_link2visual
                )

    def update_MDH_frame(self, MDH_tf_list, jointname_list):
        self.calculate_tfs_in_world_frame()
        last2world_MDH = np.eye(4)
        for MDH_tf, jointname in zip(MDH_tf_list, jointname_list):
            current2world_MDH = MDH_tf
            # last2world_MDH * current2last_MDH = current2world_MDH
            current2last_MDH = np.matmul(inv_tf(last2world_MDH), current2world_MDH)
            last2world_MDH = current2world_MDH

            # update joint property
            self.chain_joints[jointname].xyz_MDH = current2last_MDH[:3, 3]
            self.chain_joints[jointname].rpy_MDH = get_rpy_from_rotation(
                current2last_MDH
            )

            # update link property
            linkname = self.chain_joints[jointname].child_link
            link = self.chain_links[linkname]
            current2world = self.chain_links[linkname].abs_tf_link
            # current2world * get_extrinsic_rotation(rpy) = current2world_MDH * get_extrinsic_rotation(rpy_MDH)
            old2MDH = np.matmul(inv_tf(current2world_MDH), current2world)
            rpy_MDH_matrix = np.matmul(
                np.matmul(inv_tf(current2world_MDH), current2world),
                link.pure_rotation_tf,
            )
            self.chain_links[linkname].rpy_MDH = get_rpy_from_rotation(rpy_MDH_matrix)
            # current2world * com = current2world_MDH * com_MDH
            self.chain_links[linkname].com_MDH = tf_coordinate(
                np.matmul(inv_tf(current2world_MDH), current2world),
                self.chain_links[linkname].com,
            )
            # update inertia matrix
            mass = link.mass
            cx, cy, cz = link.com_MDH
            # current2world*get_extrinsic_rotation(robotlink.rpy) * inertia_urdf * ...
            # current2world_MDH*get_extrinsic_rotation(robotlink.rpy_MDH) * inertia_MDH * ...
            old2MDH = old2MDH[:3, :3]
            # tmp = np.matmul(np.matmul(get_extrinsic_rotation(robotlink.rpy_MDH)[:3, :3].T, old2MDH), get_extrinsic_rotation(robotlink.rpy)[:3, :3])
            tmp = np.matmul(old2MDH, get_extrinsic_rotation(link.rpy)[:3, :3])
            self.chain_links[linkname].inertia_MDH = np.matmul(
                np.matmul(tmp, link.inertia), tmp.T
            ) + np.array(
                [
                    [mass * cy**2 + mass * cz**2, -mass * cx * cy, -mass * cx * cz],
                    [-mass * cx * cy, mass * cx**2 + mass * cz**2, -mass * cy * cz],
                    [-mass * cx * cz, -mass * cy * cz, mass * cx**2 + mass * cy**2],
                ]
            )

            # update visual property
            current_link2visual = self.chain_links[linkname].visual_origin
            current_link2visual_MDH = np.matmul(
                np.matmul(
                    inv_tf(current2world_MDH), self.chain_links[linkname].abs_tf_link
                ),
                current_link2visual,
            )
            self.chain_links[linkname].xyz_visual_MDH = current_link2visual_MDH[:3, 3]
            self.chain_links[linkname].rpy_visual_MDH = get_rpy_from_rotation(
                current_link2visual_MDH
            )
        self.calculate_MDH_tfs_in_world_frame()


class Robot:
    def __init__(self, fileName="../../urdf_examples/GX11/GX11.urdf"):
        self.urdf_file = fileName
        self.urdf_robot = URDF.load(fileName)
        # number
        self.num_joints = 0
        self.num_links = 0
        # urdf tree and nodes
        self.chains = []
        self.selected_chain_index = 0
        # load link and joint info from urdf file
        self.parse_urdf()
        for chain in self.chains:
            chain.calculate_tfs_in_world_frame()
            chain.show_MDH_frame()

        self.log_urdf_info()

    @property
    def chain(self):
        return self.chains[self.selected_chain_index]

    def change_chain(self, ee_link_name):
        for i, chain in enumerate(self.chains):
            if chain.ee_link_name == ee_link_name:
                self.selected_chain_index = i
                break

        self.chain.calculate_tfs_in_world_frame()
        self.chain.show_MDH_frame()

    def log_urdf_info(self):
        self.chain.log_chain_info()

    def update_tf(self):
        for chain in self.chains:
            chain.update_tf()

    def show_MDH_frame(self, verbose=False):
        # TODO:注意可能需要先reset joint angle，避免abs_tf_link的影响
        self.set_joint_angle(np.zeros(self.chain.num_chain_joints))
        self.chain.show_MDH_frame(verbose=verbose)

    def set_joint_angle(self, joint_angles):
        try:
            assert len(joint_angles) == len(self.chain.chain_joints)
        except:
            print("Number of joints mismatched with joint angles...")
        for index, joint_name in enumerate(self.chain.ordered_chain_joints_name):
            self.chain.chain_joints[joint_name].angle = joint_angles[index]
        # update
        self.update_tf()

    def invert_joint_z(self, jointname):
        last2old = get_extrinsic_tf(
            self.chain.chain_joints[jointname].rpy,
            self.chain.chain_joints[jointname].xyz,
        )
        old2new = np.eye(4)
        old2new[:3, :3] = matrix33.create_from_x_rotation(np.pi)
        last2new = np.matmul(last2old, old2new)
        last2new_rpy = get_rpy_from_rotation(last2new)
        self.set_joint_rpy_xyz(jointname, last2new_rpy, last2new[:3, 3])
        # change joint pos
        for robotjoint in self.chian.chain_joints.values():
            if robotjoint.parent_link == self.chain.chain_joints[jointname].child_link:
                old2next = get_extrinsic_tf(robotjoint.rpy, robotjoint.xyz)
                last2next = np.matmul(last2old, old2next)
                new2next = np.matmul(inv_tf(last2new), last2next)
                new2next_rpy = get_rpy_from_rotation(new2next)
                self.set_joint_rpy_xyz(
                    robotjoint.jointname, new2next_rpy, new2next[:3, 3]
                )
        # change link pos
        linkname = self.chain.chain_joints[jointname].child_link
        link = self.chain.chain_links[linkname]
        old2link = get_extrinsic_tf(robotjoint.rpy, np.zeros(3))
        link_rpy = get_rpy_from_rotation(np.matmul(inv_tf(old2new), old2link))
        link_com = tf_coordinate(inv_tf(old2new), link.com)
        self.set_link_rpy_com(linkname, link_rpy, link_com)
        # set visual
        old2visual = get_extrinsic_tf(
            self.chain.chain_links[linkname].rpy_visual,
            self.chain.chain_links[linkname].xyz_visual,
        )
        new2visual = np.matmul(inv_tf(old2new), old2visual)
        rpy_visual = get_rpy_from_rotation(new2visual)
        xyz_visual = new2visual[:3, 3]
        self.set_link_rpy_xyz_visual(linkname, rpy_visual, xyz_visual)
        self.update_tf()

    """The followings are utility functions"""

    # export URDF
    def export_to_urdf(self):
        self.urdf_tree.write(
            osp.join(
                osp.dirname(self.urdf_file), "generated_" + osp.basename(self.urdf_file)
            )
        )

    def export_to_MDH(self):
        filename = osp.join(
            osp.dirname(self.urdf_file),
            "generated_" + osp.splitext(osp.basename(self.urdf_file))[0] + "_MDH.txt",
        )
        base2world_rpy = list(self.robotjoints.values())[0].rpy_MDH
        base2world_xyz = list(self.robotjoints.values())[0].xyz_MDH
        with open(filename, "w") as f:
            f.writelines(
                "base2world_rpy = "
                + np.array2string(base2world_rpy, separator=", ")
                + "\n"
            )
            f.writelines(
                "base2world_xyz = "
                + np.array2string(base2world_xyz, separator=", ")
                + "\n"
            )
            f.writelines(np.array2string(self.MDH_params, separator=", ") + "\n")

    def set_joint_rpy_xyz(self, jointname, rpy, xyz):
        self.robotjoints[jointname].rpy = rpy
        self.robotjoints[jointname].xyz = xyz
        # update urdf tree
        a = self.urdf_tree.findall("./*[@name='{0}']/origin".format(jointname))
        assert len(a) == 1
        a[0].set(
            "rpy",
            str(self.robotjoints[jointname].rpy).replace("[", "").replace("]", ""),
        )
        a[0].set(
            "xyz",
            str(self.robotjoints[jointname].xyz).replace("[", "").replace("]", ""),
        )

    def set_link_rpy_com(self, linkname, rpy, com):
        self.robotlinks[linkname].rpy = rpy
        self.robotlinks[linkname].com = com
        # update urdf tree
        a = self.urdf_tree.findall("./*[@name='{0}']/inertial/origin".format(linkname))
        assert len(a) == 1
        a[0].set(
            "rpy", str(self.robotlinks[linkname].rpy).replace("[", "").replace("]", "")
        )
        a[0].set(
            "xyz", str(self.robotlinks[linkname].com).replace("[", "").replace("]", "")
        )

    def set_link_rpy_xyz_visual(self, linkname, rpy_visual, xyz_visual):
        self.robotlinks[linkname].rpy_visual = rpy_visual
        self.robotlinks[linkname].xyz_visual = xyz_visual
        # update urdf tree
        a = self.urdf_tree.findall("./*[@name='{0}']/visual/origin".format(linkname))
        assert len(a) == 1
        a[0].set(
            "rpy",
            str(self.robotlinks[linkname].rpy_visual).replace("[", "").replace("]", ""),
        )
        a[0].set(
            "xyz",
            str(self.robotlinks[linkname].xyz_visual).replace("[", "").replace("]", ""),
        )

    # parse URDF
    def parse_urdf(self):
        # find chains
        for n in self.urdf_robot._end_links:
            # for each chain
            current_chain = Chain()
            path = self.urdf_robot._paths_to_base[n]
            current_chain.ee_link_name = n.name
            current_chain.root_link_name = path[-1].name
            for i in range(len(path)):
                child = path[i]
                current_chain.ordered_chain_links_name.insert(0, child.name)
                current_chain.chain_links[child.name] = self.process_link(child)
                node = AnyNode(id=child.name, parent=None, children=None, type="link")
                current_chain.tree.append(node)
            current_chain.tree_root = current_chain.tree[-1]
            for i in range(len(path) - 1):
                child = path[i]
                parent = path[i + 1]

                joint = self.urdf_robot._G.get_edge_data(child, parent)["joint"]
                if joint.joint_type == "fixed":
                    self.process_fixed_joint(joint)
                elif joint.joint_type == "revolute":
                    current_chain.chain_joints[joint.name] = (
                        self.process_revolute_joint(joint)
                    )
                    current_chain.ordered_chain_joints_name.insert(0, joint.name)
                else:
                    raise Exception(
                        f"Can only deal with revolute or fixed joints now!!! The urdf contains {joint.joint_type} joints..."
                    )

                # construct tree
                j_node = AnyNode(
                    id=joint.name, parent=None, children=None, type="joint"
                )
                for node in current_chain.tree:
                    if node.id == parent.name:
                        j_node.parent = node
                    if node.id == child.name:
                        node.parent = j_node

                for node in LevelOrderIter(current_chain.tree_root):
                    if node.type == "joint":
                        current_chain.root_joint_name = node.id
                        break

                current_chain.tree.append(j_node)
            self.chains.append(current_chain)
            self.num_links += current_chain.num_chain_links
            self.num_joints += current_chain.num_chain_joints

    def process_link(self, link):
        robotlink = Robotlink()
        robotlink.linkname = link.name
        robotlink.mesh_fileName = osp.join(
            osp.dirname(self.urdf_file), link.visuals[0].geometry.mesh.filename
        )
        robotlink.origin = link.inertial.origin
        robotlink.mass = link.inertial.mass
        robotlink.inertia = link.inertial.inertia
        robotlink.visual_origin = link.visuals[0].origin
        return robotlink

    def process_fixed_joint(self, joint):
        return
        # remove the parent link of the fixed joint
        self.robotlinks[joint.child].origin = np.matmul(
            joint.origin, self.robotlinks[joint.parent].origin
        )
        self.robotlinks[joint.child].parent_jointname = self.robotlinks[
            joint.parent
        ].parent_jointname
        self.robotlinks[joint.child].parent_linkname = self.robotlinks[
            joint.parent
        ].parent_linkname
        self.robotlinks[joint.child].inertia = self.robotlinks[
            joint.child
        ].inertia  # FIXME:

        self.robotlinks[joint.child].visual_origin = np.matmul(
            joint.origin, self.robotlinks[joint.child].visual_origin
        )
        self.robotlinks.pop(joint.parent)

        # update anytree node
        for node in self.urdf_tree_nodes:
            if node.id == joint.parent:
                self.urdf_tree_nodes.remove(node)
            if node.id == joint.child:
                node.parent = node.parent.parent
            if node.children == joint.parent:
                node.children = joint.child

        # update joint child link
        for robotjoint in self.robotjoints.values():
            if robotjoint.child_link == joint.parent:
                robotjoint.child_link = joint.child

    def process_revolute_joint(self, joint):
        robotjoint = Robotjoint()
        robotjoint.jointname = joint.name
        robotjoint.axis = joint.axis
        robotjoint.origin = joint.origin
        robotjoint.parent_link = joint.parent
        robotjoint.child_link = joint.child
        return robotjoint


if __name__ == "__main__":
    file_full_path = osp.dirname(osp.abspath(__file__))
    robot = Robot(
        fileName=osp.join(file_full_path, "../../urdf_examples/GX11/GX11_single.urdf")
    )
    robot.log_urdf_info()
    robot.show_MDH_frame(verbose=True)
    robotlinks = list(robot.robotlinks.values())
    print("absolute com=", robotlinks[-1].abs_com)
    robot.set_joint_angle(np.random.random(robot.chain.num_chain_joints))
