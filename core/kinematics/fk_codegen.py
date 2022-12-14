import numpy as np
import os.path as osp

class fk_CODEGEN:
    def __init__(self, robot) -> None:
        self.file_full_path = osp.dirname(osp.abspath(__file__))
        self.robot = robot
        self.fk_code = self.fk_python_codegen()
        self.jacobian_code = self.jacobian_python_codegen()
        self.check_fk_code = self.check_fk_codegen()
        self.check_jacobian_code = self.check_jacobian_codegen()

    def fk_python_codegen(self, write=False):
        with open(osp.join(self.file_full_path, 'template/fk_python_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        
        # 替换base link2world
        root_joint = self.robot.return_root_joint()
        leave_link = self.robot.return_leave_link()
        base2world_rpy_string = "base2world_rpy = {0}".format(np.array2string(root_joint.rpy_MDH, separator=","))
        content = content.replace("base2world_rpy = []", base2world_rpy_string, 1)
        base2world_xyz_string = "base2world_xyz = {0}".format(np.array2string(root_joint.xyz_MDH, separator=","))
        content = content.replace("base2world_xyz = []", base2world_xyz_string, 1)
        # 替换MDH参数
        MDHs_string = "MDHs = {0}".format(np.array2string(self.robot.MDH_params, separator=",").replace("\n", "\n\t\t\t"))
        content = content.replace("MDHs = [[]]", MDHs_string, 1)
        # local_pos默认替换为最后一个link的质心
        CoM_pos_string = "local_pos = {0}".format(np.array2string(leave_link.com_MDH, separator=", "))
        CoM_rpy_string = "local_rpy = {0}".format(np.array2string(leave_link.rpy_MDH, separator=", "))
        content = content.replace("local_pos = []", CoM_pos_string, 1)
        content = content.replace("local_rpy = []", CoM_rpy_string, 1)
        return content
    
    def jacobian_python_codegen(self, write=False):
        with open(osp.join(self.file_full_path, 'template/jacobian_python_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        
        # 替换base link2world
        root_joint = self.robot.return_root_joint()
        leave_link = self.robot.return_leave_link()
        base2world_rpy_string = "base2world_rpy = {0}".format(np.array2string(root_joint.rpy_MDH, separator=","))
        content = content.replace("base2world_rpy = []", base2world_rpy_string, 1)
        base2world_xyz_string = "base2world_xyz = {0}".format(np.array2string(root_joint.xyz_MDH, separator=","))
        content = content.replace("base2world_xyz = []", base2world_xyz_string, 1)
        # 替换MDH参数
        MDHs_string = "MDHs = {0}".format(np.array2string(self.robot.MDH_params, separator=",").replace("\n", "\n\t\t\t"))
        content = content.replace("MDHs = [[]]", MDHs_string, 1)
        # local_pos默认替换为最后一个link的质心
        CoM_string = "local_pos = {0}".format(np.array2string(leave_link.com_MDH, separator=","))
        content = content.replace("local_pos = []", CoM_string, 1)
        return content

    def check_fk_codegen(self, write=False):
        with open(osp.join(self.file_full_path, 'template/check_fk_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        
        urdf_parser_path = osp.dirname(osp.abspath(osp.join(osp.abspath(__file__), "../")))
        content = content.replace("sys.path.append(r'../')", "sys.path.append(r'{0}')".format(urdf_parser_path))
        content = content.replace("$filename", self.robot.urdf_file)
        check_code_path = osp.dirname(self.robot.urdf_file)
        if write:
            with open(osp.join(check_code_path, "generated_check_fk.py"), "w") as f:
                f.write(content)
        return content
    
    def check_jacobian_codegen(self, write=False):
        with open(osp.join(self.file_full_path, 'template/check_jacobian_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        
        urdf_parser_path = osp.dirname(osp.abspath(osp.join(osp.abspath(__file__), "../")))
        content = content.replace("sys.path.append(r'../')", "sys.path.append(r'{0}')".format(urdf_parser_path))
        content = content.replace("$filename", self.robot.urdf_file)
        check_code_path = osp.dirname(self.robot.urdf_file)
        if write:
            with open(osp.join(check_code_path, "generated_check_jacobian.py"), "w") as f:
                f.write(content)
        return content
