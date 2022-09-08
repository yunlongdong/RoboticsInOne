import numpy as np
import os.path as osp

class CODEGEN:
    def __init__(self, robot) -> None:
        self.file_full_path = osp.dirname(osp.abspath(__file__))
        self.robot = robot
    
    def fk_python_code_gen(self):
        with open(osp.join(self.file_full_path, 'template/fk_python_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        
        # 替换base link2world
        first_joint = list(self.robot.robotjoints.values())[0]
        base2world_rpy_string = "base2world_rpy = {0}".format(np.array2string(first_joint.rpy, separator=","))
        content = content.replace("base2world_rpy = []", base2world_rpy_string, 1)
        base2world_xyz_string = "base2world_xyz = {0}".format(np.array2string(first_joint.xyz, separator=","))
        content = content.replace("base2world_xyz = []", base2world_xyz_string, 1)
        # 替换MDH参数
        MDHs_string = "MDHs = {0}".format(np.array2string(self.robot.MDH_params, separator=",").replace("\n", "\n\t\t\t"))
        content = content.replace("MDHs = [[]]", MDHs_string, 1)
        # local_pos默认替换为最后一个link的质心
        CoM_string = "local_pos = {0}".format(np.array2string(list(self.robot.robotlinks.values())[-1].com, separator=","))
        content = content.replace("local_pos = []", CoM_string, 1)
        # 生成pybullet测试代码
        self.check_fk_gen()
        return content
    
    def jacobian_python_code_gen(self):
        with open(osp.join(self.file_full_path, 'template/jacobian_python_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        
        # 替换base link2world
        first_joint = list(self.robot.robotjoints.values())[0]
        base2world_rpy_string = "base2world_rpy = {0}".format(np.array2string(first_joint.rpy, separator=","))
        content = content.replace("base2world_rpy = []", base2world_rpy_string, 1)
        base2world_xyz_string = "base2world_xyz = {0}".format(np.array2string(first_joint.xyz, separator=","))
        content = content.replace("base2world_xyz = []", base2world_xyz_string, 1)
        # 替换MDH参数
        MDHs_string = "MDHs = {0}".format(np.array2string(self.robot.MDH_params, separator=",").replace("\n", "\n\t\t\t"))
        content = content.replace("MDHs = [[]]", MDHs_string, 1)
        # local_pos默认替换为最后一个link的质心
        CoM_string = "local_pos = {0}".format(np.array2string(list(self.robot.robotlinks.values())[-1].com, separator=","))
        content = content.replace("local_pos = []", CoM_string, 1)
        # 生成pybullet测试代码
        self.check_jacobian_gen()
        return content

    def check_fk_gen(self):
        with open(osp.join(self.file_full_path, 'template/check_fk_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        
        urdf_parser_path = osp.dirname(osp.abspath(osp.join(osp.abspath(__file__), "../")))
        print(urdf_parser_path)
        content = content.replace("sys.path.append(r'../')", "sys.path.append(r'{0}')".format(urdf_parser_path))
        content = content.replace("$filename", self.robot.urdf_file)
        check_code_path = osp.dirname(self.robot.urdf_file)
        with open(osp.join(check_code_path, "generated_check_fk.py"), "w") as f:
            f.write(content)

    def check_jacobian_gen(self):
        pass