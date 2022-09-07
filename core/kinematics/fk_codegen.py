import numpy as np
import os.path as osp

class CODEGEN:
    def __init__(self, robot) -> None:
        self.file_full_path = osp.dirname(osp.abspath(__file__))
        self.robot = robot
    
    def fk_python_code_gen(self):
        with open(osp.join(self.file_full_path, 'fk_python_code_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        
        # 替换base link2world
        first_joint = list(self.robot.robotjoints.values())[0]
        base2world_rpy_string = "base2world_rpy = {0}".format(np.array2string(first_joint.rpy, separator=","))
        content = content.replace("base2world_rpy = [0, 0, 0.]", base2world_rpy_string, 1)
        base2world_xyz_string = "base2world_xyz = {0}".format(np.array2string(first_joint.xyz, separator=","))
        content = content.replace("base2world_xyz = [0, 0, 0.]", base2world_xyz_string, 1)
        # 替换MDH参数
        MDHs_string = "MDHs = {0}".format(np.array2string(self.robot.MDH_params, separator=",").replace("\n", "\n\t\t\t"))
        content = content.replace("MDHs = [[1.57, 0, 0, -0.121], [0, -0.543, 0, 0.]]", MDHs_string, 1)
        # local_pos默认替换为最后一个link的质心
        CoM_string = "local_pos = {0}".format(np.array2string(list(self.robot.robotlinks.values())[-1].com, separator=","))
        content = content.replace("local_pos = [0.1, 0.2, 0.3]", CoM_string, 1)
        return content
    
    def jacobian_python_code_gen(self):
        file_full_path = osp.dirname(osp.abspath(__file__))
        with open(osp.join(self.file_full_path, 'jacobian_python_code_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        
        # 替换base link2world
        first_joint = list(self.robot.robotjoints.values())[0]
        base2world_rpy_string = "base2world_rpy = {0}".format(np.array2string(first_joint.rpy, separator=","))
        content = content.replace("base2world_rpy = [0, 0, 0.]", base2world_rpy_string, 1)
        base2world_xyz_string = "base2world_xyz = {0}".format(np.array2string(first_joint.xyz, separator=","))
        content = content.replace("base2world_xyz = [0, 0, 0.]", base2world_xyz_string, 1)
        # 替换MDH参数
        MDHs_string = "MDHs = {0}".format(np.array2string(self.robot.MDH_params, separator=",").replace("\n", "\n\t\t\t"))
        content = content.replace("MDHs = [[1.57, 0, 0, -0.121], [0, -0.543, 0, 0.]]", MDHs_string, 1)
        # local_pos默认替换为最后一个link的质心
        CoM_string = "local_pos = {0}".format(np.array2string(list(self.robot.robotlinks.values())[-1].com, separator=","))
        content = content.replace("local_pos = [0.1, 0.2, 0.3]", CoM_string, 1)
        return content
