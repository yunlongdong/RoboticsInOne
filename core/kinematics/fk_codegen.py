import numpy as np
import os.path as osp

class CODEGEN:
    def __init__(self, robot) -> None:
        self.file_full_path = osp.dirname(osp.abspath(__file__))
        self.robot = robot
    
    def fk_python_code_gen(self):
        with open(osp.join(self.file_full_path, 'fk_python_code_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        MDHs_string = "MDHs = {0}".format(np.array2string(self.robot.MDH_params, separator=",").replace("\n", ""))
        content = content.replace("MDHs = [[1.57, 0, 0, 0], [0, 0.149, 1.527, 0.144]]", MDHs_string, 1)
        # local_pos默认替换为最后一个link的质心
        CoM_string = "local_pos = {0}".format(np.array2string(list(self.robot.robotlinks.values())[-1].com, separator=","))
        content = content.replace("local_pos = [0.1, 0.2, 0.3]", CoM_string, 1)
        return content
    
    def jacobian_python_code_gen(self):
        file_full_path = osp.dirname(osp.abspath(__file__))
        with open(osp.join(self.file_full_path, 'jacobian_python_code_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        MDHs_string = "MDHs = {0}".format(np.array2string(self.robot.MDH_params, separator=",").replace("\n", " "))
        content = content.replace("MDHs = [[1.57, 0, 0, 0], [0, 0.149, 1.527, 0.144]]", MDHs_string, 1)
        # local_pos默认替换为最后一个link的质心
        CoM_string = "local_pos = {0}".format(np.array2string(list(self.robot.robotlinks.values())[-1].com, separator=","))
        content = content.replace("local_pos = [0.1, 0.2, 0.3]", CoM_string, 1)
        return content
