import numpy as np
import os.path as osp

class CODEGEN:
    def __init__(self, robot) -> None:
        self.robot = robot
    
    def fk_python_code_gen(self):
        file_full_path = osp.dirname(osp.abspath(__file__))
        with open(osp.join(file_full_path, 'fk_python_code_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        MDHs_string = "MDHs = {0}".format(np.array2string(self.robot.MDH_params).replace("\n", ","))
        content = content.replace("MDHs = [[1.57, 0, 0, 0], [0, 0.149, 1.527, 0.144]]", MDHs_string, 1)
        return content
    
    def jacobian_python_code_gen(self):
        return "import numpy as np"
