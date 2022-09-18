import numpy as np
import os.path as osp

class fk_CODEGEN:
    def __init__(self, robot) -> None:
        self.file_full_path = osp.dirname(osp.abspath(__file__))
        self.robot = robot
        self.basePara_code = self.basePara_python_codegen()
        self.systemID_code = self.systemID_python_codegen()
        self.check_basePara_code = self.check_basePara_codegen()
        self.check_systemID_code = self.check_systemID_codegen()
    
    def basePara_python_codegen(self):
        pass
    
    def systemID_python_codegen(self):
        pass

    def check_basePara_codegen(self):
        pass

    def check_systemID_codegen(self):
        pass