import numpy as np
import os.path as osp
import shutil, re

import sys
sys.path.append("../../")
from core.urdf_parser.robot_from_urdf import *
from core.urdf_parser.utils import *
# symoro
from core.interfaces.symoro.symoroutils import parfile

class systemID_CODEGEN:
    def __init__(self, robot) -> None:
        self.file_full_path = osp.dirname(osp.abspath(__file__))
        self.robot = robot
        self.robotname = osp.splitext(osp.basename(robot.urdf_file))[0]
        self.par_filename = osp.abspath(osp.join(osp.dirname(robot.urdf_file), "generated_"+self.robotname+".par"))
        print("generated par_filename=", self.par_filename)
        self.symoro_par_gen()
        self.base_robot = None
        # self.basePara_code = self.basePara_python_codegen()
        # self.systemID_code = self.systemID_python_codegen()
        # self.check_basePara_code = self.check_basePara_codegen()
        # self.check_systemID_code = self.check_systemID_codegen()
    
    def symoro_basePara(self):
        # calculate base parameters
        self.symoro_robot, _  = parfile.readpar(self.robotname, self.par_filename)
        model_symo, base_robo = self.symoro_robot.compute_baseparams()
        old_file_path = model_symo.file_out.name
        new_file_path = osp.join(osp.dirname(self.par_filename), "generated_"+self.robotname+"_dim.txt")
        shutil.move(old_file_path, new_file_path)
        # 保存为新的.par文件
        base_robo.par_file_path = osp.join(osp.dirname(self.par_filename), "generated_"+self.robotname+"_base.par")
        parfile.writepar(base_robo)
        self.base_robot = base_robo


    def symoro_systemID(self):
        # calculate coefficients of base parameters
        self.symoro_robot, _  = parfile.readpar(self.robotname, self.par_filename)
        model_symo = self.symoro_robot.compute_dynidenmodel()
        old_file_path = model_symo.file_out.name
        new_file_path = osp.join(osp.dirname(self.par_filename), "generated_"+self.robotname+"_regp.txt")
        shutil.move(old_file_path, new_file_path)

    def basePara_python_codegen(self):
        self.symoro_basePara()
        pass
    
    def systemID_python_codegen(self):
        self.symoro_systemID()
        pass

    def check_basePara_codegen(self):
        pass

    def check_systemID_codegen(self):
        pass
    
    # The followings are utility functions
    def symoro_par_gen(self):
        with open(osp.join(self.file_full_path, 'template/symoro_template.par'),'r',encoding='utf-8') as f:
            content = f.read()
        
        index_list = [str(i+1) for i in range(self.robot.num_robotjoints)]
        content = content.replace("$robotname", self.robotname)
        content = content.replace("$num_joints", str(self.robot.num_robotjoints))
        content = content.replace("$zeros_nq", ','.join([str(0)] * self.robot.num_robotjoints))
        content = content.replace("$ones_nq", ','.join([str(1)] * self.robot.num_robotjoints))
        
        content = content.replace("$ant", ','.join([str(i) for i in range(self.robot.num_robotjoints)]))
        
        content = content.replace("$b", ','.join([str(0)] * self.robot.num_robotjoints))
        # alpha, a, theta, d in wikipedia
        # alpha, d, theta, r in symoro
        alpha = np.array2string(self.robot.MDH_params[:, 0], separator=',').replace(" ", "")[1:-1]
        d = np.array2string(self.robot.MDH_params[:, 1], separator=',').replace(" ", "")[1:-1]
        theta_value = self.robot.MDH_params[:, 2].tolist()
        theta_value = list(map(str, theta_value))
        theta = self.return_aggregated_list([theta_value, ['+q'],index_list])
        r = np.array2string(self.robot.MDH_params[:, 3], separator=',').replace(" ", "")[1:-1]
        content = content.replace("$alpha", alpha)
        content = content.replace("$d", d)
        content = content.replace("$theta", theta)
        content = content.replace("$r", r)

        content = content.replace("$QP", self.return_aggregated_list([['dq'],index_list]))
        content = content.replace("$QDP", self.return_aggregated_list([['ddq'],index_list]))

        inertia_list = ["XX", "XY", "XZ", "YY", "YZ", "ZZ"]
        for inertia_name in inertia_list:
            content = content.replace("$"+inertia_name, self.return_aggregated_list([['I'], index_list, [inertia_name.lower()]]))
        content = content.replace("$MX", self.return_aggregated_list([['c'],index_list, ["x*m"], index_list]))
        content = content.replace("$MY", self.return_aggregated_list([['c'],index_list, ["y*m"], index_list]))
        content = content.replace("$MZ", self.return_aggregated_list([['c'],index_list, ["z*m"], index_list]))
        content = content.replace("$M", self.return_aggregated_list([['m'],index_list]))
        content = content.replace("$FV", self.return_aggregated_list([['F'],index_list, ['v']]))
        content = content.replace("$FS", self.return_aggregated_list([['F'],index_list, ['s']]))


        with open(self.par_filename, 'w') as f:
            f.write(content)

    def return_aggregated_list(self, double_list):
        max_len = 0
        for single_list in double_list:
            max_len = max(max_len, len(single_list))
        result = [""] * max_len
        for i in range(max_len):
            for list_num in range(len(double_list)):
                if i >= len(double_list[list_num]):
                    result[i] += double_list[list_num][-1]
                else:
                    result[i] += double_list[list_num][i]
        return ','.join(result)

    def return_matrix_string(self, num_rows, num_cols, header, symoro_code):
        if num_cols == 1:
            matrix_string = np.empty(num_rows, dtype='<U9')
            for i in range(1, num_rows+1):
                    if header+str(i) not in symoro_code:
                        matrix_string[i-1] = '0'
                    else:
                        matrix_string[i-1] = header+str(i)
        else:
            matrix_string = np.empty([num_rows, num_cols], dtype='<U9')
            for i in range(1, num_rows+1):
                for j in range(1, num_cols+1):
                    max_ij = max(i, j)
                    min_ij = min(i, j)
                    if header+str(max_ij)+str(min_ij) not in symoro_code:
                        matrix_string[max_ij-1, min_ij-1] = '0'
                        matrix_string[min_ij-1, max_ij-1] = '0'
                    else:
                        matrix_string[max_ij-1, min_ij-1] = header+str(max_ij)+str(min_ij)
                        matrix_string[min_ij-1, max_ij-1] = header+str(max_ij)+str(min_ij)
        return np.array2string(matrix_string, separator=', ').replace("'", "")

if __name__ == "__main__":
    file_full_path = osp.dirname(osp.abspath(__file__))
    # robot = Robot(fileName=osp.join(file_full_path, '../../urdf_examples/half_exo/half_exo.urdf'))
    fileName = osp.join(file_full_path, '../../urdf_examples/3r/urdf/3r.urdf')
    robot = Robot(fileName=fileName)
    code_gen = systemID_CODEGEN(robot)
    code_gen.basePara_python_codegen()
    # code_gen.systemID_python_codegen()
    code_gen.check_basePara_codegen()
    code_gen.check_systemID_codegen()
    clean_folder(fileName)