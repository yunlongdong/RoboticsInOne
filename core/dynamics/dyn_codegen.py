import numpy as np
import os.path as osp
import shutil, re

import sys
sys.path.append("../../")
from core.urdf_parser.robot_from_urdf import *
# symoro
from core.interfaces.symoro.symoroutils import parfile

class dyn_CODEGEN:
    def __init__(self, robot) -> None:
        self.file_full_path = osp.dirname(osp.abspath(__file__))
        
        self.robot = robot
        self.robotname = osp.splitext(osp.basename(robot.urdf_file))[0]
        self.par_filename = osp.abspath(osp.join(osp.dirname(robot.urdf_file), "generated_"+self.robotname+".par"))
        print("generated par_filename=", self.par_filename)
        self.symoro_par_gen()
        # 使用symoro计算inm和idm
        self.symoro_dyn_M()
        self.idm_code = self.idm_python_codegen()
        self.M_code = self.M_python_codegen()
        self.check_idm_code = self.check_idm_codegen()
        self.check_M_code = self.check_M_codegen()

    def symoro_dyn_M(self):
        # calculate M
        self.symoro_robot, _  = parfile.readpar(self.robotname, self.par_filename)
        model_symo = self.symoro_robot.compute_inertiamatrix()
        old_file_path = model_symo.file_out.name
        new_file_path = osp.join(osp.dirname(self.par_filename), "generated_"+self.robotname+"_inm.txt")
        shutil.move(old_file_path, new_file_path)
        # calculate inverse dynamics
        self.symoro_robot, _  = parfile.readpar(self.robotname, self.par_filename)
        model_symo = self.symoro_robot.compute_idym()
        old_file_path = model_symo.file_out.name
        new_file_path = osp.join(osp.dirname(self.par_filename), "generated_"+self.robotname+"_idm.txt")
        shutil.move(old_file_path, new_file_path)

    def idm_python_codegen(self, write=False):
        idm_file_path = osp.join(osp.dirname(self.par_filename), "generated_"+self.robotname+"_idm.txt")
        pat = re.compile("Equations:"+'(.*?)'+"\*=\*", re.S)
        with open(idm_file_path, 'r') as f:
            idm_content = f.read()
        symoro_idm_code = pat.findall(idm_content)[0].replace(";", "")
        symoro_idm_code = symoro_idm_code.replace("\n", "\n    ")
        # 替换
        with open(osp.join(self.file_full_path, 'template/inv_dyn_python_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        index_list = [str(i+1) for i in range(self.robot.num_robotjoints)]

        links_in_order = self.robot.return_links_in_order()
        root_link = self.robot.return_root_link()
        mass_list_from_robot = np.array([robotlink.mass for robotlink in links_in_order])
        # 替换数值
        content = content.replace("$mass_list_from_robot", np.array2string(mass_list_from_robot, separator=', '))
        inertia_list_from_robot = ["np.array("+np.array2string(robotlink.inertia_MDH, separator=', ').replace('\n', '\n'+' '*8)+")" for robotlink in links_in_order]
        inertia_list_from_robot = "[" + ', '.join(inertia_list_from_robot) + "]"
        content = content.replace("$inertia_list_from_robot", inertia_list_from_robot)
        base_rotation = get_extrinsic_rotation(root_link.rpy_MDH)[:3, :3]
        content = content.replace("$base_rotation", "np.array("+np.array2string(base_rotation, separator=', ').replace("\n", "\n"+' '*8)+")")

        # 替换数值
        content = content.replace("$qs", '[0.]*'+str(self.robot.num_robotjoints))
        content = content.replace("$dqs", '[0.]*'+str(self.robot.num_robotjoints))
        content = content.replace("$ddqs", '[0.]*'+str(self.robot.num_robotjoints))
        # 符号
        content = content.replace("$m_index", self.return_aggregated_list([['m'], index_list]))
        content = content.replace("$q_index", self.return_aggregated_list([['q'], index_list]))
        content = content.replace("$dq_index", self.return_aggregated_list([['dq'], index_list]))
        content = content.replace("$ddq_index", self.return_aggregated_list([['ddq'], index_list]))
        content = content.replace("$set_Fs", ' = '.join(['F'+str(i)+'s' for i in range(1, self.robot.num_robotjoints+1)]) + '= 0.')
        content = content.replace("$set_Fv", ' = '.join(['F'+str(i)+'v' for i in range(1, self.robot.num_robotjoints+1)]) + '= 0.')
        com_code = ""
        inertia_code = ""
        for i in range(self.robot.num_robotjoints):
            com_code += "c{0}x, c{0}y, c{0}z = {1}\n    ".format(i+1, np.array2string(links_in_order[i].com_MDH, separator=', '))
            inertia_code += "I{0}xx, I{0}xy, I{0}xz, I{0}yy, I{0}yz, I{0}zz = return_elements(inertia_list[{0}])\n    ".format(i+1)
        content = content.replace("$com_code", com_code)
        content = content.replace("$inertia_code", inertia_code)
        content = content.replace("$symoro_idm_code", symoro_idm_code)
        content = content.replace("$Matrix", self.return_matrix_string(self.robot.num_robotjoints, 1, header='GAM', symoro_code=symoro_idm_code).replace("\n", "\n"+" "*12))
        check_code_path = osp.dirname(self.robot.urdf_file)
        if write:
            with open(osp.join(check_code_path, "generated_calculate_idm.py"), "w") as f:
                f.write(content)
        return content
    
    def check_idm_codegen(self, write=False):
        idm_file_path = osp.join(osp.dirname(self.par_filename), "generated_"+self.robotname+"_idm.txt")
        pat = re.compile("Equations:"+'(.*?)'+"\*=\*", re.S)
        with open(idm_file_path, 'r') as f:
            idm_content = f.read()
        symoro_idm_code = pat.findall(idm_content)[0].replace(";", "")
        symoro_idm_code = symoro_idm_code.replace("\n", "\n    ")
        
        # 替换
        with open(osp.join(self.file_full_path, 'template/check_idm_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        index_list = [str(i+1) for i in range(self.robot.num_robotjoints)]

        urdf_parser_path = osp.dirname(osp.abspath(osp.join(osp.abspath(__file__), "../")))
        content = content.replace("sys.path.append(r'../')", "sys.path.append(r'{0}')".format(urdf_parser_path))
        content = content.replace("$filename", self.robot.urdf_file)
        content = content.replace("$m_index", self.return_aggregated_list([['m'], index_list]))
        content = content.replace("$q_index", self.return_aggregated_list([['q'], index_list]))
        content = content.replace("$dq_index", self.return_aggregated_list([['dq'], index_list]))
        content = content.replace("$ddq_index", self.return_aggregated_list([['ddq'], index_list]))
        content = content.replace("$set_Fs", ' = '.join(['F'+str(i)+'s' for i in range(1, self.robot.num_robotjoints+1)]) + '= 0.')
        content = content.replace("$set_Fv", ' = '.join(['F'+str(i)+'v' for i in range(1, self.robot.num_robotjoints+1)]) + '= 0.')
        com_code = ""
        inertia_code = ""
        for i in range(self.robot.num_robotjoints):
            com_code += "c{0}x, c{0}y, c{0}z = links_in_order[{0}].com_MDH\n    ".format(i+1)
            inertia_code += "I{0}xx, I{0}xy, I{0}xz, I{0}yy, I{0}yz, I{0}zz = return_elements(inertia_list[{0}])\n    ".format(i+1)
        content = content.replace("$com_code", com_code)
        content = content.replace("$inertia_code", inertia_code)
        content = content.replace("$symoro_idm_code", symoro_idm_code)
        content = content.replace("$Matrix", self.return_matrix_string(self.robot.num_robotjoints, 1, header='GAM', symoro_code=symoro_idm_code).replace("\n", "\n"+" "*12))
        check_code_path = osp.dirname(self.robot.urdf_file)
        if write:
            with open(osp.join(check_code_path, "generated_check_idm.py"), "w") as f:
                f.write(content)
        return content
    
    def M_python_codegen(self, write=False):
        inm_file_path = osp.join(osp.dirname(self.par_filename), "generated_"+self.robotname+"_inm.txt")
        pat = re.compile("Equations:"+'(.*?)'+"\*=\*", re.S)
        with open(inm_file_path, 'r') as f:
            inm_content = f.read()
        symoro_M_code = pat.findall(inm_content)[0].replace(";", "")
        symoro_M_code = symoro_M_code.replace("\n", "\n    ")
        
        # 替换
        with open(osp.join(self.file_full_path, 'template/M_python_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        index_list = [str(i+1) for i in range(self.robot.num_robotjoints)]

        links_in_order = self.robot.return_links_in_order()
        root_joint = self.robot.return_root_joint()
        mass_list_from_robot = np.array([robotlink.mass for robotlink in links_in_order])
        content = content.replace("$mass_list_from_robot", np.array2string(mass_list_from_robot, separator=', '))
        inertia_list_from_robot = ["np.array("+np.array2string(robotlink.inertia_MDH, separator=', ').replace('\n', '\n'+' '*8)+")" for robotlink in links_in_order]
        inertia_list_from_robot = "[" + ', '.join(inertia_list_from_robot) + "]"
        content = content.replace("$inertia_list_from_robot", inertia_list_from_robot)

        content = content.replace("$qs", str([0.]*self.robot.num_robotjoints))
        content = content.replace("$m_index", self.return_aggregated_list([['m'], index_list]))
        content = content.replace("$q_index", self.return_aggregated_list([['q'], index_list]))
        com_code = ""
        inertia_code = ""
        for i in range(self.robot.num_robotjoints):
            com_code += "c{0}x, c{0}y, c{0}z = {1}\n    ".format(i+1, np.array2string(links_in_order[i].com_MDH, separator=', '))
            inertia_code += "I{0}xx, I{0}xy, I{0}xz, I{0}yy, I{0}yz, I{0}zz = return_elements(inertia_list[{0}])\n    ".format(i+1)
        content = content.replace("$com_code", com_code)
        content = content.replace("$inertia_code", inertia_code)
        content = content.replace("$symoro_M_code", symoro_M_code)
        content = content.replace("$Matrix", self.return_matrix_string(self.robot.num_robotjoints, self.robot.num_robotjoints, header='A', symoro_code=symoro_M_code).replace("\n", "\n"+" "*12))
        check_code_path = osp.dirname(self.robot.urdf_file)
        if write:
            with open(osp.join(check_code_path, "generated_calculate_M.py"), "w") as f:
                f.write(content)
        return content

    def check_M_codegen(self, write=False):
        inm_file_path = osp.join(osp.dirname(self.par_filename), "generated_"+self.robotname+"_inm.txt")
        pat = re.compile("Equations:"+'(.*?)'+"\*=\*", re.S)
        with open(inm_file_path, 'r') as f:
            inm_content = f.read()
        symoro_M_code = pat.findall(inm_content)[0].replace(";", "")
        symoro_M_code = symoro_M_code.replace("\n", "\n    ")
        
        # 替换
        with open(osp.join(self.file_full_path, 'template/check_M_template.py'),'r',encoding='utf-8') as f:
            content = f.read()
        index_list = [str(i+1) for i in range(self.robot.num_robotjoints)]

        urdf_parser_path = osp.dirname(osp.abspath(osp.join(osp.abspath(__file__), "../")))
        content = content.replace("sys.path.append(r'../')", "sys.path.append(r'{0}')".format(urdf_parser_path))
        content = content.replace("$filename", self.robot.urdf_file)
        content = content.replace("$m_index", self.return_aggregated_list([['m'], index_list]))
        content = content.replace("$q_index", self.return_aggregated_list([['q'], index_list]))

        content = content.replace("$q_index", self.return_aggregated_list([['q'], index_list]))
        com_code = ""
        inertia_code = ""
        for i in range(self.robot.num_robotjoints):
            com_code += "c{0}x, c{0}y, c{0}z = links_in_order[{0}].com_MDH\n    ".format(i+1)
            inertia_code += "I{0}xx, I{0}xy, I{0}xz, I{0}yy, I{0}yz, I{0}zz = return_elements(inertia_list[{0}])\n    ".format(i+1)
        content = content.replace("$com_code", com_code)
        content = content.replace("$inertia_code", inertia_code)
        content = content.replace("$symoro_M_code", symoro_M_code)
        content = content.replace("$Matrix", self.return_matrix_string(self.robot.num_robotjoints, self.robot.num_robotjoints, header='A', symoro_code=symoro_M_code).replace("\n", "\n"+" "*12))
        check_code_path = osp.dirname(self.robot.urdf_file)
        if write:
            with open(osp.join(check_code_path, "generated_check_M.py"), "w") as f:
                f.write(content)
        return content

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
    robot = Robot(fileName=osp.join(file_full_path, '../../urdf_examples/kuka iiwa/model.urdf'))
    code_gen = dyn_CODEGEN(robot)
    code_gen.check_M_codegen()
    code_gen.M_codegen()
    code_gen.check_dyn_codegen()
    code_gen.inv_dyn_codegen()