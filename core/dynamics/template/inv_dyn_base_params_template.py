from numpy.random import random
import numpy as np
from numpy import sin, cos, sign

import symengine
from symengine import var, sin, cos, sign, sqrt, zeros, eye, ones, Matrix
from sympy import simplify, nsimplify, lambdify
import sympy as sym
from sympy.utilities.codegen import codegen
from copy import deepcopy
import time


def return_elements(inertia_matrix):
    return inertia_matrix[0, 0], inertia_matrix[0, 1], inertia_matrix[0, 2], inertia_matrix[1, 1], inertia_matrix[1, 2], inertia_matrix[2, 2]

def returnTheta():
    # base parameters from URDF
    _, $m_index = $mass_list_from_robot
    $q_index = [0.] * $num_joints
    $set_Fs
    $set_Fv
    inertia_list = $inertia_list_from_robot
    $com_code
    $inertia_code


    $symoro_dim_code

    # $num_theta parameters
    $theta_para
    return theta

def split_M_C_G_fric(theta, tol=1e-10):
    # theta from numpy array to sympy matrix
    theta = Matrix(sym.Matrix(theta))
    num_joints = $num_joints
    qs = [var('q{}'.format(i+1)) for i in range(num_joints)]
    dqs = [var('dq{}'.format(i+1)) for i in range(num_joints)]
    ddqs = [var('ddq{}'.format(i+1)) for i in range(num_joints)]

    X = returnA(qs, dqs, ddqs)
    fric = X[:, -2*num_joints:] * theta[-2*num_joints:, 0]
    Mdqq_C_G = X[:, :-2*num_joints] * theta[:-2*num_joints, 0]

    # M+G 矩阵
    Mdqq_G = Mdqq_C_G.subs(dqs, [0.]*num_joints)
    # G矩阵
    G = Mdqq_G.subs(ddqs, [0.]*num_joints)
    # Mdqq
    Mdqq = Mdqq_G - G
    # C
    C = Mdqq_C_G.subs(ddqs, [0.]*num_joints) - G

    # M*ddq矩阵 -> M矩阵
    M = zeros(num_joints)
    start_time = time.time()
    Mdqq = Mdqq.expand()
    print("expanding time={0}...".format(time.time()-start_time))
    for i in range(Mdqq.shape[0]):
        start_time = time.time()
        Mdqq_ij = Mdqq[i, 0]
        print("num_term of joint {0} = {1}".format(i+1, len(Mdqq_ij.args)))
        for index, term in enumerate(Mdqq_ij.args):
            if index % 1e4 == 0:
                print("index={0}".format(index))
            clean_term = 0.
            if abs(term.args[0]) > tol:
                clean_term = term
            else:
                continue
            
            for k in range(num_joints):
                if ddqs[k] in clean_term.free_symbols:
                    M[i, k] += clean_term.subs(ddqs[k], 1.)
                    break
        print("time for calculating M of joint {0} = {1}...".format(i+1, time.time()-start_time))
    M = 0.5*(M + M.T)
    
    return M, C, G, fric

def returnA(qs, dqs, ddqs):
    $q_index = qs
    $dq_index = dqs
    $ddq_index = ddqs
    g = -9.81
    base_rotation = $base_rotation
    gx, gy, gz = np.matmul(base_rotation.T, np.array([0, 0, g]))

    $symoro_regp_code

    
    # $num_theta parameters
    $theta_name

    X = zeros($num_joints, len(theta_name))
    for index, name in enumerate(theta_name):
        for i in range(1, $num_joints+1):
            tmp = 0
            try:
                tmp = eval("DG"+str(i)+name)
            except:
                pass
            X[i-1, index] = tmp

    return X

def gencpp(matrix):
    [(c_name, c_code), (h_name, c_header)] = codegen(("matrix", matrix), 'c89')
    return c_code, c_header

if __name__ =="__main__":
    # 这里替换为辨识得到的theta
    theta = np.ones($num_theta)#returnTheta()
    # 计算M, C, G, fric矩阵
    M, C, G, fric = split_M_C_G_fric(theta)
    c_code, c_header = gencpp(sym.Matrix(M))
    print("M code=", c_code)