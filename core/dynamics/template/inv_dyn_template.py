from numpy.random import random
import numpy as np
from numpy import sin, cos, sign

def return_elements(inertia_matrix):
    return inertia_matrix[0, 0], inertia_matrix[0, 1], inertia_matrix[0, 2], inertia_matrix[1, 1], inertia_matrix[1, 2], inertia_matrix[2, 2]

def return_jointtorque(qs, dqs, ddqs):
    _, $m_index = $mass_list_from_robot
    $q_index = qs
    $dq_index = dqs
    $ddq_index = ddqs
    g = -9.81
    base_rotation = $base_rotation
    gx, gy, gz = np.matmul(base_rotation.T, np.array([0, 0, g]))
    $set_Fs
    $set_Fv
    inertia_list = $inertia_list_from_robot
    $com_code
    $inertia_code

    $symoro_idm_code

    return np.array($Matrix)


if __name__ == "__main__":
    jointtorque = return_jointtorque(qs=$qs, dqs=$dqs, ddqs=$ddqs)
    print("joint torque=", jointtorque)
