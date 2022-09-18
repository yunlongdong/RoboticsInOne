import numpy as np
from numpy import sin, cos

def return_elements(inertia_matrix):
    return inertia_matrix[0, 0], inertia_matrix[0, 1], inertia_matrix[0, 2], inertia_matrix[1, 1], inertia_matrix[1, 2], inertia_matrix[2, 2]

def return_M(qs):
    _, $m_index = $mass_list_from_robot
    $q_index = qs
    inertia_list = $inertia_list_from_robot
    $com_code
    $inertia_code

    $symoro_M_code

    return np.array($Matrix)


if __name__ == "__main__":
    M = return_M(qs=$qs)
    print("M=", M)
