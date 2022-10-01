import os.path as osp
import numpy as np
from numpy import sin, cos, sign
from numpy.random import randn


class RLS:
    # tau=A*theta, solve for theta
    # tau: num_tau; A: (sample_num*num_tau)*num_vars; theta: num_vars
    def __init__(self, num_vars=$num_theta, delta=1):
        '''
        num_vars: number of variables including constant
        lam: forgetting factor, usually very close to 1.
        '''
        self.num_vars = num_vars
        
        # delta controls the initial state.
        self.P = delta*np.matrix(np.identity(self.num_vars))
        self.theta = np.zeros(self.num_vars)
        
        # error
        self.J = 0.
        self.meanJ = 0.
        self.meanJ_list = []
        
        # Count of number of observations added
        self.num_obs = 0

    def add_obs(self, A, tau):
        '''
        Add the observation A with label tau.
        A is a matrix with dimension num_tau*num_vars
        tau is a numpy array with dimension num_tau
        '''
        num_tau = A.shape[0]
        for row in range(num_tau):
            x = A[row,:]
            tau_row = tau[row]
            s = float(np.matmul(np.matmul(x, self.P), x.T)) + 1.0
            Inn = tau_row - np.matmul(x, self.theta)
            K = np.matmul(self.P, x) / s
            self.P -= np.matmul(K.T, K) * s
            self.theta += np.squeeze(np.asarray(K)) * Inn
            self.J += Inn**2/s
            
        self.num_obs += 1
        self.meanJ = self.J / self.num_obs
        self.meanJ_list.append(self.meanJ)

    def cal_theta_err(self, real_theta):
        return np.linalg.norm(self.theta-real_theta)

    def predict(self, A):
        # Predict the value of observation A. A should be a numpy matrix (col vector)
        return np.matmul(A, self.theta)

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

    X = np.zeros(($num_joints, len(theta_name)))
    for index, name in enumerate(theta_name):
        for i in range(1, $num_joints+1):
            tmp = 0
            try:
                tmp = eval("DG"+str(i)+name)
            except:
                pass
            X[i-1, index] = tmp

    return X


if __name__ =="__main__":
    my_rls = RLS(num_vars = $num_theta)
    print("base parameters number = ", $num_theta)
    # my_rls.add_obs(A, tau)
    # my_rls.predict(A)
