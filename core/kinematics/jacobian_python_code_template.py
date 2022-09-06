from sympy import symbols, sin, cos, lambdify, zeros
from sympy.matrices import Matrix, eye
from sympy.vector import cross
class JAC_SYM:
    def __init__(self, MDHs) -> None:
        self.qs = [symbols('q{}'.format(i+1)) for i in range(len(MDHs))]
        self.mdhs = MDHs
        self.jacobian = zeros(6, len(self.qs))
        self.global_tf_list = []
        self.z_list = []
        self.o_oc = []
        self.return_jacobian = self.calulate_global_jacobian()
        
    def tf(self, index):
        mdh = self.mdhs[index]
        alpha, a, theta, d = mdh
        theta += self.qs[index]

        T = eye(4)
        T[0, 0] = cos(theta)
        T[0, 1] = -sin(theta)
        T[0, 3] = a

        T[1, 0] = sin(theta)*cos(alpha)
        T[1, 1] = cos(theta)*cos(alpha)
        T[1, 2] = -sin(alpha)
        T[1, 3] = -d*sin(alpha)

        T[2, 0] = sin(theta)*sin(alpha)
        T[2, 1] = cos(theta)*sin(alpha)
        T[2, 2] = cos(alpha)
        T[2, 3] = d*cos(alpha)
        return T
    
    def calulate_global_jacobian(self):
        local_pos = Matrix([symbols('x'), symbols('y'), symbols('z'), 1.])
        self.global_tf = eye(4)
        for i in range(len(self.qs)):
            self.global_tf = self.global_tf * self.tf(i)
            self.global_tf_list.append(self.global_tf)
            self.z_list.append(self.global_tf[:3, 2])
        oc = self.global_tf_list[-1] * local_pos
        for i in range(len(self.qs)):
            o_oc = oc - self.global_tf_list[i][:, 3]
            o_oc = o_oc[:3, :]
            self.o_oc.append(o_oc)
            self.jacobian[:3, i] = self.skew_symmetric_matrix(self.z_list[i]) * self.o_oc[i]
            self.jacobian[3:, i] = self.z_list[i]
        calulate_global_jacobian = lambdify([self.qs, ['x', 'y', 'z']], self.jacobian, "numpy")
        return calulate_global_jacobian

    def skew_symmetric_matrix(self, vec):
        a1, a2, a3 = vec[0], vec[1], vec[2]
        return Matrix([[0, -a3, a2],
                    [a3, 0, -a1],
                    [-a2, a1, 0]])

if __name__ == "__main__":
    MDHs = [[1.57, 0, 0, 0], [0, 0.149, 1.527, 0.144]]
    jac = JAC_SYM(MDHs)

    qs = [0., 0.]
    local_pos = [0.1, 0.2, 0.3]
    jacobian = jac.return_jacobian(qs, local_pos)
    print(jacobian)