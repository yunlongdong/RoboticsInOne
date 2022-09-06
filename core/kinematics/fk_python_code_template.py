from sympy import symbols, sin, cos, lambdify
from sympy.matrices import Matrix, eye
class FK_SYM:
    def __init__(self, MDHs) -> None:
        self.qs = [symbols('q{}'.format(i+1)) for i in range(len(MDHs))]
        self.mdhs = MDHs
        self.global_pos = eye(4)
        self.return_global_pos =  self.calulate_global_pos()

    def tf(self, index):
        """
        https://blog.csdn.net/Haiyang19980818/article/details/123948552
        """
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
    
    def calulate_global_pos(self):
        self.global_tf = eye(4)
        for i in range(len(self.qs)):
            self.global_tf = self.global_tf * self.tf(i)
        self.global_pos = self.global_tf * Matrix([symbols('x'), symbols('y'), symbols('z'), 1.])
        self.global_pos = self.global_pos[0:3]
        return_global_pos = lambdify([self.qs, ['x', 'y', 'z']], self.global_pos, "numpy")
        return return_global_pos


if __name__ == "__main__":
    MDHs = [[1.57, 0, 0, 0], [0, 0.149, 1.527, 0.144]]
    fk = FK_SYM(MDHs)

    qs = [0., 0.]
    local_pos = [0.1, 0.2, 0.3]
    global_pos = fk.return_global_pos(qs, local_pos)
    print(global_pos)