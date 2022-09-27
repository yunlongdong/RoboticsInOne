from sympy import symbols, sin, cos, lambdify
from sympy.matrices import Matrix, eye
from sympy.utilities.codegen import codegen

class FK_SYM:
    def __init__(self, base2world_rpy, base2world_xyz, MDHs) -> None:
        self.num_joints = len(MDHs)
        self.qs = [symbols('q{}'.format(i+1)) for i in range(self.num_joints)]
        self.mdhs = MDHs
        self.global_tf_list = []
        self.global_tf_list.append(self.get_extrinsic_tf(base2world_rpy, base2world_xyz)*self.tf(0))
        self.global_pos = None
        self.return_global_pos =  self.calulate_global_pos()

    def tf(self, index):
        """
        https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
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
        last_global_tf = self.global_tf_list[-1]
        for i in range(1, self.num_joints):
            self.global_tf_list.append(last_global_tf * self.tf(i))
            last_global_tf = self.global_tf_list[-1]
        self.global_pos = self.global_tf_list[-1] * Matrix([symbols('x'), symbols('y'), symbols('z'), 1.])
        # self.global_pos = self.global_pos[0:3]    
        return_global_pos = lambdify([self.qs, ['x', 'y', 'z']], self.global_pos, "numpy")
        return return_global_pos
    
    # the followings are utility functions
    def get_extrinsic_tf(self, rpy, xyz):
        tf = eye(4)
        x_rot = self.create_from_x_rotation(rpy[0])
        y_rot = self.create_from_y_rotation(rpy[1])
        z_rot = self.create_from_z_rotation(rpy[2])
        tf[:3, :3] = z_rot * y_rot * x_rot
        tf[:3, 3] = xyz
        return tf
    
    def create_from_x_rotation(self, theta):
        return Matrix(
            [[1.0, 0.0, 0.0],
             [0.0, cos(theta),-sin(theta)],
             [0.0, sin(theta), cos(theta)]]
        )
    def create_from_y_rotation(self, theta):
        return Matrix(
            [[cos(theta), 0.0, sin(theta)],
            [0.0, 1.0, 0.0],
            [-sin(theta), 0.0, cos(theta)]]
        )
    def create_from_z_rotation(self, theta):
        return Matrix(
            [[cos(theta),-sin(theta), 0.0],
            [sin(theta), cos(theta), 0.0],
            [0.0, 0.0, 1.0]]
        )

    def gencpp(self):
        [(c_name, c_code), (h_name, c_header)] = codegen(('fk', self.global_pos), 'c89')
        return c_code, c_header

if __name__ == "__main__":
    base2world_rpy = []
    base2world_xyz = []
    MDHs = [[]]
    fk = FK_SYM(base2world_rpy, base2world_xyz, MDHs)

    qs = [0.] * fk.num_joints
    local_pos = []
    global_pos = fk.return_global_pos(qs, local_pos)
    print("global_pos=", global_pos)
    