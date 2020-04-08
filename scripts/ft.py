import numpy as np
import tf2_ros
from scipy.spatial.transform import Rotation as RotM

ft = np.identity(4)


class frame_t():
    def __init__(self):
        # self.F = np.identity(4)
        # self.P = self.F[0:3, 3].reshape(3, 1)
        # self.R = self.F[0:3, 0:3]
        pass

    def __str__(self):
        return np.array_str(self.F)

    def __updateFrameR(self):
        self.F[0:3, 0:3] = self.R

    def __updateFrameP(self):
        self.F[0:3, 3] = self.P.reshape(1, 3)

    def __mul__(self, other):
        if isinstance(other, frame_t):
            return np.dot(self, other)
        else:
            # raise TypeError(f"sorry, don't know how to multiply by {type(other).__name__}")
            pass

    def from_q(self, q):
        F = np.identity(4)
        r = RotM.from_quat(q)
        F[0:3, 0:3] = r.as_dcm()
        return F

    def from_euler(self, e):
        r = RotM.from_euler('xyz', e)  # rpy
        self.R = r.as_dcm()
        self.__updateFrameR()


    def from_tf(self, tf):
        tl = tf.transform.translation
        q = tf.transform.rotation

        F_R = np.identity(4)
        F_T = np.identity(4)
        r = RotM.from_quat([-q.x, -q.y, -q.z, q.w])
        F_R[0:3, 0:3] = r.as_dcm()
        F_T[0:3, 3] = np.array([-tl.x, -tl.y, -tl.z])

        return np.dot(F_R, F_T)

    def trans(self, x, y, z):
        F = np.identity(4)
        F[0:3, 3] = np.array([x, y, z])
        return F

    def to_array(self):
        return self.F


ft = frame_t()


# if __name__ == '__main__':
#     ft = frame_transformation()
#     print(ft.trans(1,2,3)))
