from ft import ft
import numpy as np

# f = frame_transformation()


a = ft.from_q([ 0.247404, 0, 0, 0.9689124 ])
b = ft.trans(2,3,4)

print(a)
print(b)

print(np.dot(a, b))