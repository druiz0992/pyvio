import numpy as np
import quaternion as q

q1 = q.quaternion(1,0,0,0)
R = q.as_rotation_matrix(q1)

print(q1, R)