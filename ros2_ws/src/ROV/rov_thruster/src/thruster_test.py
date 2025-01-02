import numpy as np
from scipy.spatial.transform import Rotation as R

# Headless mode, always goes foward no matter the orientation
rpy_to_quat = R.from_euler('xyz', np.radians([0, 0, 45]))
quaternion = rpy_to_quat.as_quat()  # Returns [x, y, z, w]

rotation_matrix = rpy_to_quat.as_matrix()
# print(quaternion, rotation_matrix)

wrench = np.array([[10, 0, 0, 0, 0, 0]]).T

force_rotated = rotation_matrix @ wrench[0:3]
torque_rotated = rotation_matrix @ wrench[3:]

wrench_rotated = np.vstack((force_rotated, torque_rotated))

print(np.round(wrench_rotated))


CoM = np.array([0.0, 0.0, 0.0])

thruster_locations = np.array([[1, 1, 1],
                               [1, -1, 1],
                               [-1, 1, -1],
                               [-1, -1, -1],
                               [0, 1, 0],
                               [0, -1, 0]])

xyz = thruster_locations + CoM

deg = 45
s = np.sin(np.radians(deg))
c = np.cos(np.radians(deg))
f_uvw = np.array([[0, s, -c],
                  [0, -s, -c],
                  [0, s, c],
                  [0, -s, c],
                  [1, 0, 0],
                  [1, 0, 0]])
     


T_pqr = np.cross(xyz, f_uvw).T
print(T_pqr)

A = np.vstack((f_uvw.T, T_pqr))
print(A)
Ainv = np.linalg.pinv(A)
print(np.round(Ainv, 2))
# print(np.round(A.T@np.linalg.inv(A@A.T), 2))
K = np.diag(np.ones(len(f_uvw)))

print(np.round(Ainv @ wrench, 2))

print(np.round(Ainv @ wrench_rotated, 2))

