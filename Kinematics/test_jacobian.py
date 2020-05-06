import math
import numpy as np
from franka_robot import FrankaRobot

if __name__ == '__main__':
    dh_params = np.array([[0, 0.333, 0, 0],
                          [0, 0, -math.pi/2, 0],
                          [0, 0.316, math.pi/2, 0],
                          [0.0825, 0, math.pi/2, 0],
                          [-0.0825, 0.384, -math.pi/2, 0],
                          [0, 0, math.pi/2, 0],
                          [0.088, 0, math.pi/2, 0],
                          [0, 0.107, 0, 0],
                          [0, 0.1034, 0, 0]])
    fr = FrankaRobot('franka_robot.urdf', dh_params, 7)
    joints = np.array([0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4])
    jacobian = fr.jacobian(joints)

    sample_jacobian = np.array([[0,           0.153882052, 0,           0.1279, 0,      0.2104, 0],
                                [0.306890567, 0,           0.325815443, 0,      0.2104, 0,      0],
                                [0,          -0.306890567, 0,           0.472,  0,      0.088,  0],
                                [0,           0,          -0.707106781, 0,      1,      0,      0],
                                [0,           1,           0,          -1,      0,     -1,      0],
                                [1,           0,           0.707106781, 0,      0,      0,     -1]])

    print(np.allclose(jacobian, sample_jacobian))