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
	joints = [0, -math.pi/4, 0, -3*math.pi/4, 0, math.pi/2, math.pi/4]
	ee = fr.ee(joints)
	ee[1] += 0.01
	new_joints = fr.inverse_kinematics(ee, joints)
	new_ee = fr.ee(new_joints)

	print(np.allclose(new_ee,ee))