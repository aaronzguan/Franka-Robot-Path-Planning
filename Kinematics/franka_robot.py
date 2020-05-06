import math
import numpy as np


def parse_urdf(urdf_file_path):
    '''
    TODO(Q3.1.1)

    Implement a urdf file parser and extract the origins and axes for each joint into numpy arrays.
    Arguments: string
    Returns: A dictionary with numpy arrays that contain the origins and axes of each joint
    origin = [x y z roll pitch yaw]
    '''

    num_joints = 0  # Change this to the number of times you encounter the word "axis" in the document

    urdf_file = open(urdf_file_path, "r")
    urdf_lines = urdf_file.readlines()
    for line in urdf_lines:
        if "axis" in line:
            num_joints += 1

    origin = np.zeros((num_joints + 1, 6))
    axis = np.zeros((num_joints + 1, 3))

    index = 0
    for line in urdf_lines:
        if "origin" in line:
            p = line.split('\"')
            rpy_list = p[1].split(' ')
            xyz_list = p[3].split(' ')
            if len(rpy_list) > 1 and len(xyz_list) > 1:
                for i, (trans, rot) in enumerate(zip(xyz_list, rpy_list)):
                    if '$' in rot:
                        rot = rot.replace('$', '').replace('{', '').replace('}', '')
                        nums = rot.split('/')
                        if '-' in nums[0]:
                            origin[index, i + 3] = -math.pi / float(nums[1])
                        else:
                            origin[index, i + 3] = math.pi / float(nums[1])
                    else:
                        origin[index, i + 3] = float(rot)

                    origin[index, i] = float(trans)
        if "axis" in line:
            p = line.split('\"')
            axis_list = p[1].split(' ')
            for i, ax in enumerate(axis_list):
                axis[index, i] = float(ax)
            index += 1
    # Since the end-effector transformation is not included in the franka urdf, I will manually provide
    # the transformation here for you from the flange frame.
    origin[-1, 2] = 0.1034

    return {'origin': origin, 'axis': axis}


class FrankaRobot():

    def __init__(self, urdf_file_path, dh_params, num_dof):

        self.robot_params = parse_urdf(urdf_file_path)
        self.dh_params = dh_params
        self.num_dof = num_dof
        self.desc_box_list = self.get_robot_description_as_box()

    def forward_kinematics_urdf(self, joints):
        '''
        TODO(Q3.1.2)

        Calculate the position of each joint using the robot_params
        Arguments: array of joint positions (rad)
        Returns: A numpy array that contains the 4x4 transformation matrices from the base to the position of each joint.
        '''

        axis = self.robot_params['axis']
        origin = self.robot_params['origin']
        forward_kinematics = np.zeros((axis.shape[0], 4, 4))
        last_T = np.identity(4)
        for i in range(len(origin)):
            x, y, z, roll, pitch, yaw = origin[i]
            translation_matrix = self.generate_translation_matrix(x,y,z)
            rotation_matrix = self.generate_rotation_matrix_from_rpy(roll, pitch, yaw)

            if i < len(joints):
                pos = joints[i]
                x_axis, y_axis, z_axis = axis[i]
                exp_map = np.asarray(([0, -z_axis, y_axis, 0],
                                [z_axis, 0, -x_axis, 0],
                                [-y_axis, x_axis, 0, 0],
                                [0, 0, 0, 0]))
                R = np.identity(4) + math.sin(pos) * exp_map + (1-math.cos(pos))*np.dot(exp_map, exp_map)
                # R = np.asarray(([math.cos(pos), -math.sin(pos), 0, 0],
                #                 [math.sin(pos), math.cos(pos), 0, 0],
                #                 [0, 0, 1, 0],
                #                 [0, 0, 0, 1]))
            else:
                R = np.identity(4)

            T = translation_matrix.dot(rotation_matrix).dot(R)
            T = np.dot(last_T, T)
            last_T = T
            forward_kinematics[i] = T

        return forward_kinematics

    def forward_kinematics_dh(self, joints):
        '''
        TODO(Q3.2.1)

        Calculate the position of each joint using the dh_params
        Arguments: array of joint positions (rad)
        Returns: A numpy array that contains the 4x4 transformation matrices from the base to the position of each joint.
        '''

        forward_kinematics = np.zeros((self.dh_params.shape[0], 4, 4))
        last_T = np.identity(4)
        for i in range(self.dh_params.shape[0]):
            a, d, alpha, _ = self.dh_params[i]
            if i < len(joints):
                theta = joints[i]
            else:
                theta = 0
            rotX = np.asarray(([1, 0, 0, 0],
                               [0, math.cos(alpha), -math.sin(alpha), 0],
                               [0, math.sin(alpha), math.cos(alpha), 0],
                               [0, 0, 0, 1]))
            transX = np.asarray(([1, 0, 0, a],
                                 [0, 1, 0, 0],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]))
            transZ = np.asarray(([1, 0, 0, 0],
                                 [0, 1, 0, 0],
                                 [0, 0, 1, d],
                                 [0, 0, 0, 1]))
            rotZ = np.asarray(([math.cos(theta), -math.sin(theta), 0, 0],
                               [math.sin(theta), math.cos(theta), 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]))
            T = rotX.dot(transX).dot(transZ).dot(rotZ)
            T = np.dot(last_T, T)

            last_T = T
            forward_kinematics[i] = T
        return forward_kinematics

    def ee(self, joints):
        '''
        TODO(Q3.2.2)

        Use one of your forward kinematics implementations to return the position of the end-effector.
        Arguments: array of joint positions (rad)
        Returns: A numpy array that contains the [x, y, z, roll, pitch, yaw] location of the end-effector.
        '''
        dh_fk = self.forward_kinematics_dh(joints)
        H = dh_fk[-1]
        x, y, z, roll, pitch, yaw = self.decompose_homogeneous_matrx(H)
        return np.array([x, y, z, roll, pitch, yaw])

    def jacobian(self, joints):
        '''
        TODO(Q4.1.1)

        Calculate the end-effector jacobian analytically using your forward kinematics
        Arguments: array of joint positions (rad)
        Returns: A numpy array that contains the 6 x num_dof end-effector jacobian.
        '''
        jacobian = np.zeros((6, self.num_dof))
        dh_fk = self.forward_kinematics_dh(joints)
        for i in range(self.num_dof):
            offset = dh_fk[-1][:-1, -1] - dh_fk[i][:-1, -1]
            angular_v = np.dot(dh_fk[i][:-1, :-1], [0, 0, 1])
            linear_v = np.cross(angular_v, offset)
            jacobian[:, i] = np.concatenate((linear_v, angular_v), axis=0)
        return jacobian

    def inverse_kinematics(self, desired_ee_pos, current_joints):
        '''
        TODO(Q5.1.1)

        Implement inverse kinematics using one of the methods mentioned in class.
        Arguments: desired_ee_pos which is a np array of [x, y, z, r, p, y] which represent the desired end-effector position of the robot
                   current_joints which represents the current location of the robot
        Returns: A numpy array that contains the joints required in order to achieve the desired end-effector position.
        '''
        joints = current_joints[:self.num_dof]
        current_ee_pos = self.ee(joints)
        ee_error = desired_ee_pos - current_ee_pos
        while np.linalg.norm(ee_error) > 1e-10:
            J = self.jacobian(joints)
            ## Jacobian Transpose Approach
            joints_error = 0.2 * np.dot(J.transpose(), ee_error)
            ## Pseudo-Inverse Approach, cannot work in this case
            # joints_error = np.dot(np.transpose(J), np.linalg.inv(np.dot(J, np.transpose(J)))).dot(ee_error)
            joints += joints_error
            current_ee_pos = self.ee(joints)
            ee_error = desired_ee_pos - current_ee_pos
        return joints

    def check_box_collision(self, joints, box):
        '''
        TODO(Q6.1.1)

        Implement collision checking with a box.
        Arguments: joints represents the current location of the robot
                   box contains the position of the center of the box [x, y, z, r, p, y] and the length, width, and height [l, w, h]
        Returns: A boolean where True means the box is in collision with the arm and false means that there are no collisions.
        '''
        x_box, y_box, z_box, roll_box, pitch_box, yaw_box, l_box, w_box, h_box = box
        rotation_matrix_box = self.generate_rotation_matrix_from_rpy(roll_box, pitch_box, yaw_box)
        Bx, By, Bz = self.get_local_axis(rotation_matrix_box)
        Bl, Bw, Bh = l_box / 2, w_box / 2, h_box / 2

        # Get the homogeneous matrix for each joint by using forward kinematics
        dh_fk = self.forward_kinematics_dh(joints)

        num_boxes = len(self.desc_box_list)
        in_collision = np.ones(num_boxes)

        # Interating over boxes to check if each of the box has collision with the input box
        for i, desc_box in enumerate(self.desc_box_list):
            x, y, z = desc_box["trans"]
            qw, qx, qy, qz = desc_box["rot"]
            l, w, h = desc_box["size"]

            # Get the translation and rotation matrix for local transformation
            translation_matrix = self.generate_translation_matrix(x, y, z)
            rotation_matrix = self.generate_rotation_matrix_from_quaternion(qw, qx, qy, qz)
            # Get the local transformation matrix
            T = np.dot(translation_matrix, rotation_matrix)
            # Get the homogeneous matrix for each description box referred to the base using forward kinematics
            H = np.dot(dh_fk[desc_box["link"] - 1], T)
            # Decompose the homogeneous matrix to get the current position and orientation referred to the base
            x, y, z, roll, pitch, yaw = self.decompose_homogeneous_matrx(H)

            Ax, Ay, Az = self.get_local_axis(rotation_matrix)
            Al, Aw, Ah = l / 2, w / 2, h / 2

            # Using seperating axis theorem for collision checking
            proj_axis = [Ax, Ay, Az, Bx, By, Bz, np.cross(Ax, Bx), np.cross(Ax, By), np.cross(Ax, Bz), np.cross(Ay, Bx),
                         np.cross(Ay, By), np.cross(Ay, Bz), np.cross(Az, Bx), np.cross(Az, By), np.cross(Az, Bz)]

            for axis in proj_axis:
                proj_origin_dist = abs(np.dot(np.asarray([x_box, y_box, z_box]) - np.asarray([x, y, z]), axis))

                proj_box_length = abs(np.dot(Al * Ax, axis)) + abs(np.dot(Aw * Ay, axis)) \
                                  + abs(np.dot(Ah * Az, axis)) + abs(np.dot(Bl * Bx, axis)) + \
                                  abs(np.dot(Bw * By, axis)) + abs(np.dot(Bh * Bz, axis))

                if proj_origin_dist > proj_box_length:
                    in_collision[i] = 0  # No collision for the i-th box
                    break

        if sum(in_collision) == 0:
            # All boxes have no collision
            return False
        else:
            print('The Box {} has collision'.format(*list(np.where(in_collision == 1)[0] + 1)))
            return True

    def get_robot_description_as_box(self):
        """
        :return A list of description boxes for collision checking
        """
        box1 = {"link": 1, "trans": np.asarray([-0.04, 0, - 0.283]), "rot": np.asarray([1, 0, 0, 0]), "size": np.asarray([0.23, 0.2, 0.1])}
        box2 = {"link": 1, "trans": np.asarray([-0.009, 0, -0.183]), "rot": np.asarray([1, 0, 0, 0]), "size": np.asarray([0.13, 0.12, 0.1])}
        box3 = {"link": 1, "trans": np.asarray([0, -0.032, -0.082]), "rot": np.asarray([0.9514, 0.3079, 0, 0]), "size": np.asarray([0.12, 0.1, 0.2])}
        box4 = {"link": 1, "trans": np.asarray([-0.008, 0, 0]), "rot": np.asarray([1, 0, 0, 0]), "size": np.asarray([0.15, 0.27, 0.11])}
        box5 = {"link": 1, "trans": np.asarray([0, 0.042, 0.067]), "rot": np.asarray([0.9514, 0.3079, 0, 0]), "size": np.asarray([0.12, 0.1, 0.2])}
        box6 = {"link": 3, "trans": np.asarray([0.00687, 0, -0.139]), "rot": np.asarray([1, 0, 0, 0]), "size": np.asarray([0.13, 0.12, 0.25])}
        box7 = {"link": 4, "trans": np.asarray([-0.008, 0.004, 0]), "rot": np.asarray([0.7071, -0.7071, 0, 0]), "size": np.asarray([0.13, 0.23, 0.15])}
        box8 = {"link": 5, "trans": np.asarray([0.00422, 0.05367, -0.121]), "rot": np.asarray([0.9962, -0.08715, 0, 0]), "size": np.asarray([0.12, 0.12, 0.4])}
        box9 = {"link": 5, "trans": np.asarray([0.00422, 0.00367, -0.263]), "rot": np.asarray([1, 0, 0, 0]), "size": np.asarray([0.12, 0.12, 0.25])}
        box10 = {"link": 5, "trans": np.asarray([0.00328, 0.0176, -0.0055]), "rot": np.asarray([1, 0, 0, 0]), "size": np.asarray([0.13, 0.23, 0.12])}
        box11 = {"link": 7, "trans": np.asarray([-0.0136, 0.0092, 0.0083]), "rot": np.asarray([0, 1, 0, 0]), "size": np.asarray([0.12, 0.12, 0.2])}
        box12 = {"link": 7, "trans": np.asarray([-0.0136, 0.0092, 0.1407]), "rot": np.asarray([0.9239, 0, 0, -0.3827]), "size": np.asarray([0.08, 0.22, 0.17])}

        return [box1, box2, box3, box4, box5, box6, box7, box8, box9, box10, box11, box12]

    def generate_translation_matrix(self, x, y, z):
        translation_matrix = np.asarray(([1, 0, 0, x],
                                         [0, 1, 0, y],
                                         [0, 0, 1, z],
                                         [0, 0, 0, 1]))
        return translation_matrix

    def generate_rotation_matrix_from_quaternion(self, qw, qx, qy, qz):
        rotation_matrix = np.asarray(
                ([1 - 2 * (qy ** 2) - 2 * (qz ** 2), 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw, 0],
                 [2 * qx * qy + 2 * qz * qw, 1 - 2 * (qx ** 2) - 2 * (qz ** 2), 2 * qy * qz - 2 * qx * qw, 0],
                 [2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * (qx ** 2) - 2 * (qy ** 2), 0],
                 [0, 0, 0, 1]))
        return rotation_matrix

    def generate_rotation_matrix_from_rpy(self, roll, pitch, yaw):
        """
        :param roll:  roll angle of the rotation matrix in rad
        :param pitch: pitch angle of the rotation matrix in rad
        :param yaw:  yaw angle of the rotation matrix in rad
        :return: A 4x4 rotation matrix
        """
        rotX = np.asarray(([1, 0, 0, 0],
                           [0, math.cos(roll), -math.sin(roll), 0],
                           [0, math.sin(roll), math.cos(roll), 0],
                           [0, 0, 0, 1]))
        rotY = np.asarray(([math.cos(pitch), 0, math.sin(pitch), 0],
                           [0, 1, 0, 0],
                           [-math.sin(pitch), 0, math.cos(pitch), 0],
                           [0, 0, 0, 1]))
        rotZ = np.asarray(([math.cos(yaw), -math.sin(yaw), 0, 0],
                           [math.sin(yaw), math.cos(yaw), 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]))

        return rotX.dot(rotY).dot(rotZ)

    def decompose_homogeneous_matrx(self, matrix):
        """
        :param matrix: A 4x4 homogeneous matrix related to the base of the robot
        :return: [x y z roll pitch yaw] of the current joint related to the base of the robot
        """
        x, y, z = matrix[:-1, -1]
        R = matrix[:-1, :-1]

        # roll = math.atan2(R[2, 1], R[2, 2])
        # pitch = math.atan2(-R[2, 0], (math.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
        # yaw = math.atan2(R[1, 0], R[0, 0])

        sy = math.sqrt(R[2, 1]**2 + R[2, 2]**2)
        singular = sy < 1e-6
        if not singular:
            if R[2, 2] < 1e-6:
                roll = math.pi / 2
            else:
                roll = math.atan2(R[2, 1], R[2, 2])
            if R[0, 0] < 1e-6:
                yaw = math.pi / 2
            else:
                yaw = math.atan2(R[1, 0], R[0, 0])
            pitch = math.atan2(-R[2, 0], sy)
        else:
            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.pi / 2
            yaw = math.atan2(R[1, 0], R[0, 0])

        return np.array([x, y, z, roll, pitch, yaw])

    def get_local_axis(self, rotation_matrix):
        """
        :param rotation_matrix:  4 x 4 homogeneous matrix
        :return: local x-axis, y-axis, and z-axis, unit vector, not homogeneous
        """
        x = rotation_matrix[:-1, 0]
        y = rotation_matrix[:-1, 1]
        z = rotation_matrix[:-1, 2]

        x = x / np.linalg.norm(x)
        y = y / np.linalg.norm(y)
        z = z / np.linalg.norm(z)

        return x, y, z
