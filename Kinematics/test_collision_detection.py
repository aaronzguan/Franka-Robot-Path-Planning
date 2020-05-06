import rospy
import math
import numpy as np
import tf
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, InteractiveMarkerFeedback
from franka_robot import FrankaRobot
import os


def lin_interp(start, goal, num_samples):
    interp = np.empty((num_samples, 0))

    for i in range(np.array(start).shape[0]):
        interp = np.concatenate([interp, np.linspace(start[i], goal[i], num_samples).reshape((-1, 1))], axis=1)

    return interp


class RobotStatePublisher():

    def __init__(self):
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.box_sub = rospy.Subscriber('/basic_controls/feedback', InteractiveMarkerFeedback, self.box_callback)

        rospy.init_node('joint_state_publisher')
        self.hz = 10
        self.rate = rospy.Rate(self.hz)  # 30 hz
        self.joint_state = JointState()
        self.joint_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
                                 'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2']
        self.joint_state.position = [0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4, 0.0, 0.0]

        self.red_cylinder = Marker()
        self.red_cylinder.header.frame_id = "panda_link0"
        self.red_cylinder.type = Marker.CYLINDER
        self.red_cylinder.pose.position.x = 0
        self.red_cylinder.pose.position.y = 0.3
        self.red_cylinder.pose.position.z = 0
        self.red_cylinder.color.a = 1.0
        self.red_cylinder.color.r = 1.0
        self.red_cylinder.color.g = 0.0
        self.red_cylinder.color.b = 0.0
        self.red_cylinder.scale.x = 0.1
        self.red_cylinder.scale.y = 0.1
        self.red_cylinder.scale.z = 0.1

        self.green_cylinder = Marker()
        self.green_cylinder.header.frame_id = "panda_link0"
        self.green_cylinder.type = Marker.CYLINDER
        self.green_cylinder.pose.position.x = 0
        self.green_cylinder.pose.position.y = 0.3
        self.green_cylinder.pose.position.z = 0
        self.green_cylinder.color.a = 1.0
        self.green_cylinder.color.r = 0.0
        self.green_cylinder.color.g = 1.0
        self.green_cylinder.color.b = 0.0
        self.green_cylinder.scale.x = 0.1
        self.green_cylinder.scale.y = 0.1
        self.green_cylinder.scale.z = 0.1

        self.dh_params = np.array([[0, 0.333, 0, 0],
                                   [0, 0, -math.pi / 2, 0],
                                   [0, 0.316, math.pi / 2, 0],
                                   [0.0825, 0, math.pi / 2, 0],
                                   [-0.0825, 0.384, -math.pi / 2, 0],
                                   [0, 0, math.pi / 2, 0],
                                   [0.088, 0, math.pi / 2, 0],
                                   [0, 0.107, 0, 0],
                                   [0, 0.1034, 0, 0]])
        self.fr = FrankaRobot('franka_robot.urdf', self.dh_params, 7)

        self.box = [0.5, 0, 0.21, 0, 0, 0, 0.15, 0.09, 0.126]

        self.ee_trajectory = np.empty((0, 6))
        self.joint_trajectory = np.empty((0, 7))
        self.joint_trajectory_index = 0
        self.num_joint_trajectory_points = 0

        if os.path.exists('joint_trajectory.npy'):
            self.joint_trajectory = np.load('joint_trajectory.npy')
            self.num_joint_trajectory_points = self.joint_trajectory.shape[0]
            print('Load joint_trajectory.npy as the joint_trajectory')
        else:
            self.calculate_joint_trajectory()
            np.save('joint_trajectory.npy', self.joint_trajectory)

    def calculate_joint_trajectory(self):
        current_joints = self.joint_state.position[:7]
        current_ee_pose = self.fr.ee(current_joints)
        desired_goal_pose_1 = np.copy(current_ee_pose)
        desired_goal_pose_1[0] += 0.1
        desired_goal_pose_1[1] += 0.1
        desired_goal_pose_2 = np.copy(current_ee_pose)
        desired_goal_pose_2[2] += 0.1
        desired_goal_pose_2[5] += math.pi / 2

        ee_trajectory_1 = lin_interp(current_ee_pose, desired_goal_pose_1, 3 * self.hz)
        ee_trajectory_2 = lin_interp(desired_goal_pose_1, desired_goal_pose_2, 3 * self.hz)
        ee_trajectory_3 = lin_interp(desired_goal_pose_2, current_ee_pose, 3 * self.hz)
        self.ee_trajectory = np.concatenate([ee_trajectory_1, ee_trajectory_2, ee_trajectory_3], axis=0)

        self.num_joint_trajectory_points = self.ee_trajectory.shape[0]
        self.joint_trajectory = np.zeros((self.num_joint_trajectory_points, 7))
        for i in range(self.num_joint_trajectory_points):
            print(i)
            self.joint_trajectory[i, :] = self.fr.inverse_kinematics(self.ee_trajectory[i, :], current_joints)
            current_joints = self.joint_trajectory[i, :]

    def box_callback(self, data):
        self.box[0] = data.pose.position.x
        self.box[1] = data.pose.position.y
        self.box[2] = data.pose.position.z

        euler = tf.transformations.euler_from_quaternion(
            [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

        self.box[3] = euler[0]
        self.box[4] = euler[1]
        self.box[5] = euler[2]

    def run(self):
        self.joint_state.position[:7] = self.joint_trajectory[self.joint_trajectory_index, :]
        self.joint_trajectory_index = (self.joint_trajectory_index + 1) % self.num_joint_trajectory_points

        if (self.fr.check_box_collision(self.joint_state.position, self.box)):
            self.red_cylinder.header.stamp = rospy.Time.now()
            self.marker_pub.publish(self.red_cylinder)
        else:
            self.green_cylinder.header.stamp = rospy.Time.now()
            self.marker_pub.publish(self.green_cylinder)

        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state_pub.publish(self.joint_state)

        self.rate.sleep()


if __name__ == '__main__':
    robot_state_publisher = RobotStatePublisher()

    while not rospy.is_shutdown():
        robot_state_publisher.run()