import rospy
import math
from sensor_msgs.msg import JointState
from franka_robot import FrankaRobot

def robot_state_publisher():
    joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(30) # 30 hz
    joint_state = JointState()
    joint_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2']
    joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    while not rospy.is_shutdown():
        joint_state.header.stamp = rospy.Time.now()
        joint_state_pub.publish(joint_state)
        rate.sleep()
if __name__ == '__main__':
    robot_state_publisher()