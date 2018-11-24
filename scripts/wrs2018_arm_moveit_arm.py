# -*- coding: utf-8 -*
import rospy
import sys
from mikata_arm_msgs.msg import dxl_double
from sensor_msgs.msg import JointState

def callback(msg):
	global pose_target
	pose_target = [
		msg.position[0],
		msg.position[1],
		msg.position[2],
		msg.position[3]]
	print pose_target

	positin = pose_target
	pose_posi = dxl_double()
	pose_posi = dxl_double()
	pose_posi.id = [1,2,3,4,5]
	pose_posi.data = [
					pose_target[0],
					pose_target[1],
					pose_target[2],
					pose_target[3],
					0.0]

	pub_pose.publish(pose_posi)
	rospy.sleep(0.1)

if __name__ == '__main__':
	rospy.init_node("moveit_move_arm")
	pub_pose = rospy.Publisher('/dxl/goal_position',dxl_double,queue_size = 100)
	rospy.Subscriber('/moveit/joint_states',JointState,callback)
	rospy.spin()

