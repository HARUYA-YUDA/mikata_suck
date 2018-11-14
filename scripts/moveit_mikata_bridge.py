import rospy
import moveit_msgs.msg
from math import pi
from mikata_arm_msgs.msg import dxl_double

class move_arm():
    def __init__(self):
        #motor's actual angles 
        self._actual_pose = [0, 0, 0, 0]
        #motor's goal angles 
        self._goal_pose = [0, 0, 0, 0]

        #motor adject his angle by angle's error 
        #less than motor_angle_threshold
        self.motor_angle_threshold = 0.0174533
        self. error = self.motor_angle_threshold

        #It's for reading topic of actual robotic arm pose
        rospy.Subscriber("dxl/joint_state", dxl_double, actual_arm_cb)

        #It's for reading topic of IK solution
        rospy.Subscriber("move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, move_arm_cb)

        #It's for sending topic to mikata_bringup
        dxl_goal_publisher = rospy.Publisher('dxl/goal_position', dxl_double, que_size = 100)
        self.dxl_goal_publisher = dxl_goal_publisher

    def actual_arm_cb(self, data):
        #Update actual pose
        self._actual_pose = data

    def move_arm_cb(self, data):
        #Compare arm's actual pose and goal pose, and move
        #motors while motors' angle are bigger than threshold.

        #Subscribe the IK solution, and update _goal_pose
        self._goal_pose = data.trajectory.joint_trajectory.points.positions

        for i in range(len(self._goal_pose)):
            dxl_goal_publisher.publish(self._goal_pose)
            for j in range(len(self._actual_pose)):
                while not rospy.is_shutdown() & angle_error[i]>motor_angle_threshold[i]:
                    rospy.sleep(10.)


if __name__ == '__main__':
    pass
