#!/usr/bin/env python
#[inport]
#----------------------------------------------------
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Point
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
#----------------------------------------------------

def all_close(goal, actual, tolerance):
    """
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal)is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


# Node mikata_suck class.
class mikata_suck(object):
    def __init__(self):
        super(mikata_suck, self).__init__()

        #initialize 'moveit_commander'_ and a 'rospy'_ node;
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('mikata_suck', anonymous=True)

        #Instantiate a 'RobotCommander'_ object
        robot = moveit_commander.RobotCommander()

        #Instantiate a 'PlanningSceneInterface'_ object.
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

        #initialize subscriber to get goal position
        pose_goal_sub = rospy.Subscriber('pose_goal', Point, self.ik_callback)

        planning_frame = move_group.get_planning_frame()
        print "======== Planning frame: %s" % planning_frame

        eef_link = move_group.get_end_effector_link()
        print "======== End effector link: %s" % eef_link

        group_names = robot.get_group_names()
        print "======== Available Planning Groups:", robot.get_group_names()

        #print robot state for debugging
        print "======== printing robot state"
        print robot.get_current_state()
        print ""

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_pose_goal(self,data):
        #Subscribe the pose goal
        #Calculate the inverse kinematic
        #Publish motor angles
        group = self.group
        #goal positions
        print "---------------------------------------"
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = data.x
        pose_goal.position.y = data.y
        pose_goal.position.z = data.z
        move_group.set_pose_target(pose_goal)

        print "    calculating inverse kinematic "
        print "---------------------------------------"
        print ""
        
        #calculate inverse kinematic
        plan = group.go(wait=True)
        #calling 'stop()' ensures that there is no residual movement
        group.stop()
        group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        group = self.group

        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.z -= scale * 0.1 # First move up (z)
        wpose.position.y += scale * 0.2 # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1 #second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        #Cartesian path is interpolated at a resolution of 1cm
        #Which is why I specify 0.01 as the eef_step in cartesian
        (plan, fraction) = group.compute_cartesian_path(
                                            waypoints, #waypoints to follow
                                            0.01,       #eef_step
                                            0.0)       #jump_threshold
        #It is just planning.
        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        #'DisplayTrajectory'_ msg has two primary fields, trajectory_start and trajectory.
        #It put trajectory_start current robot
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        #publish
        display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self,plan):
        group = self.group

        #robot follow the plan
        group.execute(plan, wait=True)

    def move_mikata_arm(self,plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

    def ik_callback(self,data):
        #Subscribe goal position topic named /pose_goal specified in geometry_msgs/Point.msg
        #Calculate Inverse Kinematic using mikata_arm model
        #Publish solution topic named /Display_planned_path
        print ""
        self.go_to_pose_goal(data)
        self.plan_cartesian_path()
        self.display_trajectory(cartesian_plan)

        print ""
        print "---------------------------------------"
        print " published topic /display_planned_path "
        print "---------------------------------------"
        print ""

if __name__ == '__main__':
    ik = mikata_suck()
    rospy.spin()

