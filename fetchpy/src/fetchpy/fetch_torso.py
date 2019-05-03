

import rospy

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FetchTorso(object):

    def __init__(self, robot):

        self.robot = robot

        topic = "torso_controller/follow_joint_trajectory"
        rospy.loginfo("Waiting for Action Client on '" + topic + "' ...")
        self.joint_trajectory_action = actionlib.SimpleActionClient(
            topic, FollowJointTrajectoryAction)
        self.joint_trajectory_action.wait_for_server()

    def moveToPosition(self, position):

        if (position < 0.0) or (position > 0.4):
            print("ERROR: torso position must be in the range 0.0 to 0.4m")
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = ["torso_lift_joint"]

        point1 = JointTrajectoryPoint()
        point1.positions = [position]

        point1.time_from_start = rospy.Duration(0.1)
        trajectory.points.append(point1)

        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        print("\nMoving torso ... \n")

        self.joint_trajectory_action.send_goal(follow_goal)
        print("\nsent goal \n")

        self.joint_trajectory_action.wait_for_result()
        print("\nwaiting ... \n")

        position_tolerance = 0.01
        self.robot.waitUntilReachedJointConfig(
            trajectory.joint_names, point1.positions, position_tolerance)
        print("\ndone \n")

    def raiseTorso(self):

        max_torso_height = 0.39
        self.moveToPosition(max_torso_height)

    def lowerTorso(self):
        self.moveToPosition(0.0)
