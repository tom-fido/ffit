
import numpy

import rospy

import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import Point, Pose, Quaternion

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


PI = numpy.pi


class FetchHead(object):

    def __init__(self, robot):

        self.robot = robot

        topic = "head_controller/follow_joint_trajectory"
        rospy.loginfo("Waiting for Action Client on '" + topic + "' ...")
        self.joint_trajectory_action = actionlib.SimpleActionClient(
            topic, FollowJointTrajectoryAction)
        self.joint_trajectory_action.wait_for_server()

        topic = "head_controller/point_head"
        rospy.loginfo("Waiting for Action Client on '" + topic + "' ...")
        self.client = actionlib.SimpleActionClient(topic, PointHeadAction)
        self.client.wait_for_server()

    def moveToPosition(self, joint_names, joint_positions):

        head_joint_names = ["head_pan_joint", "head_tilt_joint"]

        head_joint_positions = [numpy.NaN, numpy.NaN]

        pan_joint_idx = -1
        tilt_joint_idx = -1
        for i in range(len(head_joint_names)):
            got_pos = False
            for j in range(len(joint_names)):

                if head_joint_names[i] == joint_names[j]:

                    got_pos = True
                    head_joint_positions[i] = joint_positions[j]

            if not got_pos:

                head_joint_positions[i] = self.robot.getJointPosition(
                    [head_joint_names[i]])

        trajectory = JointTrajectory()
        trajectory.joint_names = head_joint_names

        point1 = JointTrajectoryPoint()
        point1.positions = head_joint_positions

        point1.time_from_start = rospy.Duration(0.1)
        trajectory.points.append(point1)

        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        print("\nMoving head ... \n")

        self.joint_trajectory_action.send_goal_and_wait(
            follow_goal, rospy.Duration(10))

        self.joint_trajectory_action.wait_for_result()

    def straight(self):
        self.moveToPosition(["head_pan_joint", "head_tilt_joint"], [0.0, 0.0])

    def panLeft(self):
        self.moveToPosition(["head_pan_joint"], [PI / 2.0])

    def panRight(self):
        self.moveToPosition(["head_pan_joint"], [-PI / 2.0])

    def tiltUp(self):
        self.moveToPosition(["head_tilt_joint"], [-PI / 4.0])

    def tiltDown(self):
        self.moveToPosition(["head_tilt_joint"], [PI / 2.0])

    def lookAtPoint(self, point, frame="map", duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point = point
        goal.min_duration = rospy.Duration(duration)

        self.client.send_goal(goal)
        self.client.wait_for_result()

    def lookAtPosition(self, x, y, z, frame="map", duration=1.0):

        self.lookAtPoint(Point(x, y, z), frame, duration)
