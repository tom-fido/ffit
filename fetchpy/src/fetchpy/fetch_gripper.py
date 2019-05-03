

import rospy

import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


CLOSED_POS = 0.0
OPENED_POS = 0.10

MIN_EFFORT = 35
MAX_EFFORT = 100


class FetchGripper(object):

    def __init__(self):

        topic = "gripper_controller/gripper_action"
        rospy.loginfo("Waiting for Action Client on '" + topic + "' ...")
        self.client = actionlib.SimpleActionClient(topic, GripperCommandAction)
        self.client.wait_for_server()

    def open(self):
        goal = GripperCommandGoal()
        goal.command.position = OPENED_POS
        self.client.send_goal_and_wait(goal, rospy.Duration(10))

    def close(self, max_effort=MAX_EFFORT):
        goal = GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max_effort
        self.client.send_goal_and_wait(goal, rospy.Duration(10))
