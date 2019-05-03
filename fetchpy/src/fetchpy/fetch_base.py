

from math import sin, cos
import time

import rospy

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import Twist, Vector3


from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


RADIANS_TO_DEGREES = 57.2957795129


class FetchBase(object):

    def __init__(self, kw_args):

        self.options = {"enable_move_base_action": False,
                        "enable_base_controller_closed_loop": False,
                        }
        for key in kw_args:

            if key in self.options:
                self.options[key] = kw_args[key]

        if self.options["enable_move_base_action"]:

            topic = "move_base"
            rospy.loginfo(
                "[FetchBase] Waiting for Action Client on '" +
                topic +
                "' ...")
            self.move_base_action = actionlib.SimpleActionClient(
                topic, MoveBaseAction)
            self.move_base_action.wait_for_server()

        if self.options["enable_base_controller_closed_loop"]:

            from fetch_base_controller.srv import SetPidGains, MoveDistance, MoveDistanceRequest

            rospy.loginfo(
                "[FetchBase] Using custom plugin DiffDriveBaseControllerClosedLoop")

            topic = "/base_controller/command"
            self.cmd_vel_pub = rospy.Publisher(topic, Twist, queue_size=10)

            topic = "/base_controller/command_service"
            rospy.loginfo(
                "[FetchBase] Waiting for Service on '" +
                topic +
                "' ...")
            rospy.wait_for_service(topic)
            self.base_controller_command_service = rospy.ServiceProxy(
                topic, MoveDistance)

            topic = "/base_controller/set_pid_gains"
            rospy.loginfo(
                "[FetchBase] Waiting for Service on '" +
                topic +
                "' ...")
            rospy.wait_for_service(topic)
            self.base_controller_set_pid_gains_service = rospy.ServiceProxy(
                topic, SetPidGains)
            try:

                ret = self.base_controller_set_pid_gains_service(
                    0.6, 0.0, 0.05)
            except Exception as e:

                rospy.logerr("Error calling service '%s'" % (topic))

        else:

            topic = "/cmd_vel"
            self.cmd_vel_pub = rospy.Publisher(topic, Twist, queue_size=10)

    def planToPosition(self, x, y, theta, frame="map"):

        if not self.options["enable_move_base_action"]:
            print("[FetchBase] ERROR: 'move_base_action' is disabled")
            return

        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta / 2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta / 2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        self.move_base_action.send_goal(move_goal)
        self.move_base_action.wait_for_result()

    def rotateDegrees(self, angular_velocity, distance_degrees):

        if not self.options["enable_base_controller_closed_loop"]:
            print(
                "[FetchBase] ERROR: not using custom controller 'DiffDriveBaseControllerClosedLoop'")
            return

        track_width = 0.37476

        msg = MoveDistanceRequest()
        msg.rotate = True
        msg.angular_velocity = angular_velocity
        msg.distance_degrees = distance_degrees

        try:

            ret = self.base_controller_command_service(msg)

            degrees_moved = ret.left_distance_moved * \
                (2 / 0.37476) * RADIANS_TO_DEGREES
            print("Robot moved distance: %f degrees" % (degrees_moved))

        except Exception as e:

            rospy.logerr(
                "Error calling service: Maybe you did not specify all "
                "the required parameters in the Request msg?")

    def rotateLeft(self, distance_degrees):
        self.rotateDegrees(1.0, distance_degrees)

    def rotateRight(self, distance_degrees):
        self.rotateDegrees(-1.0, distance_degrees)

    def driveStraight(self, distance_meters):

        if not self.options["enable_base_controller_closed_loop"]:
            print(
                "[FetchBase] ERROR: not using custom controller 'DiffDriveBaseControllerClosedLoop'")
            return

        msg = MoveDistanceRequest()
        msg.drive_straight = True
        msg.linear_velocity = 0.5
        msg.distance_meters = distance_meters

        try:

            ret = self.base_controller_command_service(msg)

            print(
                "Robot moved distance: %f meters" %
                (ret.left_distance_moved))

        except Exception as e:

            rospy.logerr(
                "Error calling service: Maybe you did not specify all "
                "the required parameters in the Request msg?")

    def rotateForTime(self, angular_velocity, time_seconds):

        msg = Twist()
        msg.linear = Vector3(0, 0, 0)
        msg.angular = Vector3(0, 0, angular_velocity)

        delta_t = 0.01
        start_time = time.time()

        while (1):
            self.cmd_vel_pub.publish(msg)

            current_time = time.time()
            if (current_time - start_time) >= time_seconds:
                break
            time.sleep(delta_t)

    def rotateLeftForTime(self, time_seconds):
        self.rotateForTime(1.0, time_seconds)

    def rotateRightForTime(self, time_seconds):
        self.rotateForTime(-1.0, time_seconds)
