

import numpy

import rospy
from geometry_msgs.msg import Point, Pose
import tf

from .world_gazebo import GazeboWorld

from .fetch_arm import FetchArm
from .fetch_base import FetchBase
from .fetch_head import FetchHead
from .fetch_gripper import FetchGripper
from .fetch_torso import FetchTorso
from .joint_state_listener import JointStateListener


class Robot(object):

    def __init__(self):

        self.joint_state_listener = JointStateListener(True)

        self.tf_sub = tf.TransformListener()
        self.tf_pub = tf.TransformBroadcaster()

    def getJointPosition(self, requested_names):
        return self.joint_state_listener.getJointPosition(requested_names)

    def getJointPositionVelocity(self, requested_names):
        return self.joint_state_listener.getJointPositionVelocity(
            requested_names)

    def waitUntilReachedJointConfig(
            self,
            joint_names,
            joint_configuration,
            angle_tolerance=None):
        return self.joint_state_listener.waitUntilReachedJointConfig(
            joint_names, joint_configuration, angle_tolerance)

    def getTransform(self, target_frame_name, source_frame_name="/base_link"):

        (trans, rot) = self.tf_sub.lookupTransform(
            source_frame_name, target_frame_name, rospy.Time(0))
        print("Got transform:")
        print(type(trans))
        print(type(rot))

        return (trans, rot)

    def addFrame(self, p, child_frame_name, parent_frame_name="map"):

        pose = Pose()
        print("Type of p: ", type(p))
        if isinstance(p, Point):
            pose.position = p
        elif isinstance(p, Pose):

            pose = p
        elif isinstance(p, numpy.ndarray):

            pose.position = p
        else:
            print("Point 'p' has unsupported type in addFrame(), ", type(p))

        self.tf_pub.sendTransform(
            (pose.position.x,
             pose.position.y,
             pose.position.z),
            (pose.orientation.x,
             pose.orientation.y,
             pose.orientation.z,
             pose.orientation.w),
            rospy.Time.now(),
            child_frame_name,
            parent_frame_name)


class FetchRobot(Robot, object):

    def __init__(self, kw_args):
        Robot.__init__(self)

        fetchbase_keys = [
            "enable_move_base_action",
            "enable_base_controller_closed_loop"]
        fetchbase_args = dict((k, kw_args[k])
                              for k in fetchbase_keys if k in kw_args)

        self.base = FetchBase(fetchbase_args)
        self.torso = FetchTorso(self)
        self.arm = FetchArm(kw_args)
        self.gripper = FetchGripper()
        self.head = FetchHead(self)

    def raiseTorso(self):
        self.torso.raiseTorso()

    def lowerTorso(self):
        self.torso.lowerTorso()


def initialize(sim=True, **kw_args):

    print("")

    robot_args = {"sim": True,
                  "enable_move_base_action": False,
                  "enable_base_controller_closed_loop": False,
                  "enable_moveit": True,
                  }

    print("Initializing ROS Node ...")
    rospy.init_node("fetchpy")

    print("Waiting for simulated time ...")
    while not rospy.Time.now():
        pass

    print("")

    world = GazeboWorld()

    robot = FetchRobot(robot_args)

    return (world, robot)
