

import math
import numpy
import os

import rospy

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState, GetWorldProperties
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from tf.transformations import identity_matrix, translation_matrix, rotation_matrix, quaternion_matrix, euler_from_quaternion, quaternion_from_euler

from moveit_python.geometry import pose_msg_from_matrix, matrix_from_pose_msg, inverse_matrix


class GazeboWorld(object):

    def __init__(self):

        topic = "/gazebo/get_world_properties"
        rospy.loginfo("Waiting for Service on '" + topic + "' ...")
        rospy.wait_for_service(topic)
        self.get_world_properties_service = rospy.ServiceProxy(
            topic, GetWorldProperties)

        topic = "/gazebo/get_model_state"
        rospy.loginfo("Waiting for Service on '" + topic + "' ...")
        rospy.wait_for_service(topic)
        self.get_model_state_service = rospy.ServiceProxy(topic, GetModelState)

        topic = "/gazebo/set_model_state"
        rospy.loginfo("Waiting for Service on '" + topic + "' ...")
        rospy.wait_for_service(topic)
        self.set_model_state_service = rospy.ServiceProxy(topic, SetModelState)

    def getModelNames(self):

        try:
            ret = self.get_world_properties_service()
            print type(ret)
            print(ret)

            if ret.success:
                return ret.model_names

        except Exception as e:
            rospy.logerr('Error on calling service: %s', str(e))

    def getModelPose(self, model_name):

        relative_entity_name = ""

        try:

            ret = self.get_model_state_service(
                model_name, relative_entity_name)
            print("Service returned:")

            print(ret)

            if ret.success:

                return ret.pose
            else:
                rospy.logerr(
                    "Model '%s' does not exist in Gazebo",
                    str(model_name))

                return None

        except Exception as e:
            rospy.logerr("Error on calling service: %s", str(e))

    def getModelPosition(self, model_name):

        pose = self.getModelPose(model_name)

        if pose is not None:
            return pose.position
        return None

    def getModelPoseRelativeRobot(self, model_name):

        model_pose = self.getModelPose(model_name)
        robot_pose = self.getModelPose("fetch")

        T_model = matrix_from_pose_msg(model_pose)
        T_robot = matrix_from_pose_msg(robot_pose)

        T = numpy.dot(numpy.linalg.inv(T_robot), T_model)

        numpy.set_printoptions(precision=3, suppress=True)

        return pose_msg_from_matrix(T)

    def getModelPoseRelativeMap(self, model_name):
        model_pose = self.getModelPose(model_name)

        robot_initial_pose = Pose()
        robot_initial_pose.position.x = 2.5
        robot_initial_pose.position.y = 3.1

        yaw = 3.14
        q = quaternion_from_euler(0.0, 0.0, yaw)
        robot_initial_pose.orientation = Quaternion(*q)

        robot_pose = robot_initial_pose

        print("model pose:")
        print(model_pose)
        print("robot pose:")
        print(robot_pose)

        T_model = matrix_from_pose_msg(model_pose)
        T_robot = matrix_from_pose_msg(robot_pose)

        T = numpy.dot(numpy.linalg.inv(T_robot), T_model)

        t_mat = translation_matrix(
            [-0.565856533138616, -0.29976501811487166, 0.0])
        print t_mat
        q_mat = quaternion_matrix(
            [0.0, 0.0, -0.7516058613647113, 0.659612484086081])
        print q_mat

        T_robot_wrt_map = numpy.dot(t_mat, q_mat)
        print("T_robot_wrt_map:")
        print(T_robot_wrt_map)

        numpy.set_printoptions(precision=3, suppress=True)

        return pose_msg_from_matrix(T)

    def setRobotPose(self, x, y, yaw_degrees):

        msg = ModelState()
        msg.model_name = "fetch"
        msg.reference_frame = "world"

        msg.pose.position = Point(x, y, 0.0)

        yaw = math.radians(yaw_degrees)
        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation = Quaternion(*q)

        try:
            ret = self.set_model_state_service(msg)

        except Exception as e:
            rospy.logerr('Error on calling service: %s', str(e))

    def setRobotPosition(self, x, y):
        self.setRobotPose(x, y, 0.0)

    def getGrasps(self):

        p = self.getModelPoseRelativeRobot("gearbox_bottom")
        T_part_wrt_robot = matrix_from_pose_msg(p)

        wrist_to_gripper_link_distance = 0.166
        gripper_link_to_tip_distance = 0.03
        dist = wrist_to_gripper_link_distance + gripper_link_to_tip_distance + 0.015

        T_offset = numpy.dot(translation_matrix([0.0, 0.0, dist]),
                             rotation_matrix(numpy.pi, (0, 0, 1)))

        T_offset = numpy.dot(T_offset,
                             rotation_matrix(numpy.pi / 2.0, (0, 1, 0)))

        return pose_msg_from_matrix(T_offset)

    def getDropPose2(self):

        p = self.getModelPoseRelativeRobot("bin2")
        T_bin_wrt_robot = matrix_from_pose_msg(p)

        T_offset = numpy.dot(translation_matrix([0.0, 0.0, 0.40]),
                             rotation_matrix(numpy.pi, (0, 0, 1)))

        T_offset = numpy.dot(T_offset,
                             rotation_matrix(numpy.pi / 2.0, (0, 1, 0)))

        T_wrist_wrt_bin = numpy.dot(T_bin_wrt_robot, T_offset)

        return pose_msg_from_matrix(T_wrist_wrt_bin)

    def getDropPose(self):

        p = self.getModelPoseRelativeRobot("bin2")
        T_bin_wrt_robot = matrix_from_pose_msg(p)

        T_offset = translation_matrix([-0.18, 0.0, 0.40])

        T_wrist_wrt_bin = numpy.dot(T_bin_wrt_robot, T_offset)

        return pose_msg_from_matrix(T_wrist_wrt_bin)
