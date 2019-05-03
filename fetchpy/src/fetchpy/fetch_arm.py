
import numpy
import os

import rospy

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes, Grasp
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import matrix_from_pose_msg, pose_msg_from_matrix


def getGraspPosture(dist_between_fingers):

    max_effort = 50.0

    trajectory = JointTrajectory()
    trajectory.joint_names = [
        "l_gripper_finger_joint",
        "r_gripper_finger_joint"]

    point1 = JointTrajectoryPoint()
    point1.positions = [dist_between_fingers / 2.0, dist_between_fingers / 2.0]
    point1.effort = [max_effort, max_effort]

    point1.time_from_start = rospy.Duration(4.0)
    trajectory.points.append(point1)

    return trajectory


class FetchArm(object):

    def __init__(self, kw_args):

        self.arm_configs = dict()
        self.arm_configs["tuck"] = [1.32, 1.40, -0.20, 1.72, 0.00, 1.66, 0.00]
        self.arm_configs["home"] = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
        self.arm_configs["gripper_to_left_side"] = [0.8960487578, 1.2837294439, -
                                                    1.1732834282, -1.5595816118, -2.8655984934, 1.49291230480, -0.6071420747]
        self.arm_configs["gripper_above_ear"] = [1.6029415869, -
                                                 0.0466674154, 0.0049053205, -
                                                 1.6754725580, -
                                                 0.0195304363, -
                                                 1.0237425102, -
                                                 3.0962350003]
        self.arm_configs["gripper_above_head_one"] = [1.5928230791, -
                                                      0.4425835811, -
                                                      0.0693093358, -
                                                      1.8613221167, 0.0657791998, -
                                                      0.6528428007, -
                                                      0.2047865835]
        self.arm_configs["gripper_above_head_two"] = [1.5930058738, -
                                                      0.5921686315, -
                                                      0.0735785515, -
                                                      1.7020496880, 0.0827851208, -
                                                      0.6634486310, -
                                                      0.0677483712]

        self.options = {"enable_moveit": False,
                        }
        for key in kw_args:
            if key in self.options:
                self.options[key] = kw_args[key]

        home_dir = os.path.expanduser("~")
        gazebo_models_path = home_dir + "/.gazebo/models"
        print("gazebo_models_path = %s" % (gazebo_models_path))

        topic = "arm_controller/follow_joint_trajectory"
        rospy.loginfo(
            "[FetchArm] Waiting for Action Client on '" +
            topic +
            "' ...")
        self.joint_trajectory_action = actionlib.SimpleActionClient(
            topic, FollowJointTrajectoryAction)
        self.joint_trajectory_action.wait_for_server()

        if self.options["enable_moveit"]:

            rospy.loginfo("[FetchArm] Initializing ROS MoveIt ...")

            self.planning_scene = PlanningSceneInterface("base_link")

            self.move_group = MoveGroupInterface("arm", "base_link")

            self.pickplace_interface = PickPlaceInterface(
                "arm", "gripper", verbose=True)

            topic = "clear_octomap"
            rospy.loginfo(
                "[FetchArm] Waiting for Service on '" +
                topic +
                "' ...")
            rospy.wait_for_service(topic)
            self.clear_octomap_service = rospy.ServiceProxy(topic, Empty)

            self.planning_scene.removeCollisionObject("ground_plane")
            self.planning_scene.addCube("ground_plane", 2, 0.0, 0.0, -1.0)
            self.planning_scene.waitForSync()

    def addModelToPlanningScene(self, model_pose, model_name, mesh_name):

        if not self.options["enable_moveit"]:
            print("[FetchArm] ERROR: ROS MoveIt is disabled")
            return

        mesh_file = None
        try:
            mesh_file = self.model_meshes[mesh_name]

        except Exception as e:
            print("ERROR: Invalid mesh_name '%s'" % (mesh_name))

            return
        print(
            "Attempting to add '%s' to the Planning Scene ...  (%s)" %
            (model_name, mesh_file))

        wait_for_update = True
        self.planning_scene.addMesh(
            model_name, model_pose, mesh_file, wait_for_update)

    def removeModelFromPlanningScene(self, model_name):

        if not self.options["enable_moveit"]:
            print("[FetchArm] ERROR: ROS MoveIt is disabled")
            return

        wait_for_update = True
        self.planning_scene.removeCollisionObject(model_name, wait_for_update)

    def clearPlanningScene(self):

        if not self.options["enable_moveit"]:
            print("[FetchArm] ERROR: ROS MoveIt is disabled")
            return

        self.planning_scene.clear()
        self.planning_scene.waitForSync()

    def clearOctoMap(self):

        if not self.options["enable_moveit"]:
            print("[FetchArm] ERROR: ROS MoveIt is disabled")
            return

        try:

            ret = self.clear_octomap_service()
            print("Service returned:")
            print(type(ret))
            print(ret)

            if isinstance(ret, EmptyResponse):
                return True
            else:
                rospy.logerr("Error clearing OctoMap")
                return False

        except Exception as e:
            rospy.logerr("Error calling service: %s", str(e))

    def moveToConfiguration(self, joint_angles):

        joint_names = ["shoulder_pan_joint",
                       "shoulder_lift_joint",
                       "upperarm_roll_joint",
                       "elbow_flex_joint",
                       "forearm_roll_joint",
                       "wrist_flex_joint",
                       "wrist_roll_joint"]

        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        point1 = JointTrajectoryPoint()
        point1.positions = joint_angles

        point1.time_from_start = rospy.Duration(1.0)
        trajectory.points.append(point1)

        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.joint_trajectory_action.send_goal(follow_goal)
        self.joint_trajectory_action.wait_for_result()

    def moveJointToAngleOffset(joint_index, angle_degrees):
        pass

    def planToConfiguration(self, joint_angles):

        if not self.options["enable_moveit"]:
            print("[FetchArm] ERROR: ROS MoveIt is disabled")
            return

        joint_names = ["shoulder_pan_joint",
                       "shoulder_lift_joint",
                       "upperarm_roll_joint",
                       "elbow_flex_joint",
                       "forearm_roll_joint",
                       "wrist_flex_joint",
                       "wrist_roll_joint"]

        planner_name = 'RRTConnectkConfigDefault'

        self.move_group.setPlannerId(planner_name)

        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(
                joint_names, joint_angles, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def planToNamedConfiguration(self, name):

        joint_angles = None
        try:
            joint_angles = self.arm_configs[name]
        except Exception as e:
            print("ERROR: Invalid joint configuration '%s'" % (name))

        self.planToConfiguration(joint_angles)

    def tuck(self):

        self.planToNamedConfiguration("tuck")

    def straight(self):

        self.planToNamedConfiguration("home")

    def planToEndEffectorPose(self, pose, gripper_frame_name="gripper_link"):

        if not self.options["enable_moveit"]:
            print("[FetchArm] ERROR: ROS MoveIt is disabled")
            return

        if gripper_frame_name == "gripper_link":

            T_gripper_wrt_base = matrix_from_pose_msg(pose)
            print("T_gripper_wrt_base:")
            print(T_gripper_wrt_base)
            T_gripper_wrt_wrist = numpy.eye(4)
            T_gripper_wrt_wrist[0, 3] = -0.166
            print(T_gripper_wrt_wrist)

            T_wrist_wrt_base = numpy.dot(
                T_gripper_wrt_base, T_gripper_wrt_wrist)
            print("\nT_wrist_wrt_base:")
            print(T_wrist_wrt_base)

            pose = pose_msg_from_matrix(T_wrist_wrt_base)

        elif gripper_frame_name == "wrist_roll_link":
            pass
        else:
            print("ERROR: Unsupported link name in planToEndEffectorPose()")
            return

        """
        pose = Pose(Point(1.148, -0.0055, -0.7737),
                    Quaternion(-0.0038, 0.0127, -0.0046, 0.9998))
        # error, says
        # arm[RRTConnectkConfigDefault]: Unable to sample any valid states for goal tree

        pose = Pose(Point(1.1279130219309674, 0.016179773930791393, 0.7761878976420075),
                    Quaternion(0.0015884147029364389,0.006862488118127173,0.007369907985205331,0.9999480324756128))
        """

        gripper_pose_stamped = PoseStamped()
        gripper_pose_stamped.header.frame_id = "base_link"

        gripper_pose_stamped.header.stamp = rospy.Time.now()

        gripper_pose_stamped.pose = pose

        gripper_frame = "wrist_roll_link"

        while not rospy.is_shutdown():
            self.move_group.moveToPose(gripper_pose_stamped, gripper_frame)
            result = self.move_group.get_move_action().get_result()

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

        """

        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Hello there!")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Arm goal in state: %s",
                             move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")

        """

    def waveArm(self):

        if not self.options["enable_moveit"]:
            print("[FetchArm] ERROR: ROS MoveIt is disabled")
            return

        gripper_frame = 'wrist_roll_link'

        gripper_poses = [Pose(Point(0.042, 0.384, 1.826),
                              Quaternion(0.173, -0.693, -0.242, 0.657)),
                         Pose(Point(0.047, 0.545, 1.822),
                              Quaternion(-0.274, -0.701, 0.173, 0.635))]

        gripper_pose_stamped = PoseStamped()
        gripper_pose_stamped.header.frame_id = 'base_link'

        while not rospy.is_shutdown():
            for pose in gripper_poses:

                gripper_pose_stamped.header.stamp = rospy.Time.now()

                gripper_pose_stamped.pose = pose

                self.move_group.moveToPose(gripper_pose_stamped, gripper_frame)
                result = self.move_group.get_move_action().get_result()

                if result:

                    if result.error_code.val == MoveItErrorCodes.SUCCESS:
                        rospy.loginfo("Hello there!")
                    else:

                        rospy.logerr(
                            "Arm goal in state: %s",
                            self.move_group.get_move_action().get_state())
                else:
                    rospy.logerr("MoveIt! failure no result returned.")

    def graspObject(self, object_name, pose, enable_gripper=False):

        if not self.options["enable_moveit"]:
            print("[FetchArm] ERROR: ROS MoveIt is disabled")
            return

        print(
            "Attempting to pick-up object '%s' located on '' ..." %
            (object_name))

        grasps = list()

        msg = Grasp()
        msg.id = "grasp0"

        object_to_attach = "no_object"
        if enable_gripper:
            msg.pre_grasp_posture = getGraspPosture(0.11)
            msg.grasp_posture = getGraspPosture(0.0)
            object_to_attach = object_name

        msg.grasp_pose = PoseStamped()
        msg.grasp_pose.pose = pose
        msg.grasp_pose.header.frame_id = "gearbox_bottom"

        planning_frame = "/base_link"

        min_approach_distance = 0.02
        desired_approach_distance = 0.06

        min_distance = 0.3
        max_distance = 0.50

        msg.pre_grasp_approach.direction.header.frame_id = planning_frame
        msg.pre_grasp_approach.direction.vector.z = -1
        msg.pre_grasp_approach.min_distance = min_approach_distance
        msg.pre_grasp_approach.desired_distance = desired_approach_distance

        msg.post_grasp_retreat.direction.header.frame_id = planning_frame
        msg.post_grasp_retreat.direction.vector.z = 1
        msg.post_grasp_retreat.min_distance = min_distance
        msg.post_grasp_retreat.desired_distance = max_distance

        msg.max_contact_force = 25.0

        msg.allowed_touch_objects = ["gearbox_bottom", "table3"]

        grasps.append(msg)

        support_surface_name = "table3"
        planner_name = 'RRTConnectkConfigDefault'

        self.pickplace_interface.pickup(
            object_to_attach,
            grasps,
            support_name=support_surface_name,
            planner_id=planner_name)
