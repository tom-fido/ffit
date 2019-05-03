/*
 *  An Action Server to move the head & take PointCloud snapshots
 *
 *  
 *  Used as part of the perception system, to provide an easy method to request
 *  the robot to scan an area and aggregate the snapshots of PointCloud data
 *  e.g. scanning a table with head-mounted RGBD sensor & building a Collision Map
 *  
 *  The same behaviour could be accomplished be repeatedly calling the Point Head 
 *  Action Server with some x-y co-ordinates, so this node offers a specific
 *  "scan the table" behaviour.
 *  
 *  
 *  This node should be launched in the same namespace as the 'follow_joint_trajectory'
 *  Action for the head, and provides the 'scan_table_action' Action.
 *  
 *  It is called using an empty Action message of type pr2_controllers_msgs::PointHeadAction
 *  
 *  When requested, the robot will perform the following hard-coded behaviour:
 *     1. look straight ahead, snapshot
 *     2. look left, snapshot
 *     3. look right, snapshot
 *     4. look down-left, snapshot
 *     5. look down-right, snapshot
 *     6. look down-center, snapshot
 *  
 *  After requesting each PointCloud snapshot, this node waits 2.5 sec because it seems there is some
 *  delay in matching the PointCloud data with its correct TF transform.
 *  
 *  
 *  ToDo: - Maybe the robot should finish looking straight ahead, then the user's code should move the 
 *         head back down at the table, or add multiple pre-programmed scan patterns.
 *        - Make some of the variables as ROS parameters e.g. the 2 sec delay after each snapshot.
 *        - Test the goal cancel callback, maybe the head should return to look straight ahead.
 *        - Test the watchdog callback, in case the head move fails.
 *
 *  Author: David Butterworth
 */

/*
 * Copyright (c) 2013, David Butterworth, PAL Robotics S.L.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

// Advanced Action Server, accepts empty PointHead Action Message
#include <actionlib/server/action_server.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

// Simple Action Client, connects to the JointTrajectoryAction for the head
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

// PointCloud snapshotter is triggered with an empty Service call
#include <std_srvs/Empty.h> 

// Trigger for PointCloud snapshotter, this gets remapped in launch file
std::string POINTCLOUD_SNAPSHOT_SERVICE_NAME = "snapshot_service"; 

class HeadScanSnapshotAction
{
private:
    // Action Server
    typedef actionlib::ActionServer<pr2_controllers_msgs::PointHeadAction> PHAS;
    typedef PHAS::GoalHandle GoalHandle;
    PHAS* action_server_;

    // Action Client for moving the head
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  *head_traj_action_client_;

    std::string node_name_;
    std::string action_name_;
    ros::NodeHandle nh_, pnh_;

    // state of the JointTrajectoryAction controller
    ros::Subscriber sub_controller_state_;
    pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr last_controller_state_;

    // watchdog, check for timeout
    ros::Timer watchdog_timer_;

    bool has_active_goal_;
    GoalHandle active_goal_;

    // PointCloud snapshot Service
    ros::ServiceClient get_pointcloud_snapshot_client_;

public:
    // Constructor
    HeadScanSnapshotAction(const ros::NodeHandle &node) :
        node_name_(ros::this_node::getName())
        , action_name_("head_scan_snapshot_action")
        , nh_(node)
        , pnh_("~")
        , has_active_goal_(false)
    {
        action_server_ = new actionlib::ActionServer<pr2_controllers_msgs::PointHeadAction>(nh_, action_name_,
                boost::bind(&HeadScanSnapshotAction::goalCB, this, _1) ,
                boost::bind(&HeadScanSnapshotAction::cancelCB, this, _1),
                false ); // false prevents race condition warnings

        // Check for JointTrajectoryAction from head controller, should be in the same namespace
        std::string head_action_name = nh_.resolveName("follow_joint_trajectory");
        head_traj_action_client_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(head_action_name, true);
        while(!head_traj_action_client_->waitForServer(ros::Duration(2.0)) && nh_.ok())
        {
            ROS_INFO_STREAM("Waiting for Action Server on " << head_action_name);
        }
        if (!nh_.ok()) exit(0);
        ROS_INFO_STREAM("Connected to JointTrajectoryAction Server.");

        // Wait for PointCloud snapshot Service
        while (!ros::service::waitForService(POINTCLOUD_SNAPSHOT_SERVICE_NAME, ros::Duration(2.0)) && nh_.ok() )
        {
            ROS_INFO("Waiting for PointCloud snapshotter service on %s", POINTCLOUD_SNAPSHOT_SERVICE_NAME.c_str() );
        }
        if (!nh_.ok()) exit(0);
        get_pointcloud_snapshot_client_ = nh_.serviceClient<std_srvs::Empty>(POINTCLOUD_SNAPSHOT_SERVICE_NAME, true);

        ROS_INFO_STREAM("Connected to PointCloud Snapshot Service.");

        // Start this Action Server
        action_server_->start();

        // Subscribe to state of JointTrajectoryAction controller
        sub_controller_state_ = nh_.subscribe("state", 1, &HeadScanSnapshotAction::controllerStateCB, this);

        // Monitor for Action timeout
        watchdog_timer_ = nh_.createTimer(ros::Duration(1.0), &HeadScanSnapshotAction::watchdog, this);
    }

    // Destructor
    ~HeadScanSnapshotAction()
    {
        // Delete Action Client
        delete head_traj_action_client_;
    }

    // Action handler
    void goalCB(GoalHandle gh)
    {
        // Cancel the currently active goal
        if (has_active_goal_)
        {
            active_goal_.setCanceled();
            has_active_goal_ = false;
        }

        gh.setAccepted();
        active_goal_ = gh;
        has_active_goal_ = true;

        // Define head trajectories:

        control_msgs::FollowJointTrajectoryGoal straight_ahead;
        straight_ahead.trajectory.points.resize(1); // 1 single waypoint
        straight_ahead.trajectory.joint_names.push_back("head_1_joint");
        straight_ahead.trajectory.joint_names.push_back("head_2_joint");
        straight_ahead.trajectory.points[0].positions.push_back(0.0);
        straight_ahead.trajectory.points[0].positions.push_back(0.0);
        straight_ahead.trajectory.points[0].time_from_start = ros::Duration(1.5); // velocity

        control_msgs::FollowJointTrajectoryGoal straight_ahead_left;
        straight_ahead_left.trajectory.points.resize(1); // 1 single waypoint
        straight_ahead_left.trajectory.joint_names.push_back("head_1_joint");
        straight_ahead_left.trajectory.joint_names.push_back("head_2_joint");
        straight_ahead_left.trajectory.points[0].positions.push_back(0.5);
        straight_ahead_left.trajectory.points[0].positions.push_back(0.0);
        straight_ahead_left.trajectory.points[0].time_from_start = ros::Duration(1.5); // velocity

        control_msgs::FollowJointTrajectoryGoal straight_ahead_right;
        straight_ahead_right.trajectory.points.resize(1); // 1 single waypoint
        straight_ahead_right.trajectory.joint_names.push_back("head_1_joint");
        straight_ahead_right.trajectory.joint_names.push_back("head_2_joint");
        straight_ahead_right.trajectory.points[0].positions.push_back(-0.5);
        straight_ahead_right.trajectory.points[0].positions.push_back(0.0);
        straight_ahead_right.trajectory.points[0].time_from_start = ros::Duration(1.5); // velocity

        control_msgs::FollowJointTrajectoryGoal down_center;
        down_center.trajectory.points.resize(1); // 1 single waypoint
        down_center.trajectory.joint_names.push_back("head_1_joint");
        down_center.trajectory.joint_names.push_back("head_2_joint");
        down_center.trajectory.points[0].positions.push_back(0.0);
        down_center.trajectory.points[0].positions.push_back(0.34); // tilt down, limit is 20 degrees or 0.349 radians
        down_center.trajectory.points[0].time_from_start = ros::Duration(1.5); // velocity

        control_msgs::FollowJointTrajectoryGoal down_left;
        down_left.trajectory.points.resize(1); // 1 single waypoint
        down_left.trajectory.joint_names.push_back("head_1_joint");
        down_left.trajectory.joint_names.push_back("head_2_joint");
        down_left.trajectory.points[0].positions.push_back(0.5); // left
        down_left.trajectory.points[0].positions.push_back(0.34);
        down_left.trajectory.points[0].time_from_start = ros::Duration(1.5); // velocity

        control_msgs::FollowJointTrajectoryGoal down_right;
        down_right.trajectory.points.resize(1); // 1 single waypoint
        down_right.trajectory.joint_names.push_back("head_1_joint");
        down_right.trajectory.joint_names.push_back("head_2_joint");
        down_right.trajectory.points[0].positions.push_back(-0.5); // right
        down_right.trajectory.points[0].positions.push_back(0.34);
        down_right.trajectory.points[0].time_from_start = ros::Duration(1.5); // velocity

        // Delay to wait after taking each snapshot, so TF can catch-up
        // ToDo: make this a ROS param
        double tf_delay_ = 2.5;

        std_srvs::Empty snapshot_call;

        // Look straight ahead, take snapshot
        ROS_INFO("Looking straight ahead...");
        head_traj_action_client_->sendGoal(straight_ahead);
        head_traj_action_client_->waitForResult();
        if(head_traj_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("done!");
        else
            ROS_ERROR("The action failed.");
        ros::Duration(tf_delay_).sleep();
        if (get_pointcloud_snapshot_client_.call(snapshot_call))
            ROS_INFO("Taking PointCloud snapshot from ASUS Xtion");
        else
            ROS_ERROR("Failed requesting snapshot from ASUS Xtion");

        // Look left, take snapshot
        ROS_INFO("Looking to the left...");
        head_traj_action_client_->sendGoal(straight_ahead_left);
        head_traj_action_client_->waitForResult();
        if(head_traj_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("done!");
        else
            ROS_ERROR("The action failed.");
        ros::Duration(tf_delay_).sleep();
        if (get_pointcloud_snapshot_client_.call(snapshot_call))
            ROS_INFO("Taking PointCloud snapshot from ASUS Xtion");
        else
            ROS_ERROR("Failed requesting snapshot from ASUS Xtion");

        // Look right, take snapshot
        ROS_INFO("Looking to the right...");
        head_traj_action_client_->sendGoal(straight_ahead_right);
        head_traj_action_client_->waitForResult();
        if(head_traj_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("done!");
        else
            ROS_ERROR("The action failed.");
        ros::Duration(tf_delay_).sleep();
        if (get_pointcloud_snapshot_client_.call(snapshot_call))
            ROS_INFO("Taking PointCloud snapshot from ASUS Xtion");
        else
            ROS_ERROR("Failed requesting snapshot from ASUS Xtion");

        // Look down & left, take snapshot
        ROS_INFO("Looking down & to the left...");
        head_traj_action_client_->sendGoal(down_left);
        head_traj_action_client_->waitForResult();
        if(head_traj_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("done!");
        else
            ROS_ERROR("The action failed.");
        ros::Duration(tf_delay_).sleep();
        if (get_pointcloud_snapshot_client_.call(snapshot_call))
            ROS_INFO("Taking PointCloud snapshot from ASUS Xtion");
        else
            ROS_ERROR("Failed requesting snapshot from ASUS Xtion");

        // Look down & right, take snapshot
        ROS_INFO("Looking down & to the right...");
        head_traj_action_client_->sendGoal(down_right);
        head_traj_action_client_->waitForResult();
        if(head_traj_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("done!");
        else
            ROS_ERROR("The action failed.");
        ros::Duration(tf_delay_).sleep();
        if (get_pointcloud_snapshot_client_.call(snapshot_call))
            ROS_INFO("Taking PointCloud snapshot from ASUS Xtion");
        else
            ROS_ERROR("Failed requesting snapshot from ASUS Xtion");

        // Look down at table, take snapshot
        ROS_INFO("Looking down at table...");
        head_traj_action_client_->sendGoal(down_center);
        head_traj_action_client_->waitForResult();
        if(head_traj_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("done!");
        else
            ROS_ERROR("The action failed.");
        ros::Duration(tf_delay_).sleep();
        if (get_pointcloud_snapshot_client_.call(snapshot_call))
            ROS_INFO("Taking PointCloud snapshot from ASUS Xtion");
        else
            ROS_ERROR("Failed requesting snapshot from ASUS Xtion");

        /*
        // Look straight ahead (again), take snapshot
        ROS_INFO("Looking straight ahead...");
        head_traj_action_client_->sendGoal(straight_ahead);
        head_traj_action_client_->waitForResult();
        if(head_traj_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          ROS_INFO("done!");
        else
          ROS_ERROR("The action failed.");
        ros::Duration(tf_delay_).sleep();
        if (get_pointcloud_snapshot_client_.call(snapshot_call))
          ROS_INFO("Taking PointCloud snapshot from ASUS Xtion");
        else
          ROS_ERROR("Failed requesting snapshot from ASUS Xtion");
        */

        // Action Goal is complete
        active_goal_.setSucceeded();
        has_active_goal_ = false;
    }

    // Watchdog, should detect if the Action is taking too long, or failed
    void watchdog(const ros::TimerEvent &e)
    {
        ros::Time now = ros::Time::now();

        // Aborts the active goal if the controller does not appear to be active.
        if (has_active_goal_)
        {
            bool should_abort = false;
            if (!last_controller_state_)
            {
                should_abort = true;
                ROS_WARN("Aborting goal because we have never heard a controller state message.");
            }
            else if ((now - last_controller_state_->header.stamp) > ros::Duration(5.0))
            {
                should_abort = true;
                ROS_WARN("Aborting goal because we haven't heard from the controller in %.3lf seconds",
                         (now - last_controller_state_->header.stamp).toSec());
            }

            if (should_abort)
            {
                // Stops the head trajectory controller.
                head_traj_action_client_->cancelGoal();

                // Marks the current goal as aborted.
                active_goal_.setAborted();
                has_active_goal_ = false;
            }
        }
    }

    // JointTrajectoryAction controller state callback
    // (gets the last time for the watchdog, we don't do any controller feedback)
    void controllerStateCB(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr &msg)
    {
        last_controller_state_ = msg;
        ros::Time now = ros::Time::now();
    }

    // Cancel Goal handler, not tested yet, maybe move head back to default position
    void cancelCB(GoalHandle gh)
    {
        if (active_goal_ == gh)
        {
            // Stops the head trajectory controller.
            head_traj_action_client_->cancelGoal();

            // Marks the current goal as canceled.
            active_goal_.setCanceled();
            has_active_goal_ = false;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "head_scan_snapshot_action");
    ros::NodeHandle node;
    HeadScanSnapshotAction scan(node);
    ros::spin();
    return 0;
}

