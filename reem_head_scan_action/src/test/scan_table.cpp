/*
 *  A simple Action Client to call the head scan snapshot Action Server,
 *  to make the REEM robot scan a table with its head-mounted RGBD sensor.
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

// Simple Action Client, uses a empty message of type PointHeadAction
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

std::string SCAN_TABLE_SERVICE_NAME = "/head_traj_controller/head_scan_snapshot_action"; 

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class REEMHeadScanTest
{
  private:
    PointHeadClient* point_head_client_;  

  public:
    // Action client initialization
    REEMHeadScanTest()
    {
      // Initialize the client for the Action interface & spin a thread
      point_head_client_ = new PointHeadClient(SCAN_TABLE_SERVICE_NAME, true); 

      // Wait for the action server to come up 
      while(!point_head_client_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the '%s' Action Server to come up", SCAN_TABLE_SERVICE_NAME.c_str() );
      }
    }

    ~REEMHeadScanTest()
    {
      delete point_head_client_;
    }
 
  // Call the Action Server
  void move()
  {
    pr2_controllers_msgs::PointHeadGoal scan_table;

    ROS_INFO("Sending scan table goal...");
    point_head_client_->sendGoal(scan_table);

    point_head_client_->waitForResult();
    if(point_head_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    { 
      ROS_INFO("done!"); 
    }
    else 
    {
      ROS_INFO("The release action failed.");
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "head_scan_table_test");

  REEMHeadScanTest headscan;

  headscan.move();
  //ros::Duration(2.0).sleep();

  return 0;
}
