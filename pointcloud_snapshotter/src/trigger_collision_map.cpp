/*
 *  A smart trigger to initialize the ROS Collision Map.
 *  
 *  This is used in conjuction with the PointCloud snapshotter, and is run once
 *  while launching the perception system. It requests a single PointCloud
 *  snapshot every 1 sec until a message is published on '/collision_map_occ'.
 *  
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

#include "ros/ros.h"
#include <std_srvs/Empty.h> // Snapshot trigger Service

#include <arm_navigation_msgs/CollisionMap.h> // Collision Map

ros::Subscriber sub_collison_map;
bool received_map = false;

void callbackCollisionMap(const arm_navigation_msgs::CollisionMapPtr &msg)
{
    ROS_INFO("Smart trigger has received msg on Collision Map topic.");
    //printf("Received msg on Collision Map topic: \n");
    //printf("frame_id: %s \n", (msg->header.frame_id).c_str() );
    //printf("No. of boxes: %d \n", (msg->boxes).size() );

    received_map = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trigger_collision_map");

    // Precondition: Valid clock
    ros::NodeHandle nh;
    if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
    {
        ROS_FATAL("Timed-out waiting for valid time.");
        return EXIT_FAILURE;
    }

    // Wait for snapshot trigger Service and print message "...has not been advertised"
    std::string snapshot_service_name = nh.resolveName("snapshot_service");
    while (ros::ok() && !ros::service::exists(snapshot_service_name, true))
    {
        ros::Duration(2.0).sleep();
    }
    ROS_INFO_STREAM("PointCloud snapshotter Service is available.");

    // Subscribe to Collision Map topic
    std::string collision_map_topic = nh.resolveName("collision_map_output");
    sub_collison_map = nh.subscribe(collision_map_topic, 1, &callbackCollisionMap);
    ROS_INFO_STREAM("Subscribing to Collision Map on " << collision_map_topic);

    ROS_INFO("Triggering snapshots & waiting for Collision Map...");

    ros::Rate loop_rate(1); // trigger snapshot every 1 sec
    while (ros::ok() && received_map==false)
    {
        //ROS_INFO("Called snapshotter.");

        std_srvs::Empty srv;
        ros::service::call(snapshot_service_name, srv);

        ros::spinOnce();
        loop_rate.sleep();

    }
    //ROS_INFO("Collision Map has been initialized.");

    return EXIT_SUCCESS;
}

