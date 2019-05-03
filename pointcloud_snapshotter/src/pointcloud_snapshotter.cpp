/*
 *  This node takes 1 'snapshot' from a PointCloud input and publishes it on some output topic, 
 *  when triggered by a ROS Service call.
 *
 *  
 *  The node can be launched in the 'started' state. It will check for the input PointCloud and try to subscribe
 *  idefinitely until it becomes available.
 *
 *  If the node is launched in the 'stopped' state, a Service call to 'start' will only attempt to subscribe
 *  to the input PointCloud for maximum 3 sec.
 *
 *  Making a 'snapshot' request will always check if the input PointCloud is available, otherwise it would
 *  simply re-publish the last frame of data before the input was lost.
 *  If the input is lost, calls to 'snapshot' will immediately start working again once the input cloud is
 *  available. There is an option in the code if you would prefer to 'start' the node again first.
 *
 *  The Service calls are always successful, because there is nothing in here that should fail.
 *  However, the 'snapshot' Service call will fail if the snapshotter is not running, or the input PointCloud
 *  is not available.
 *
 *  ToDo: - Return success on service response when it applies?
 *        - Merge all Service calls into one, using a custom msg?
 *        - Maybe add a Service call to enable the PointCloud for a longer period?
 */

/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Adolfo Rodriguez Tsouroukdissian, David Butterworth */

/// \brief Gate publishing of a pointcloud between input and output topics.
///
/// The output is updated on demand with snapshot requests. The following ROS API is exposed:
///  - \b start Service for starting the snapshotter.
///  - \b stop Service for stopping the snapshotter.
///  - \b snapshot Service that triggers a snapshot. Will trigger snapshot publishing only if the start service
///    has been called.
///  - \b point_cloud_in Topic containing input point cloud.
///  - \b point_cloud_out Topic containing snapshots of input point cloud.
///  - \b auto_start Parameter for configuring if the service should start y default.

// C++ standard
#include <string>
#include <iomanip>
#include <cstddef>

// ROS
#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudSnapshotter
{

public:
    PointCloudSnapshotter()
        : running_(false),
          point_cloud_in_msg_()
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        pointcloud_out_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_out", 1);

        take_snapshot_server_ = private_nh.advertiseService("snapshot", &PointCloudSnapshotter::publishSnapshot, this);
        start_server_         = private_nh.advertiseService("start",    &PointCloudSnapshotter::start, this);
        stop_server_          = private_nh.advertiseService("stop",     &PointCloudSnapshotter::stop, this);

        pointcloud_in_topic_  = nh.resolveName("point_cloud_in");
        pointcloud_out_topic_ = nh.resolveName("point_cloud_out");

        bool auto_start;
        private_nh.param<bool>("auto_start", auto_start, false);
        if (auto_start)
        {
            //if (!start()) throw;  // only check once for input cloud

            pointcloud_in_subscriber_ = ros::NodeHandle().subscribe("point_cloud_in", 1, &PointCloudSnapshotter::updateLatestPointCloud, this);
            ROS_INFO("Point cloud snapshotter waiting for %s", pointcloud_in_topic_.c_str() );

            // Wait indefinitely for input PointCloud
            bool first_time = false;
            while (ros::ok())
            {
                ros::Duration(0.5).sleep(); // Magic number, if topic already published it takes 0.5 sec to subscribe
                if ((pointcloud_in_subscriber_.getNumPublishers() < 1) && (first_time == false))
                {
                    ROS_WARN("No input data for point cloud snapshotter, will retry indefinitely..." );
                    first_time = true;
                }
                else if (pointcloud_in_subscriber_.getNumPublishers() > 0)
                {
                    break;
                }
            }

            running_ = true;
            ROS_INFO("Starting point cloud snapshotter.");
            //ROS_INFO("Starting point cloud snapshotter, listening on %s", pointcloud_in_topic_.c_str() );
        }
        else
        {
            ROS_INFO("Point cloud snapshotter node is ready to be started.");
        }
    }

private:
    bool running_;
    sensor_msgs::PointCloud2ConstPtr point_cloud_in_msg_; ///< Cached pointer to last received point cloud

    ros::Subscriber pointcloud_in_subscriber_; ///< Point cloud input
    ros::Publisher  pointcloud_out_publisher_; ///< Point cloud snapshot publisher

    ros::ServiceServer take_snapshot_server_; ///< Service server for taking snapshots
    ros::ServiceServer start_server_;         ///< Service server for starting the snapshotter
    ros::ServiceServer stop_server_;          ///< Service server for stopping the snapshotter

    std::string pointcloud_in_topic_; // resolved topic names
    std::string pointcloud_out_topic_;

    // Callback for input PointCloud
    void updateLatestPointCloud(sensor_msgs::PointCloud2::ConstPtr point_cloud_in_msg)
    {
        point_cloud_in_msg_ = point_cloud_in_msg;
    }

    // Service call: start
    bool start()
    {
        // Precondition
        if ((running_) && (pointcloud_in_subscriber_.getNumPublishers() > 1))
        {
            ROS_DEBUG("Point cloud snapshotter is already running.");
            return true;
        }

        // Attempt to subscribe to input PointCloud, but give up after 3 sec
        pointcloud_in_subscriber_ = ros::NodeHandle().subscribe("point_cloud_in", 1, &PointCloudSnapshotter::updateLatestPointCloud, this);
        for (int i=0; i<6; i++)
        {
            ros::Duration(0.5).sleep(); // Magic number, it takes 0.5 sec to subscribe
            if (pointcloud_in_subscriber_.getNumPublishers() < 1)
            {
                ROS_ERROR("Cannot start snapshotter, there are no publishers of topic %s", pointcloud_in_topic_.c_str() );
                return false;
            }
            else
            {
                break;
            }
        }

        running_ = true;
        //ROS_INFO("Starting point cloud snapshotter.");
        ROS_INFO("Starting point cloud snapshotter, listening on %s", pointcloud_in_topic_.c_str() );

        return true;
    }
    bool start(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
    {
        return start();
    }

    bool stop()
    {
        // Precondition
        if (!running_)
        {
            ROS_DEBUG("Point cloud snapshotter is already stopped.");
            return true;
        }

        pointcloud_in_subscriber_.shutdown();
        running_ = false;
        point_cloud_in_msg_.reset(); // clear pointer
        ROS_INFO("Stopping point cloud snapshotter.");
        return true;
    }
    bool stop(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
    {
        return stop();
    }

    bool publishSnapshot(std_srvs::Empty::Request  &req,
                         std_srvs::Empty::Response &res)
    {
        // Preconditions
        if (!running_)
        {
            ROS_ERROR("Point cloud snapshotter is not running.");
            return false;
        }
        if (pointcloud_in_subscriber_.getNumPublishers() < 1)
        {
            ROS_ERROR("Point cloud snapshotter found no Publishers of topic %s", pointcloud_in_topic_.c_str() );
            //running_ = false; // Do this if you want a failed snapshot request to disable the snapshotter,
            // otherwise when the input PointCloud returns you can immediately get snapshots again.
            return false;
        }

        if (!point_cloud_in_msg_) //  == NULL
        {
            ROS_ERROR("Point cloud snapshotter has not yet received the first point cloud.");
            return false;
        }

        //ROS_INFO("Publishing 1 PointCloud snapshot.");
        ROS_INFO("Publishing 1 PointCloud snapshot to %s", pointcloud_out_topic_.c_str() );

        // Publish point cloud snapshot
        pointcloud_out_publisher_.publish(point_cloud_in_msg_);
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_snapshotter");

    // Precondition: Valid clock
    ros::NodeHandle nh;
    if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
    {
        ROS_FATAL("Timed-out waiting for valid time.");
        return EXIT_FAILURE;
    }

    PointCloudSnapshotter snapshotter;

    ros::Rate rate(5.0);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}
