### ROS Node: reem_head_scan_action
Copyright (c) 2013, David Butterworth, PAL Robotics S.L. 
<br>
<br>
An Action Server for the REEM robot, to move the head & take PointCloud snapshots.

Used as part of the perception system, to provide an easy method to request the robot to scan an area and aggregate the snapshots of PointCloud data <br>
e.g. scanning a table with head-mounted RGBD sensor & building a Collision Map.

The same behaviour could be accomplished be repeatedly calling the Point Head Action Server with some x-y co-ordinates, so this node offers a specific "scan the table" behaviour.
<br>

<br>
This node should be launched in the same namespace as the 'follow_joint_trajectory' Action for the head, and provides the 'scan_table_action' Action.

It is called using an empty Action message of type pr2_controllers_msgs::PointHeadAction

When requested, the robot will perform the following hard-coded behaviour: <br>
&nbsp;&nbsp;&nbsp;1. look straight ahead, snapshot <br>
&nbsp;&nbsp;&nbsp;2. look left, snapshot <br>
&nbsp;&nbsp;&nbsp;3. look right, snapshot <br>
&nbsp;&nbsp;&nbsp;4. look down-left, snapshot <br>
&nbsp;&nbsp;&nbsp;5. look down-right, snapshot <br>
&nbsp;&nbsp;&nbsp;6. look down-center, snapshot <br>

After requesting each PointCloud snapshot, this node waits 2.5 sec because it seems there is some delay in matching the PointCloud data with its correct TF transform. The minimum delay is approx 2 sec.
<br>

<br>
ToDo: - Maybe the robot should finish looking straight ahead, then the user's code should move the <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; head back down at the table, or add multiple pre-programmed scan patterns. <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Make some of the variables as ROS parameters e.g. the 2 sec delay after each snapshot. <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Test the goal cancel callback, maybe the head should return to look straight ahead. <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Test the watchdog callback, in case the head move fails.
<br>

<br>
**Required ROS packages:** <br>
reem_common  (DavidB-PAL fork, not yet merged with Master 15/3/13) <br>
pointcloud_snapshotter <br>
head_traj_controller (an instance of the Joint Trajectory Action Server) <br>
reem_manipulation_worlds (only for testing)
<br>

<br>
**Usage:** <br>
Launch simulated REEM robot with Asus RGBD sensor: <br>
$ export USE_RGBD_SENSOR=true <br>
$ roslaunch reem_manipulation_worlds reem_high_white_table_manipulation.launch <br>
$ roslaunch pointcloud_snapshotter pointcloud_snapshotter.launch <br>
$ roslaunch reem_head_scan_action scan_snapshot_action_node.launch <br>
$ roslaunch pointcloud_snapshotter rviz.launch <br>
Watch the Rviz display, then request the Action: <br>
$ rosrun reem_head_scan_action scan_table


