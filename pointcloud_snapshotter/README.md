### ROS Node: pointcloud_snapshotter
Copyright (c) 2013, Adolfo Rodriguez Tsouroukdissian, David Butterworth, PAL Robotics S.L. 
<br>
<br>
This node takes 1 'snapshot' from a PointCloud input and publishes it on some output topic, when triggered by a ROS Service call.

Typically an RGBD sensor will publish at upto 30Hz, unless frame dropping is enabled. This can induce some computational overhead while subscribing to all these PointCloud messages, so two methods can reduce this load: <br>
1. This snapshotter can be launched in a 'stopped' state, and will only subscribe to the PointCloud input when it is 'started'. This is useful if the RGBD sensor is only used in particular situations. <br>
2. PointCloud messages are only published when the trigger is called, which reduces the load required to continuously filter the PointCloud and update the Collision Map. <br>
Additionally, it prevents the Collision Map being corrupted due to a mismatch between PointCloud and TF data, that may be induced if it's updated continuously while the RGBD sensor is moved about.
<br>

<br>
The node can be launched in the 'started' state. It will check for the input PointCloud and try to subscribe idefinitely until it becomes available.

If the node is launched in the 'stopped' state, a Service call to 'start' will only attempt to subscribe to the input PointCloud for maximum 3 sec.

Making a 'snapshot' request will always check if the input PointCloud is available, otherwise it would simply re-publish the last frame of data before the input was lost. <br>
If the input is lost, calls to 'snapshot' will immediately start working again once the input cloud is available. There is an option in the code if you would prefer to 'start' the node again first.

The Service calls are always successful, because there is nothing in here that should fail. <br>
However, the 'snapshot' Service call will fail if the snapshotter is not running, or the input PointCloud is not available.
<br>

<br>
**Required ROS packages:** <br>
(only for testing with REEM robot) <br>
reem_common  (DavidB-PAL fork, not yet merged with Master 15/3/13) <br>
<br>

<br>
**Usage:** <br>
Launch simulated REEM robot with Asus RGBD sensor: <br>
$ export USE_RGBD_SENSOR=true <br>
$ roslaunch reem_gazebo reem_empty_world.launch <br>
$ roslaunch pointcloud_snapshotter pointcloud_snapshotter.launch <br>
$ roslaunch pointcloud_snapshotter rviz.launch <br>
Watch the Rviz display, then trigger 1 PointCloud snapshot: <br>
$ rosservice call /xtion_snapshotter/snapshot <br>

To manually start or stop the snapshotter, disable the auto-start parameter in the launch file, then run: <br>
$ rosservice call /xtion_snapshotter/start<br>
$ rosservice call /xtion_snapshotter/stop <br>

If you put this snapshotter in-line with your perception system, you may need to use the accompanying 'smart trigger' to initialize the ROS Collision Map: <br>
$ roslaunch pointcloud_snapshotter trigger_collision_map.launch 


