
import numpy
import threading
import time

import rospy
from sensor_msgs.msg import JointState


class JointStateListener:

    def __init__(self, wait_until_initialized):

        self.topic_name = "/joint_states"

        self.joint_names = []

        self.current_state = JointState()
        self.received_first_msg = False

        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.mainThread)
        self.thread.setDaemon(True)
        self.thread.start()

        if wait_until_initialized:
            rospy.loginfo(
                "Waiting to receive first msg on '" +
                self.topic_name +
                "' ...")
            while(1):
                self.lock.acquire(True)
                if self.received_first_msg:
                    self.lock.release()

                    break
                self.lock.release()

    def mainThread(self):
        rospy.Subscriber(self.topic_name, JointState, self.jointStatesCallback)
        print("in mainThread")

        rospy.spin()

        print("out of mainThread")

    def jointStatesCallback(self, msg):

        self.lock.acquire(True)

        if not self.received_first_msg:
            self.received_first_msg = True
            self.joint_names = msg.name

        self.current_state = msg

        self.lock.release()

    def getJointPosition(self, requested_names):

        self.lock.acquire(True)

        if not self.received_first_msg:
            print("ERROR: Can't get state, no joint states received yet")
            self.lock.release()
            return None

        positions = self.current_state.position

        self.lock.release()

        requested_positions = numpy.ones(len(requested_names)) * numpy.NaN

        for i in range(len(self.joint_names)):
            for j in range(len(requested_names)):
                if self.joint_names[i] == requested_names[j]:
                    requested_positions[j] = positions[i]

        return requested_positions

    def getJointPositionVelocity(self, requested_names):

        self.lock.acquire(True)

        if not self.received_first_msg:
            print("ERROR: Can't get state, no joint states received yet")
            self.lock.release()
            return (None, None)

        positions = self.current_state.position
        velocities = self.current_state.velocity

        self.lock.release()

        if len(velocities) == 0:
            velocities = [0.0] * len(positions)

        N = len(requested_names)
        requested_positions = numpy.ones(N) * numpy.NaN
        requested_velocities = numpy.ones(N) * numpy.NaN

        for i in range(len(self.joint_names)):
            for j in range(len(requested_names)):
                if self.joint_names[i] == requested_names[j]:
                    requested_positions[j] = positions[i]
                    requested_velocities[j] = velocities[i]

        return (requested_positions, requested_velocities)

    def waitUntilReachedJointConfig(
            self,
            joint_names,
            joint_positions,
            angle_tolerance=None):
        if angle_tolerance is None:

            angle_tolerance = 0.00174533

        desired_positions = numpy.array(joint_positions)
        desired_velocities = numpy.zeros(len(joint_names))
        print("using angle_tolerance = ", angle_tolerance)
        print("desired_State: ", desired_positions, desired_velocities)

        while not rospy.is_shutdown():
            (positions, velocities) = self.getJointPositionVelocity(joint_names)
            print("State: ", positions, velocities)
            if numpy.allclose(velocities, desired_velocities, atol=0.0001):

                if numpy.allclose(
                        positions,
                        desired_positions,
                        atol=angle_tolerance):
                    print("Reached config")

                    break
            time.sleep(0.1)
