#!/usr/bin/env python 
# THIS SHEBANG IS REALLY REALLY IMPORTANT
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
import rospy
import time
import numpy as np
import tf

class JointStateRepublisher(object):
    def __init__(self):
        # trims are optional, just small adjustments to the 
        # motor angles.
        self.m0_trim = rospy.get_param('m0_trim', -90.00)
        self.m1_trim = rospy.get_param('m1_trim', -90.0)
        self.m2_trim = rospy.get_param('m2_trim', 0.00)
        self.m3_trim = rospy.get_param('m3_trim', 0.00)
        self.wrist_trim = rospy.get_param('wrist_trim', 0.00)
        # We need a transform listener to get the head. 
        self.listener = tf.TransformListener()
        # The input topic 
        self.state_topic = rospy.get_param('~arduino_topic', '/robot')
        # subscribe to the state update of the robot
        rospy.Subscriber(self.state_topic, Int16MultiArray, self._update_from_arduino)

        #set the joint state publish topic
        self.joint_topic = rospy.get_param('~joint_topic', 'joint_states')
        self.joint_state_pub = rospy.Publisher(self.joint_topic, JointState, queue_size = 1 )
        # m0 m1 m2 wrist m3
        self.states = [0,0,0,0,0]
        # set our update rate to 30hz
        self.rate = rospy.get_param('~rate', 30)
        self._run()

    def _run(self):
        # update the robot's state at 30Hz
        r = rospy.Rate(self.rate) # 30hz 
        while not rospy.is_shutdown():
            self._publish_joint_state()
            r.sleep()

    def _update_from_arduino(self,msg):
        pitch = self.states[3]
        # get the pitch as a euler angle between the wrist
        # and the ground plane. Set the wrist to what it is.
        self.listener.waitForTransform('/base_link','/wrist',rospy.Time(),rospy.Duration(.1))
        (trans,rot) = self.listener.lookupTransform('/base_link', '/wrist', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        pitch = euler[1]
        # Also add the update to the pitch.
        self.states[3] += pitch 
        self.states[0] = np.deg2rad(msg.data[4]) + np.deg2rad(self.m0_trim)
        self.states[1] = -(np.deg2rad(msg.data[3]) + np.deg2rad(self.m1_trim))
        self.states[2] = np.deg2rad(msg.data[2]) + np.deg2rad(self.m2_trim)     
        self.states[4] = np.deg2rad(msg.data[1]) + np.deg2rad(self.m3_trim)
 
    def _update(self):
        try:
            # we repeat the process every time we 
            # get an update from the robot.
            self.listener.waitForTransform('/base_link','/wrist',rospy.Time(),rospy.Duration(.1))
            (trans,rot) = self.listener.lookupTransform('/base_link', '/wrist', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            pitch = euler[1]
            self.states[3] += pitch
        except:
            pass

    def _publish_joint_state(self):
        # This gets gets called at 30Hz
        # do the update
        self._update()
        # construct the joint state message.
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name =  ['base_to_body', 'body_to_arm1', 'arm1_to_arm2', 'arm2_to_wrist','wrist_to_endeffector']
        msg.position = self.states
        # we are not using these
        msg.velocity = [0.00,0.00,0.00,0.00,0.00]
        msg.effort = [0.00,0.00,0.00,0.00,0.00]
        # republish the message. 
        self.joint_state_pub.publish(msg)

if __name__ == '__main__':
    try:
        # general ROS boiler plate. 
        rospy.init_node('joint_state_republisher')
        node = JointStateRepublisher()
    except rospy.ROSInterruptException:
        rospy.logwarn('ERROR!!!')
