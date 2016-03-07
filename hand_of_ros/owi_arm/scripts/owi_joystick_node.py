#!/usr/bin/env python 
# THIS SHEBANG IS REALLY REALLY IMPORTANT
import rospy
import roscpp
import numpy as np
import actionlib
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray
from owi_arm.srv import * # for services
from owi_arm.msg import * # for actions

class MyJoystickNode(object):
    def __init__(self):
        # create the action library client to play animation
        self.animate_client = actionlib.SimpleActionClient('play_animation', play_animationAction)
        # wait for that action to spin up.
        self.animate_client.wait_for_server()
        # create a proxy to the waypoint service. 
        self.waypoint_proxy = rospy.ServiceProxy('/waypoint', waypoint)
        
        rospy.Subscriber("/joy", Joy, self.do_it)
        rospy.Subscriber("/state", Int16MultiArray, self.update_state)
        rospy.init_node('owi_joystick_node')
        self.state = [0,0,0,0,0]
        self.rb_trigger = 0
        self.rt_trigger = 0
        self.pub = rospy.Publisher('/robot', Int16MultiArray, queue_size=1)
        rospy.spin()

    def update_state(self,msg):
        self.state = msg.data

    def do_it(self,msg):

        if( self.state is None ):
            self.state = [0,0,0,0,0]
        m1 = self.state[1]
        m2 = self.state[2]
        m3 = self.state[3]
        m4 = self.state[4]
        step = 5
        if(msg.buttons[1] == 1 ):
            m1+=step
        elif( msg.buttons[2] == 1 ):
            m1-=step
        if(msg.buttons[0] == 1 ):
            m2+=step
        elif( msg.buttons[3] ==1 ):
            m2-=step
        if(msg.axes[-1] > 0 ):
            m3+=step
        elif( msg.axes[-1] < 0 ):
            m3-=step
        if(msg.axes[-2] > 0 ):
            m4+=step
        elif( msg.axes[-2] < 0 ):
            m4-=step
            
        data = [self.state[0],
                int(np.clip(m1,0,180)),
                int(np.clip(m2,0,180)),
                int(np.clip(m3,0,120)),
                int(np.clip(m4,0,180))]
        change = any([abs(a-b)>0 for a,b in zip(data,self.state)])
        self.state = data
        if( change ):
            out = Int16MultiArray()
            rospy.loginfo("sending {0}.".format(data))
            out.data = data
            self.pub.publish(out)
        # button up
        if( msg.buttons[4] == 0 and self.rb_trigger == 1):
            self.call_waypoint_service("animate.txt")
        self.rb_trigger = msg.buttons[4]
 
        if( msg.buttons[5] == 0 and self.rt_trigger == 1):
           self.call_animate_service("animate.txt",1)
        self.rt_trigger = msg.buttons[5]
        
    def call_animate_service(self,fname,t):
        rospy.logwarn("ANIMATING!!! -- like regular mating but better.")
        # create a goal for action
        goal = play_animationGoal() 
        goal.filename = fname
        goal.step_size = t
        self.animate_client.send_goal(goal)
        # wait for the animate action to complete
        self.animate_client.wait_for_result()
        # get the result of the animate
        result = self.animate_client.get_result()
        rospy.logwarn("result of animation {0}".format(result))

    def call_waypoint_service(self,fname):
        # create the request message
        req = waypointRequest()
        req.fname = fname
        # send the message
        response = self.waypoint_proxy(req)
        # print the result. 
        rospy.logwarn("JOYSTICK NODE GOT {0}".format(response.result))


 

if __name__ == '__main__':
    try:
        rospy.init_node('owi_joystick_node')
        node = MyJoystickNode()
    except rospy.ROSInterruptException:
        rospy.logwarn('ERROR!!!')
