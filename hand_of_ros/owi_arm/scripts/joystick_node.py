#!/usr/bin/env python 
# THIS SHEBANG IS REALLY REALLY IMPORTANT
import rospy
import roscpp
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray
class JoystickNode(object):
    def __init__(self):
        # put away our toys cleanly 
        rospy.on_shutdown(self.shutdown)
        self.pub = rospy.Publisher('robot', Int16MultiArray, queue_size=1)
        # subscribe to the joy and state messages
        # format is topic, message type, callback function
        rospy.Subscriber("/joy", Joy, self.do_it)
        rospy.Subscriber("/state", Int16MultiArray, self.update_state)
        rospy.init_node('joystick_node')
        # our internal state message
        self.state = [0,0,0,0,0]
        # tell ros to chill unless we get a message. 
        rospy.spin()

    def update_state(self,msg):
        # update our internal state every time the robot posts an update
        self.state = msg.data

    def do_it(self,msg):
        # Update our state
        if( self.state is None ):
            self.state = [0,0,0,0,0]
        m1 = self.state[1]
        m2 = self.state[2]
        m3 = self.state[3]
        m4 = self.state[4]
        step = 5
        # Update our state from our buttons
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
        # check for updates 
        data = [self.state[0],
                int(np.clip(m1,0,180)),
                int(np.clip(m2,0,180)),
                int(np.clip(m3,0,120)),
                int(np.clip(m4,0,180))]
        change = any([abs(a-b)>0 for a,b in zip(data,self.state)])
        self.state = data
        # if there is a change
        if( change ):
            # Set the new position out on /robot
            out = Int16MultiArray()
            rospy.loginfo("sending {0}.".format(data))
            out.data = data
            self.pub.publish(out)

    def shutdown(self):
        data = [0,0,0,0]
        out = Int16MultiArray()
        print "sending {0}.".format(data)
        out.data = data
        pub.publish(out)

 
if __name__ == '__main__':
    try:
        # boiler plate to spin up a node. 
        rospy.init_node('joystick_node')
        node = JoystickNode()
    except rospy.ROSInterruptException:
        rospy.logwarn('ERROR!!!')
