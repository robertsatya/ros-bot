#!/usr/bin/env python 
# THIS SHEBANG IS REALLY REALLY IMPORTANT
import rospy
import time
from std_msgs.msg import Int16MultiArray

if __name__ == '__main__':
    try:
        rospy.init_node('simple_publisher')
        # Tell ros we are publishing to the robot topic
        pub = rospy.Publisher('/robot', Int16MultiArray, queue_size=0)
        # Setup our message
        out = Int16MultiArray()
        val = 20
        # generate the message data
        for j in range(0,4):
            # set the joint angles
            out.data = [0,50,50,50,int(val)]
            # send the message
            pub.publish(out)
            # do some book keeping
            val += 10
            rospy.logwarn("Sent a message: {0}".format(val))
            time.sleep(1)

    except rospy.ROSInterruptException:
        rospy.logwarn('ERROR!!!')
