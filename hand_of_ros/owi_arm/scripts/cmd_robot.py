#!/usr/bin/env python 
# THIS SHEBANG IS REALLY REALLY IMPORTANT

import rospy
from std_msgs.msg import Int16MultiArray
import time
import argparse
if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(description='run the robot')
        parser.add_argument('integers', metavar='N', type=int, nargs=4,
                            help='an integer for the accumulator')
        args = parser.parse_args()
        pub = rospy.Publisher('robot', Int16MultiArray, queue_size=1)
        rospy.init_node('publisher', anonymous=True)
        msg = Int16MultiArray()
        data = [int(args.integers[0]),int(args.integers[1]),int(args.integers[2]),int(args.integers[3])]
        print "sending {0}.".format(data)
        msg.data = data
        pub.publish(msg)
        time.sleep(0.1) # need this
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
