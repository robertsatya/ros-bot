#!/usr/bin/env python

from rob_nav.srv import *
import rospy

def act_arm_cb(req):
#    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return ArmResponse(1)

def arm_main():
    rospy.init_node('nav_arm_service')
    s = rospy.Service('nav_arm', Arm, act_arm_cb)
    rospy.spin()

if __name__ == "__main__":
    arm_main()

