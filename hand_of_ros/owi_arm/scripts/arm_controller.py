#!/usr/bin/env python
import rospy
# need to use the ros actionlib
import actionlib
# need to import our automagically generated messages
from owi_arm.srv import *
from owi_arm.msg import *
from std_msgs.msg import Int16MultiArray
import sys, os, time

class ArmController:
    def __init__(self):
        self.debug = rospy.get_param('~debug', False)
        # get param let's us set parameters 
        self.path = rospy.get_param('/animation_path','/home/kscottz/Desktop/')
 
        rospy.loginfo("Start Arm Controller")        
        self.state = [0,0,0,0,0]
        # create the waypoint service, topic, message type, callback function.
        self.waypoint_service = rospy.Service('/waypoint', waypoint, self._handle_save_state_to_file)   
 
        # subscribe to the robot's state
        rospy.Subscriber("/state", Int16MultiArray, self.update_state)
        self.pub = rospy.Publisher('/robot', Int16MultiArray, queue_size=1)
        # create our action server. 
        # format is name, action message type, callback, autostart.
        self.action_server_play = actionlib.SimpleActionServer('play_animation', play_animationAction, 
                                                               execute_cb=self._handle_play_animation, auto_start = False)
        # start the server
        self.action_server_play.start()
        # run the node
        self._run()

    def update_state(self,msg):
        # update the internal state just like before. 
        self.state = msg.data

    def _run(self):
        # tell ros to just hang out until we get a message.
        rospy.spin()

    def _handle_save_state_to_file(self,req):
        # wait for a state update.
        rospy.wait_for_message('/state', Int16MultiArray, timeout=10)
        # set our file name
        fname = self.path+req.fname
        # create our output stream
        out_str = "{0},{1},{2},{3}\n".format(self.state[1],self.state[2],self.state[3],self.state[4])
        
        # write the file out of the current state
        if( not os.path.isfile(fname) ): # open in write mode
            with open(fname, 'w') as outfile:
                outfile.write(out_str)
        else:
            with open(fname, 'a') as outfile:
                outfile.write(out_str)
        # create and return a response message. 
        msg = "Wrote {0} to file {1}.".format(out_str,fname)
        rospy.logwarn(msg)
        response = waypointResponse()
        response.result = msg
        return response

    def _handle_play_animation(self,goal):
        # assemble the file name from the input goal
        fname = self.path + goal.filename 
        # get the time between each animation step. 
        sleepy_time = goal.step_size 
        # assemble the output string 
        result = play_animationResult()

        # check that our file is good, if not bail and send the result
        if( not os.path.isfile(fname) ): 
            result.result = "Could not find file {0}".format(fname)
            self.action_server_play.set_failed(result)
            return
        # create our feedback object
        self.feedback = play_animationFeedback()
        self.feedback.update = "Loading file {0}.".format(fname)
        # publish the feedback
        self.action_server_play.publish_feedback(self.feedback)
        steps = []
        # open our file and parse it into the steps list. 
        with open(fname, "rt") as f:
            for line in f:
                steps.append([int(j) for j in line.split(',')])
        # send an update 
        self.feedback.update = "Finished reading file, got {0} commands.".format(len(steps))
        self.action_server_play.publish_feedback(self.feedback)
        # go through our steps and send them to the robot. 
        for step in steps:
            out = Int16MultiArray()
            rospy.loginfo("sending {0}.".format(step))
            out.data = [0,step[0],step[1],step[2],step[3]]
            self.pub.publish(out)
            # send out our feedback
            self.feedback.update = "Going to {0}.".format(out.data)
            self.action_server_play.publish_feedback(self.feedback)
            # sleep a bit to let the command execute
            time.sleep(sleepy_time)
            # check if we need to stop
            if self.action_server_play.is_preempt_requested():
                result.result = "We did nothing"
                self.action_server_play.set_succeeded(result)
                return
            
        result.result = "All done"
        self.action_server_play.set_succeeded(result)
        return result
            



        

if __name__ == '__main__':
    try:
        rospy.init_node('ArmController')
        arm_controller = ArmController()
    except rospy.ROSInterruptException:
        pass
