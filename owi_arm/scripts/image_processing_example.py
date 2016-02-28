#!/usr/bin/env python
import rospy
# This is the tool that marshals images into OpenCV
from cv_bridge import CvBridge, CvBridgeError 
# Import some stock ROS message types.
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
# import some utils.
import numpy as np
import cv
import SimpleCV as scv
import copy as copy

class ImageProcessingExample:
    def __init__(self):
        # Allows conversion between numpy arrays and ROS sensor_msgs/Image
        self.bridge = CvBridge() 

        # Allow our topics to be dynamic.
        self.input_camera_topic = rospy.get_param('~input_camera_topic', '/camera/image_rect_color')
        self.output_camera_topic = rospy.get_param('~output_camera_topic', '/camera/color_stuff')

        # WE are going to publish a debug image as it comes in.
        self.pub = rospy.Publisher(self.output_camera_topic, Image,queue_size=10)
        # We also publish the position of the thing we found.
        self.point_pub = rospy.Publisher("/position",Point,queue_size=10)
        # we get our input from a topic, it is of type image, call _show_colored_stuff.
        rospy.Subscriber(self.input_camera_topic, Image, self._show_colored_stuff)
        # run the node
        self._run()

    # Keep the node alive
    def _run(self):
        rospy.spin()
            
    def _show_colored_stuff(self,input):
        # convert our image to CV2 numpy format from ROS format
        latest_image = self.bridge.imgmsg_to_cv2(input)
        if( latest_image is not None ):
            try:
                # convert the image to SimpleCV
                img = scv.Image(latest_image, cv2image=True, colorSpace=scv.ColorSpace.RGB)
                # create a writeable copy
                src = img.copy()
                # Do our image processing get hue, threshold, morphology, connected components.
                mask = src.hueDistance(180).invert().threshold(240).erode(2).dilate(3)
                blobs = src.findBlobsFromMask(mask=mask)
                # if we find something 
                if( blobs ):
                    # draw its convex hull with alpha
                    blobs[-1].drawHull(width=-1,color=scv.Color.YELLOW,alpha=128)
                    # and publish a position update
                    pt = Point(x=blobs[-1].x,y=blobs[-1].y,z=blobs[-1].area())
                    self.point_pub.publish(pt)
                    src = src.applyLayers()
                # convert SimpleCV to CV2 Numpy Format
                cv2img = src.getNumpyCv2()
                # Convert Cv2 numpy to ROS format
                img_msg = self.bridge.cv2_to_imgmsg(cv2img, "bgr8")
                # publish the topic.
                self.pub.publish(self.bridge.cv2_to_imgmsg(cv2img, "bgr8"))
            except CvBridgeError, e:
                print e

# Boilerplate node spin up. 
if __name__ == '__main__':
    try:
        rospy.init_node('ImageProcessingExample')
        p = ImageProcessingExample()
    except rospy.ROSInterruptException:
        pass
