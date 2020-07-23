#!/usr/bin/env python3

import cv2
import numpy as np
from remotecapture.srv import capture_img, capture_imgResponse
import rospy
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
camera = cv2.VideoCapture(0)

def handle_capture_img(req):
    """The callback function to capture images.

    :req: TODO
    :returns: Time, Image

    """
    tnow = rospy.Time.now()
    rospy.loginfo(f'handle_capture_img [called]; time = {tnow} delay={req.delay}')
    rospy.sleep(req.delay)
    rospy.loginfo("Created camera!")
    try:
        rospy.loginfo("taking image wih camera.read()")
        worked, cvimage = camera.read()
        if worked:
            # rospy.loginfo(f'type(cvimage)={type(cvimage)}\ncmimage={cvimage}')
            rospy.loginfo("converting image to ros_image")
            ros_image = bridge.cv2_to_imgmsg(np.array(cvimage))
            # rospy.loginfo("DONE!")
            tafter = rospy.Time.now()
            return capture_imgResponse(tafter,ros_image)
        else:
            rospy.logwarn("No image captured")
    except Exception as e:
        rospy.logerr("Woops that did not work =(")
        raise e






    pass

def main():
    """The setup function in the take_image.py
    :returns: TODO

    """
    rospy.init_node('remotecapture_node')
    serv = rospy.Service("capture_img",capture_img,handle_capture_img)
    rospy.loginfo("Service capture_img is ready for use!")
    rospy.spin()


if __name__ == "__main__":
    main()
