#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import yaml
from pathlib import Path
# from remotecapture.msg import video
from sensor_msgs.msg import CameraInfo, Image
from remotecapture.srv import *
from cv_bridge import CvBridge, CvBridgeError

class imgRecorder(object):

    """Docstring for imgRecorder. """
    _servimg=None
    _pubvideo=None
    _cv2WindowName="preview"
    _subscribe_to="/usb_cam/image_raw"
    running=True
    _color_modes_avaible = dict()
    _color_mode = cv2.COLOR_YUV2BGR
    _img_counter=0
    _color_counter = 0
    _color_counter_min = 0
    _color_counter_max = 142
    _color_not_accepted=None
    _path = '/home/ros/repos/valkyrie_data/'
    _camera_matrix = None
    _rec:bool=True

    def __init__(self,rec:bool=True,use_correction_matirx:bool=False):
        """ Image recorder node in terminal subscribes to /camera/img_raw
        :rec:bool:
            If rec is true you store the given images in an archive,
            But if false then it is a streamer

        :use_correction_matirx:bool:
            If you already have an camera correction matrix and set this to true
            The node will use the ros_param '/camera_param'

        """
        self._rec = rec
        if use_correction_matirx:
            matrix = rospy.get_param("/camera_param")
            self._camera_matrix =       np.array(matrix['matrix'])
            self._camera_matrix_mtx =   np.array(matrix['mtx'])
            self._camera_matrix_roi =  matrix['roi']
            self._camera_matrix_dist =  np.array(matrix['dist'])
        if rec:
            name_of_arcive:str = input('Write the name of the archive: ')
            while Path(self._path + "/" + name_of_arcive).exists():
                print("That archive already exists please enter a new one.")
                name_of_arcive:str = input('Write the name of the archive: ')
            self._path += name_of_arcive.lower()
            Path(self._path).mkdir(parents=True, exist_ok=True)
        self._bridge = CvBridge()
        self._node_name = "userinterface_node"
        self._anonymous_node = True
        self._node = rospy.init_node(self._node_name,anonymous=self._anonymous_node)
        rospy.loginfo(f'starting Subscriber to {self._subscribe_to}')
        rospy.Subscriber(self._subscribe_to,Image,self._callback)
        cv2.startWindowThread()
        cv2.namedWindow(self._cv2WindowName)
        self._color_not_accepted=[8,9]

    def __del__(self):
        """Destructor to close resources used
        :returns: None

        """
        rospy.logwarn("Nodes died")
        cv2.destroyAllWindows()

    def _callback(self, data):
        # rospy.loginfo(f'data={data}')
        # self._pubvideo = cv2.cvtColor(self._bridge.imgmsg_to_cv2(data), 4 )
        self._pubvideo = cv2.cvtColor(self._bridge.imgmsg_to_cv2(data),
                cv2.COLOR_BGR2RGBA )
        # rospy.loginfo(f'Got frame {data.time}')

    def _draw_ccs(self, img, coners,imgpts):
        """Draw Cartesian cordinate system on image

        :img: input image
        :coners: coners2
        :imgpts: cv2.projectPoints()
        :returns: image with ccs

        """
        corner = tuple(corners[0],imgpts)
        drawed_image = img
        drawed_image = cv2.line(drawed_image,coner,tuple(imgpts[0].ravel(), (255,0,0),5))
        drawed_image = cv2.line(drawed_image,coner,tuple(imgpts[0].ravel(), (0,255,0),5))
        drawed_image = cv2.line(drawed_image,coner,tuple(imgpts[0].ravel(), (0,0,255),5))
        return drawed_image

    def show_camera(self):
        """Shows the OpenCV window with the captured images.

        """
        # rospy.loginfo_onec(f'imgtype = {type(self._pubvideo)}')
        if self._pubvideo is not None:
            rospy.loginfo_once("Press [q] to quit, [enter] to take image")
            if self._camera_matrix is not None:
                rospy.loginfo_once("camera_matirx is present")
                x,y,w,h = self._camera_matrix_roi
                rospy.loginfo_once(
                        f"""
                        self._camera_matrix_mtx, \n{self._camera_matrix_mtx}
                        self._camera_matrix_dist, \n{self._camera_matrix_dist}
                        self._camera_matrix, \n{self._camera_matrix}
                        """)
                undistort_rect=cv2.undistort(
                        self._pubvideo,
                        self._camera_matrix_mtx,
                        self._camera_matrix_dist,
                        None,
                        self._camera_matrix)
                corp_rect = undistort_rect[y:y+h,x:x+w]
                cv2.imshow(self._cv2WindowName, corp_rect)
            else:
                cv2.imshow(self._cv2WindowName, self._pubvideo)
            key = cv2.waitKeyEx(1)
            if not key == -1:
                print(f'key = {key} is pressed')
            if key == 1048689 : # [q] key to quit
                rospy.loginfo("Quiting the program")
                self.running = False
            if key == 1048589 and self._rec: # [Enter] key to take image
                # status = cv2.imwrite(f'{self._path}/img_{self._img_counter}.png',self._pubvideo)
                status = cv2.imwrite(f'{self._path}/img_{self._img_counter:03}.png',self._pubvideo) # Zero padded 00f
                if status:
                    rospy.loginfo(f'Save picture {self._path}/img_{self._img_counter:03}.png')
                    self._img_counter += 1
                else:
                    rospy.loginfo("Failed to save image")

    def show_search_camera_prop(self):
        if self._pubvideo is not None:
            while self._color_counter in self._color_not_accepted:
                rospy.loginfo_once("Search")
                self._color_counter += 1
            try:
                cv2.imshow(self._cv2WindowName, cv2.cvtColor(self._pubvideo, self._color_counter ))
                # cv2.imshow(self._cv2WindowName, cv2.cvtColor(self._pubvideo, 2 ))
                rospy.loginfo(f'_color_counter ={self._color_counter}')

            except Exception as e:
                rospy.loginfo(f'_color_counter ={self._color_counter} not Accepted')
            self._color_counter += 1
            if self._color_counter > self._color_counter_max:
                self._color_counter = self._color_counter_min
                # cv2.imshow(self._cv2WindowName, self._pubvideo)



def main():
    im = imgRecorder(rec=True, use_correction_matirx=False)

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        im.show_camera()
        # im.show_search_camera_prop()
        if im.running == False:
            break
        # r.sleep()

    pass

if __name__ == "__main__":
    main()
