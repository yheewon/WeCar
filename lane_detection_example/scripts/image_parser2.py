#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from std_msgs.msg import Float64
from utils import BEVTransform,STOPLineEstimator
from utils import BEVTransform, CURVEFit, draw_lane_img,purePursuit
class IMGParser:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed",CompressedImage, self.callback)
        self.img_wlane = None
        
    def callback(self,msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        # lower_wlane = np.array([0,0,190])
        # upper_wlane = np.array([50,50,255])
        # img_wlane = cv2.inRange(img_hsv,lower_wlane,upper_wlane)

        # img_wlane[int(0.7*img_hsv.shape[0]): ,:]=0

        # self.img_wlane = img_wlane


        # img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
        # img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)
        # img_concat = np.concatenate([img_bgr, img_hsv, img_wlane], axis=1)

        # cv2.imshow("Image window", img_concat)
        # cv2.waitKey(1)
         
        h = img_bgr.shape[0]
        w = img_bgr.shape[1]

        lower_sig_y = np.array([20, 240,  240])
        upper_sig_y = np.array([40, 255, 255])

        lower_sig_r = np.array([0, 240,  220])
        upper_sig_r = np.array([10, 255, 255])

        lower_sig_g = np.array([50, 240,  240])
        upper_sig_g = np.array([70, 255, 255])

        img_r = cv2.resize(cv2.inRange(img_hsv,lower_sig_r,upper_sig_r),(w/2,h/2))
        img_y = cv2.resize(cv2.inRange(img_hsv,lower_sig_y,upper_sig_y),(w/2,h/2))
        img_g = cv2.resize(cv2.inRange(img_hsv,lower_sig_g,upper_sig_g),(w/2,h/2))
        img_r[int(h/3/2):,:] = 0
        img_y[int(h/3/2):,:] = 0
        img_g[int(h/3/2):,:] = 0

        img_concat = np.concatenate([img_r,img_y,img_g],axis = 1)

        cv2.imshow("Image window",img_concat)
        cv2.waitKey(1)
        
        # cv2.namedWindow('mouseRGB')
        # cv2.imshow('mouseRGB',self.img_hsv)
        # cv2.setMouseCallback('mouseRGB',self.mouseRGB)
        # cv2.waitKey(1)

    def mouseRGB(self,event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            colorsB = self.img_hsv[y,x,0]
            colorsG = self.img_hsv[y,x,1]
            colorsR = self.img_hsv[y,x,2]
            colors = self.img_hsv[y,x]
            print("Red : ",colorsR)
            print("Green : ",colorsG)
            print("Blue : ",colorsB)
            print("BGR Format : ",colors)
            print("Coordinates of pixel: X: ",x,"Y: ",y)

if __name__ == '__main__':

    rp = rospkg.RosPack()

    currentPath = rp.get_path("lane_detection_example")
    with open(os.path.join(currentPath,'sensor/sensor_params.json'),'r') as fp :
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]
    rospy.init_node('Lane_detector',anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam= params_cam)
    # sline_detector = STOPLineEstimator()
    curve_learner = CURVEFit(order=1)
    # ctrller = purePursuit(lfd = 0.8)

    rate = rospy.Rate(30)

    #BEVTransform
    # while not rospy.is_shutdown():
    #     if image_parser.img_wlane is not None:
    #         img_warp = bev_op.warp_bev_img(image_parser.img_wlane)

    #         cv2.imshow("Image window",img_warp)
    #         cv2.waitKey(1)

    #         rate.sleep()

    #PurePursuit
    while not rospy.is_shutdown():
        if image_parser.img_wlane is not None:
            img_warp = bev_op.warp_bev_img(image_parser.img_wlane)
            lane_pts = bev_op.recon_lane_pts(image_parser.img_wlane)

            x_pred,y_pred_l,y_pred_r = curve_learner.fit_curve(lane_pts)

            curve_learner.write_path_msg(x_pred,y_pred_l,y_pred_r)
            curve_learner.pub_path_msg()

            # ctrller.steering_angle(x_pred,y_pred_l,y_pred_r)
            # ctrller.pub_cmd()

            xyl , xyr = bev_op.project_lane2img(x_pred,y_pred_l,y_pred_r)

            img_warp1 = draw_lane_img(img_warp, xyl[:,0].astype(np.int32),
                                                xyl[:,1].astype(np.int32),
                                                xyr[:,0].astype(np.int32),
                                                xyr[:,1].astype(np.int32))

            cv2.imshow("Image_window",img_warp1)
            cv2.waitKey(1)

            rate.sleep()

    #STOPLineEstimator
    # while not rospy.is_shutdown():
    #     if image_parser.img_wlane is not None:
    #         lane_pts = bev_op.recon_lane_pts(image_parser.img_wlane)

    #         sline_detector.get_x_pointer(lane_pts)
    #         sline_detector.estimate_dist(0.3)
    #         sline_detector.visualize_dist()
    #         sline_detector.pub_sline()

    #         rate.sleep()



