#!/usr/bin/env python
"""
 @Author: Jianfeng Cui 
 @Date: 2021-05-22 11:36:30 
 @Last Modified by:   Jianfeng Cui 
 @Last Modified time: 2021-05-22 11:36:30 
"""

# Check Pytorch installation
from logging import debug
from mmcv import image
import torch, torchvision
print(torch.__version__, torch.cuda.is_available())

# Check MMDetection installation
import mmdet
print(mmdet.__version__)

# Check mmcv installation
from mmcv.ops import get_compiling_cuda_version, get_compiler_version
print(get_compiling_cuda_version())
print(get_compiler_version())

from mmdet.apis import inference_detector, init_detector, show_result_pyplot

import os
import sys
import cv2
import numpy as np

# ROS related imports
import rospy
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from mmdetection_ros.cfg import drConfig

# NOTE: 
# CvBridge meet problems since we are using python3 env
# We can do the data transformation manually
# from cv_bridge import CvBridge, CvBridgeError

from vision_msgs.msg import BoundingBox2DArray, \
    BoundingBox2D
from mmdetection_ros.srv import cargoSrv
from mmdetection_ros.srv import cargoSrvResponse

from mmdet.models import build_detector

import threading

#修改此處
DIR_PATH = '/home/fu/apriltag_ws/src/mmdetection_ros'
# Choose to use a config and initialize the detector
CONFIG_NAME = 'faster_rcnn_r101_fpn_2x_coco.py'
# CONFIG_PATH = os.path.join(os.path.dirname(sys.path[0]),'scripts', CONFIG_NAME)
CONFIG_PATH = os.path.join(DIR_PATH,'scripts', CONFIG_NAME)

# Setup a checkpoint file to load
MODEL_NAME =  'epoch_24.pth'
# MODEL_PATH = os.path.join(os.path.dirname(sys.path[0]),'scripts', MODEL_NAME)
MODEL_PATH = os.path.join(DIR_PATH,'scripts', MODEL_NAME)

R_Low1_add = np.array([0, -7, -10], dtype=np.uint8)
R_up1_add = np.array([0, 0, 0], dtype=np.uint8)

R_Low2_add = np.array([0, 0, 0], dtype=np.uint8)
R_up2_add = np.array([0, 0, 0], dtype=np.uint8)

G_Low_add = np.array([0, 0, 0], dtype=np.uint8)
G_up_add = np.array([11, 0, 0], dtype=np.uint8)

B_Low_add = np.array([-5, 0, 0], dtype=np.uint8)
B_up_add = np.array([0, 0, 0], dtype=np.uint8)

R_Low1 = np.array([0, 43, 46], dtype=np.uint8)
R_up1 = np.array([10, 255, 255], dtype=np.uint8)

R_Low2 = np.array([156, 43, 46], dtype=np.uint8)
R_up2 = np.array([180, 255, 255], dtype=np.uint8)

G_Low = np.array([35, 43, 46], dtype=np.uint8)
G_up = np.array([77, 255, 255], dtype=np.uint8)

B_Low = np.array([100, 43, 46], dtype=np.uint8)
B_up = np.array([124, 255, 255], dtype=np.uint8)

single_color = [[1], [1], [2], [3]]

class Detector:

    def __init__(self, model):
        self.image_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        # self.object_pub = rospy.Publisher("~objects", BoundingBox2D, queue_size=1)
        # self.bridge = CvBridge()
        self.model = model

        self._colors = range(1, 4)
        self._low_raw = [R_Low1 + R_Low1_add, R_Low2 + R_Low2_add, G_Low + G_Low_add, B_Low + B_Low_add]
        self._up_raw = [R_up1 + R_up1_add, R_up2 + R_up2_add, G_up + G_up_add, B_up + B_up_add]
        self._low = [R_Low1 + R_Low1_add, R_Low2 + R_Low2_add, G_Low + G_Low_add, B_Low + B_Low_add]
        self._up = [R_up1 + R_up1_add, R_up2 + R_up2_add, G_up + G_up_add, B_up + B_up_add]
        self._gain = 0.1 # 扩张比例
        self._score_thr = 0.8

        self._last_msg = None
        # self._last_res_avaliable = False
        self._last_res = BoundingBox2DArray()
        self._msg_lock = threading.Lock()
        
        self._publish_rate = rospy.get_param('~publish_rate', 1)
        self._visualization = rospy.get_param('~visualization', True)
        self._param_modification = rospy.get_param('~param_modification', False)

    def generate_obj(self, results, src, msg):
        if self._param_modification:
            cv2.imshow("src", src)

        flag = [False] * 4 #没有检测到
        objArray = BoundingBox2DArray()
        objArray.header = msg.header
        objArray.boxes = [BoundingBox2D()] * 4
        if results.shape == (0, 5):
            rospy.logwarn("Cannot detect cargo!")
            # results = [0]
        else:
            for i in range(len(results)):
                # if empty:
                #     im = src
                # else:
                result = results[i]
                if result[4] < self._score_thr:
                    continue
                size_row,size_col=result[3]-result[1], result[2]-result[0]
                start_row,start_col=int(max(0, result[1] - size_row * self._gain)),int(max(0, result[0] - size_col * self._gain))
                end_row,end_col=int(min(src.shape[0] ,result[3] + size_row * self._gain)),int(min(src.shape[1], result[2] + size_col * self._gain))
    
                # start_row and start_col are the cordinates 
                # from where we will start cropping
                # end_row and end_col is the end coordinates 
                # where we stop
                im = src[start_row:end_row,start_col:end_col]
                # im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
                if self._param_modification:
                    cv2.imshow("im", im)

                im = cv2.GaussianBlur(im, (7, 7), 0)
                im = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
                if self._param_modification:
                    cv2.imshow("hsv", im)
                element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
                s_max = 0
                colors = self._colors
                for color in colors:
                    if flag[color]:
                        continue

                    if color == 1:#红色
                        # print(self._low[0])
                        # print(self._up[0])
                        dst1 = cv2.inRange(im, self._low[0], self._up[0])
                        dst2 = cv2.inRange(im, self._low[1], self._up[1])
                        dst = dst1 + dst2
                    else:
                        dst = cv2.inRange(im, self._low[color], self._up[color])

                    if self._param_modification:
                        cv2.imshow("dst", dst)
                        cv2.waitKey(1)
                    dst = cv2.erode(dst, element)

                    contours, hierarchy = cv2.findContours(dst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #查找轮廓                
                    for contour in contours:
                        rect = cv2.minAreaRect(contour)
                        size = rect[1]
                        if size[0] * size[1] > s_max:
                            s_max = size[0] * size[1]
                            rect_max = rect
                            color_max = color

                    # if empty and s_max:
                    #     obj = BoundingBox2D()
                    #     point = rect_max[0] #中心点
                    #     obj.center.x = point[0]
                    #     obj.center.y = point[1]
                    #     obj.center.theta = rect_max[2]
                    #     size = rect_max[1]
                    #     obj.size_x = size[0] * (self._gain + 1)
                    #     obj.size_y = size[1] * (self._gain + 1)
                    #     objArray.boxes[color] = obj

                if s_max:
                    flag[color_max] = True
                    # rospy.loginfo("Succeed to detect!")
                    obj = BoundingBox2D()
                    obj.center.x = (result[0] + result[2]) / 2 #中心点
                    obj.center.y = (result[1] + result[3]) / 2
                    obj.center.theta = np.pi / 2 #由x轴逆时针转至W(宽)的角度。
                    obj.size_x = (result[2] - result[0]) * (self._gain + 1)#长
                    obj.size_y = (result[3] - result[1]) * (self._gain + 1)#宽
                    # point = rect_max[0]
                    # obj.center.x = point[0] + start_col
                    # obj.center.y = point[1] + start_row
                    # obj.center.theta = rect_max[2]
                    # size = rect_max[1]
                    # obj.size_x = size[0] * (self._gain + 1)
                    # obj.size_y = size[1] * (self._gain + 1)
                    objArray.boxes[color_max] = obj

        return objArray
        

    def run(self):

        # if not self._is_service:
        #     rospy.loginfo('RUNNING MMDETECTOR AS PUBLISHER NODE')
        #     image_sub = rospy.Subscriber("~image", Image, self._image_callback, queue_size=1)
        # else:
        if self._param_modification:
            dr_server = Server(drConfig, self.cb)
        else:
            rospy.loginfo("Defualt setting: don't modify paramater")
            
        srv = rospy.Service('cargoSrv', cargoSrv, self.service_handler)

        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            if self._msg_lock.acquire(False):
                msg = self._last_msg
                # self._last_res_avaliable = False
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue

            if msg is not None:
                # try:
                #     cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                # except CvBridgeError as e:
                #     print(e)
                # NOTE: This is a way using numpy to convert manually
                im = np.frombuffer(msg.data, dtype = np.uint8).reshape(msg.height, msg.width, -1)
                image_np = np.asarray(im)

                # Use the detector to do inference
                # NOTE: inference_detector() is able to receive both str and ndarray
                results = inference_detector(self.model, image_np)

                if isinstance(results, tuple):
                    bbox_result, segm_result = results
                    if isinstance(segm_result, tuple):
                        segm_result = segm_result[0]  # ms rcnn
                else:
                    bbox_result, segm_result = results, None

                # for i in range(len(results)):
                #     if results[i].shape != (0, 5):
                #         object_count += 1
                #         objArray.detections.append(self.generate_obj(results[i], i, msg))

                objArray = self.generate_obj(bbox_result[0], im, msg)
                # rospy.loginfo('RESPONSING SERVICE')
                self._last_res = objArray

                # Visualize results
                if self._visualization:
                    # NOTE: Hack the provided visualization function by mmdetection
                    # Let's plot the result
                    # show_result_pyplot(self.model, image_np, results, score_thr=0.3)
                    # if hasattr(self.model, 'module'):
                    #     m = self.model.module
                    debug_image = self.model.show_result(
                                    image_np,
                                    results,
                                    score_thr=0.8,
                                    show=False,
                                    wait_time=0,
                                    win_name='result',
                                    bbox_color=(72, 101, 241),
                                    text_color=(72, 101, 241))
                    # img = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
                    # image_out = Image()
                    # try:
                        # image_out = self.bridge.cv2_to_imgmsg(img,"bgr8")
                    # except CvBridgeError as e:
                    #     print(e)
                    # image_out.header = msg.header
                    image_out = msg
                    # NOTE: Copy other fields from msg, modify the data field manually
                    # (check the source code of cvbridge)
                    image_out.data = debug_image.tobytes()
                    self.image_pub.publish(image_out) 

            rate.sleep()

    def service_handler(self, request):
        rospy.logdebug("Get an image")
        if self._msg_lock.acquire(False):
            self._last_msg = request.image
            self._msg_lock.release()
            loop_rate = rospy.Rate(self._publish_rate)
            loop_rate.sleep()
            # rospy.loginfo('RESPONSING SERVICE')
            return cargoSrvResponse(self._last_res)
        else:
            # rospy.loginfo('RESPONSING SERVICE')
            return cargoSrvResponse(BoundingBox2DArray()) 

    def cb(self, config, level):
        if not self._param_modification:
            return config
        
        low_raw = self._low_raw[config.color]
        up_raw = self._up_raw[config.color]
        low = self._low[config.color]
        up = self._up[config.color]
        flag = False
        if self._colors != single_color[config.color]:
            self._colors = single_color[config.color]
            rospy.loginfo('Succeed to modify paramater!')

        if low_raw[0] + config.h_low != low[0] and low_raw[0] + config.h_low >= 0:
            flag = True
            low[0] = low_raw[0] + config.h_low

        if low_raw[1] + config.s_low != low[1] and low_raw[1] + config.s_low >= 0:
            flag = True
            low[1] = low_raw[1] + config.s_low

        if low_raw[2] + config.v_low != low[2] and low_raw[2] + config.v_low >= 0:
            flag = True
            low[2] = low_raw[2] + config.v_low

        if up_raw[0] + config.h_up != up[0] and up_raw[0] + config.h_up <= 255:
            up[0] = up_raw[0] + config.h_up
            self._up[config.color] = up
            rospy.loginfo('Succeed to modify paramater!')
        
        if flag:
            self._low[config.color] = low           
            rospy.loginfo('Succeed to modify paramater!')

        return config
                

def main(args):
    rospy.init_node('cargo_detector')
    model = init_detector(CONFIG_PATH, MODEL_PATH, device='cuda:0')
    obj = Detector(model)
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("ShutDown")
    obj.run()
    # cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)