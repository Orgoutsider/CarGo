#!/usr/bin/env python

import os
import sys
os.environ['KMP_DUPLICATE_LIB_OK']='True'
import cv2
import numpy as np
import rospy
import random
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# from detection_msgs.msg import BoundingBox, BoundingBoxes

from vision_msgs.msg import BoundingBox2DArray, \
    BoundingBox2D
from yolov5_ros.srv import cargoSrv, cargoSrvResponse
from dynamic_reconfigure.server import Server
from yolov5_ros.cfg import drConfig

# add yolov5_deepsort_tensorrt submodule to path
path = os.path.abspath(".")
# 核心
sys.path.insert(0,path + "/src/yolov5_ros/src/")
from detector_trt import Detector
# import tracker_trt
import ctypes

def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    """
    description: Plots one bounding box on image img,
                 this function comes from YoLov5 project.
    param: 
        x:      a box likes [x1,y1,x2,y2]
        img:    a opencv image object
        color:  color to draw rectangle, such as (0,255,0)
        label:  str
        line_thickness: int
    return:
        no return

    """
    tl = (
        line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )

R_Low1_add = np.array([0, 0, -13], dtype=np.uint8)
R_up1_add = np.array([0, 0, 0], dtype=np.uint8)

R_Low2_add = np.array([0, 0, -20], dtype=np.uint8)
R_up2_add = np.array([0, 0, 0], dtype=np.uint8)

G_Low_add = np.array([-2, 0, -17], dtype=np.uint8)
G_up_add = np.array([8, 0, 0], dtype=np.uint8)

B_Low_add = np.array([-10, 0, -20], dtype=np.uint8)
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

class Yolov5Detector:
    def __init__(self):
        self.line_thickness = rospy.get_param("~line_thickness", 3)
        PLUGIN_LIBRARY = rospy.get_param("~plugin_library", "/home/nano/car/car/car_ws/src/yolov5_ros/src/yolov5_deepsort_tensorrt/weights/libmyplugins.so")
        ctypes.CDLL(PLUGIN_LIBRARY)
        engine_file_path = rospy.get_param("~engine_file_path", "/home/nano/car/car/car_ws/src/yolov5_ros/src/yolov5_deepsort_tensorrt/weights/best.engine")
        self.detector = Detector(engine_file_path)

        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image", True)
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic", "/yolov5/image_out"), Image, queue_size=1
            )
        
        # Initialize CV_Bridge
        self.bridge = CvBridge()

        self.colors = range(1, 4)
        self.low_raw = [R_Low1 + R_Low1_add, R_Low2 + R_Low2_add, G_Low + G_Low_add, B_Low + B_Low_add]
        self.up_raw = [R_up1 + R_up1_add, R_up2 + R_up2_add, G_up + G_up_add, B_up + B_up_add]
        self.low = [R_Low1 + R_Low1_add, R_Low2 + R_Low2_add, G_Low + G_Low_add, B_Low + B_Low_add]
        self.up = [R_up1 + R_up1_add, R_up2 + R_up2_add, G_up + G_up_add, B_up + B_up_add]
        self.gain = 0.0 # 扩张比例
        self.srv = rospy.Service('cargoSrv', cargoSrv, self.callback)
        self.param_modification = rospy.get_param('~param_modification', False)
        if self.param_modification:
            self.dr_srv = Server(drConfig, self.dr_callback)
        else:
            rospy.loginfo("Defualt setting: don't modify paramater")

    def callback(self, req):
        # print(data.header)
        rospy.logdebug("Get an image")
        data = req.image
        im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        result_boxes, result_scores, result_classid = self.detector.detect(im)
        objArray = BoundingBox2DArray()
        objArray.boxes = [BoundingBox2D()] * 4
        objArray.header = data.header
        flag = [False] * 4 #没有检测到
        color_id = [0, 2, 1, 0]
        im_plot = im.copy()
        if len(result_boxes) > 0:
            for i in range(len(result_boxes)):
                result_box = result_boxes[i]
                # start_row and start_col are the cordinates 
                # from where we will start cropping
                # end_row and end_col is the end coordinates 
                # where we stop
                size_row,size_col=result_box[3]-result_box[1], result_box[2]-result_box[0]
                start_row,start_col=int(max(0, result_box[1] - size_row * self.gain)),int(max(0, result_box[0] - size_col * self.gain))
                end_row,end_col=int(min(im.shape[0] ,result_box[3] + size_row * self.gain)),int(min(im.shape[1], result_box[2] + size_col * self.gain))
                
                roi = im[start_row:end_row,start_col:end_col]
                if self.param_modification:
                    cv2.imshow("roi", roi)
                    cv2.waitKey(100)
                roi = cv2.GaussianBlur(roi, (3, 3), 0)
                roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
                s_max = 0
                if self.param_modification:
                    # cv2.imshow("im0", im0)
                    cv2.imshow("hsv", roi)
                    cv2.waitKey(100)

                for color in self.colors:
                    if flag[color]:
                        continue

                    if color == 1:#红色
                        # print(self._low[0])
                        # print(self._up[0])
                        dst1 = cv2.inRange(roi, self.low[0], self.up[0])
                        dst2 = cv2.inRange(roi, self.low[1], self.up[1])
                        dst = dst1 + dst2
                    else:
                        dst = cv2.inRange(roi, self.low[color], self.up[color])

                    if self.param_modification:
                        cv2.imshow("dst", dst)
                        cv2.waitKey(100)
                    dst = cv2.erode(dst, element)

                    contours, hierarchy = cv2.findContours(dst, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #查找轮廓                
                    for contour in contours:
                        rect = cv2.minAreaRect(contour)
                        size = rect[1]
                        if size[0] * size[1] > s_max:
                            s_max = size[0] * size[1]
                            color_max = color

                if s_max:
                    flag[color_max] = True
                    # rospy.loginfo("Succeed to detect!")
                    obj = BoundingBox2D()
                    obj.center.x = (result_box[0] + result_box[2]) / 2 #中心点
                    obj.center.y = (result_box[1] + result_box[3]) / 2
                    obj.center.theta = np.pi / 2 #由x轴逆时针转至W(宽)的角度。
                    obj.size_x = (result_box[2] - result_box[0])#长
                    obj.size_y = (result_box[3] - result_box[1])#宽
                    objArray.boxes[color_max] = obj

                    # Annotate the image
                    if self.publish_image:  # Add bbox to image
                        # integer class
                        box_color = [0] * 3
                        box_color[color_id[color_max]] = 128
                        plot_one_box(
                            result_box,
                            im_plot,
                            label="{}:{:.2f}".format(
                                categories[int(result_classid[i])], result_scores[i]
                            ),
                            color=tuple(box_color),
                            line_thickness=self.line_thickness
                        )
        
        else:
            rospy.logwarn("Cannot detect cargo!")

        # Publish prediction
        # self.pred_pub.publish(bounding_boxes)

        # Publish & visualize images
        # if self.view_image:
        #     cv2.imshow(str(0), im0)
        #     cv2.waitKey(1)  # 1 millisecond
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_plot, "bgr8"))

        return cargoSrvResponse(objArray)
    

    def dr_callback(self, config, level):
        if not self.param_modification:
            return config
        
        low_raw = self.low_raw[config.color]
        up_raw = self.up_raw[config.color]
        low = self.low[config.color]
        up = self.up[config.color]
        flag = False
        if self.colors != single_color[config.color]:
            self.colors = single_color[config.color]
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
            self.up[config.color] = up
            rospy.loginfo('Succeed to modify paramater!')
        
        if flag:
            self.low[config.color] = low           
            rospy.loginfo('Succeed to modify paramater!')

        return config
    
    def destroy(self):
        self.detector.destroy()


if __name__ == "__main__":

    # load coco labels
    categories = ["cargo"]
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    
    rospy.spin()
    # cv2.destroyAllWindows()
    detector.destroy()
