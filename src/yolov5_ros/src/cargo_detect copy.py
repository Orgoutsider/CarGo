#!/usr/bin/env python3

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
# from rostopic import get_topic_type

from sensor_msgs.msg import Image
# from detection_msgs.msg import BoundingBox, BoundingBoxes

from vision_msgs.msg import BoundingBox2DArray, \
    BoundingBox2D
from yolov5_ros.srv import cargoSrv, cargoSrvResponse
from dynamic_reconfigure.server import Server
from yolov5_ros.cfg import drConfig

# add yolov5 submodule to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator
from utils.torch_utils import select_device
from utils.augmentations import letterbox


R_Low1_add = np.array([0, 0, 13], dtype=np.uint8)
R_up1_add = np.array([0, 0, 0], dtype=np.uint8)

R_Low2_add = np.array([0, 0, 0], dtype=np.uint8)
R_up2_add = np.array([0, 0, 0], dtype=np.uint8)

G_Low_add = np.array([15, 0, -17], dtype=np.uint8)
G_up_add = np.array([0, 0, 0], dtype=np.uint8)

B_Low_add = np.array([-10, 0, 0], dtype=np.uint8)
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

@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        # self.view_image = rospy.get_param("~view_image")
        # Initialize weights 
        weights = rospy.get_param("~weights")
        # Initialize model
        self.device = select_device(str(rospy.get_param("~device","")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h",480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model.warmup()  # warmup        
        
        # Initialize subscriber to Image/CompressedImage topic
        # input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking = True)
        # self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        # if self.compressed_input:
        #     self.image_sub = rospy.Subscriber(
        #         input_image_topic, CompressedImage, self.callback, queue_size=1
        #     )
        # else:
        #     self.image_sub = rospy.Subscriber(
        #         input_image_topic, Image, self.callback, queue_size=1
        #     )

        # Initialize prediction publisher
        # self.pred_pub = rospy.Publisher(
        #     rospy.get_param("~output_topic"), BoundingBoxes, queue_size=10
        # )
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=1
            )
        
        # Initialize CV_Bridge
        self.bridge = CvBridge()

        self.colors = range(1, 4)
        self.low_raw = [R_Low1 + R_Low1_add, R_Low2 + R_Low2_add, G_Low + G_Low_add, B_Low + B_Low_add]
        self.up_raw = [R_up1 + R_up1_add, R_up2 + R_up2_add, G_up + G_up_add, B_up + B_up_add]
        self.low = [R_Low1 + R_Low1_add, R_Low2 + R_Low2_add, G_Low + G_Low_add, B_Low + B_Low_add]
        self.up = [R_up1 + R_up1_add, R_up2 + R_up2_add, G_up + G_up_add, B_up + B_up_add]
        self.gain = 0.1 # 扩张比例
        self.score_thr = 0.8
        self.srv = rospy.Service('cargoSrv', cargoSrv, self.callback)
        rospy.loginfo("Service set up!")
        self.debug = rospy.get_param('~debug', False)
        if self.debug:
            self.dr_srv = Server(drConfig, self.dr_callback)
        else:
            rospy.loginfo("Defualt setting: don't modify paramater")

    def callback(self, req):
        """adapted from yolov5/detect.py"""
        # print(data.header)
        rospy.logdebug("Get an image")
        data = req.image
        im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        im, im0 = self.preprocess(im)
        # print(im.shape)
        # print(img0.shape)
        # print(img.shape)

        # Run inference
        im = torch.from_numpy(im).to(self.device) 
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]

        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
        )

        ### To-do move pred to CPU and fill BoundingBox messages
        
        # Process predictions 
        det = pred[0].cpu().numpy()

        objArray = BoundingBox2DArray()
        objArray.boxes = [BoundingBox2D()] * 4
        objArray.header = data.header
        flag = [False] * 4 #没有检测到
        im_plot = im0.copy()
        annotator = Annotator(im_plot, line_width=self.line_thickness, example=str(self.names))
        color_id = [0, 2, 1, 0]
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            for *xyxy, conf, cls in reversed(det):
                # bounding_box = BoundingBox()
                c = int(cls)
                # if conf < self.conf_thres:
                #     rospy.logerr("ok")
                #     continue

                # start_row and start_col are the cordinates 
                # from where we will start cropping
                # end_row and end_col is the end coordinates 
                # where we stop
                size_row,size_col=xyxy[3]-xyxy[1], xyxy[2]-xyxy[0]
                start_row,start_col=int(max(0, xyxy[1] - size_row * self.gain)),int(max(0, xyxy[0] - size_col * self.gain))
                end_row,end_col=int(min(im0.shape[0] ,xyxy[3] + size_row * self.gain)),int(min(im0.shape[1], xyxy[2] + size_col * self.gain))
                roi = im0[start_row:end_row,start_col:end_col]
                if self.debug:
                    cv2.imshow("roi", roi)
                    cv2.waitKey(100)
                roi = cv2.GaussianBlur(roi, (3, 3), 0)
                roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
                s_max = 0
                if self.debug:
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

                    if self.debug:
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
                    obj.center.x = (xyxy[0] + xyxy[2]) / 2 #中心点
                    obj.center.y = (xyxy[1] + xyxy[3]) / 2
                    obj.center.theta = np.pi / 2 #由x轴逆时针转至W(宽)的角度。
                    obj.size_x = (xyxy[2] - xyxy[0])#长
                    obj.size_y = (xyxy[3] - xyxy[1])#宽
                    objArray.boxes[color_max] = obj

                    # Annotate the image
                    if self.publish_image:  # Add bbox to image
                        # integer class
                        label = f"{self.names[c]} {conf:.2f}"
                        box_color = [0] * 3
                        box_color[color_id[color_max]] = 128
                        box_color = tuple(box_color)
                        annotator.box_label(xyxy, label, color=box_color)       

                
                ### POPULATE THE DETECTION MESSAGE HERE

            # Stream results
            im_plot = annotator.result()
        
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
        

    def preprocess(self, img):
        """
        Adapted from yolov5/utils/datasets.py LoadStreams class
        """
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0 
    

    def dr_callback(self, config, level):
        if not self.debug:
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


if __name__ == "__main__":

    # check_requirements(exclude=("tensorboard", "thop"))
    
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    
    rospy.spin()
