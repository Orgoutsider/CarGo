#! /usr/bin/env python

import rospy
from colormath.color_objects import LabColor
from colormath.color_diff import delta_e_cie2000

if __name__ == "__main__":
    # 1.初始化节点
    rospy.init_node("my_hand_eye_plot_srv_node")
    # 2.创建服务端对象
    # srv = rospy.Service("height_plot", Plot, do_plot)
    rospy.spin()  # 4.循环
