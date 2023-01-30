#! /usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np

from my_hand_eye.srv import Plot, PlotRequest, PlotResponse

global y, z, has_midpoint
y = np.array([])
z = np.array([])
has_midpoint = np.array([])


def do_plot(req):
    arr = req.arr
    points = arr.points
    global y, z, has_midpoint
    y = np.append(y, np.array([point.point.y for point in points]))
    z = np.append(z, np.array([point.point.z for point in points]))
    has_midpoint = np.append(
        has_midpoint, np.array([point.has_midpoint for point in points]))
    if req.done:
        h1 = plt.scatter(y[has_midpoint == True],
                         z[has_midpoint == True], c="red")
        h2 = plt.scatter(y[has_midpoint == False],
                         z[has_midpoint == False], c="blue")
        plt.legend(handles=[h1, h2], labels=[
                   'has', 'does not have'], loc='best')
        plt.show()

    return PlotResponse()


if __name__ == "__main__":
    # 1.初始化节点
    rospy.init_node("my_hand_eye_plot_srv_node")
    # 2.创建服务端对象
    srv = rospy.Service("height_plot", Plot, do_plot)
    rospy.spin()  # 4.循环
