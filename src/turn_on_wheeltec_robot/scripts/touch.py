#!/usr/bin/env python
# encoding: utf-8

import Jetson.GPIO as GPIO
import rospy
import time
from motion_controller.srv import Go, GoRequest

button_pin = 11  # 开关管脚pin
already_go = False


def cb(channel):
    global already_go
    if already_go:
        return
       
    client = rospy.ServiceProxy("Go", Go)
    client.wait_for_service()
    req = GoRequest()
    client.call(req)
    already_go = True


# GPIO定义
def setup():
    GPIO.setmode(GPIO.BOARD)  # 采用实际的物理管脚给GPIO口
    GPIO.setwarnings(False)  # 忽略GPIO操作注意警告
    GPIO.setup(button_pin, GPIO.IN)  # pull_up_down=GPIO.PUD_UP) # 输入模式，上拉至高电平
    GPIO.add_event_detect(button_pin, GPIO.FALLING,
                          callback=cb, bouncetime=200)


if __name__ == '__main__':
    rospy.init_node("touch")
    setup()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        time.sleep(2)

    GPIO.cleanup()
