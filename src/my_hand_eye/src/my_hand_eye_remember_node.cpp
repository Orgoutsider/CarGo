#include "my_hand_eye/arm_controller.h"

my_hand_eye::ArmController controller;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_hand_eye_remember_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    controller.init(nh, pnh);
    double x, y, z;
    controller.remember(x, y, z);
    return 0;
}