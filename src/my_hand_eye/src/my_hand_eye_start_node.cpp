#include "my_hand_eye/arm_controller.h"

my_hand_eye::ArmController controller;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "my_hand_eye_start_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // pnh.setParam("emulation", true);
    controller.init(nh, pnh);
    controller.start();
    // controller.ready(false);
    // controller.start();
    return 0;
}
