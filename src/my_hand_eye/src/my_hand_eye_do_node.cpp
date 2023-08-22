#include "my_hand_eye/arm_server.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_hand_eye_do_node");

    my_hand_eye::ArmServer server;

    ros::spin();
}