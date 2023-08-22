#include "my_hand_eye/arm_server.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_hand_eye_do_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    my_hand_eye::ArmServer server(nh, pnh);

    ros::spin();
}