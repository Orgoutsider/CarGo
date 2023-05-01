#include "my_hand_eye/my_eye.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_hand_eye_do_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    my_hand_eye::MyEye my_eye(nh, pnh);

    ros::spin();
}