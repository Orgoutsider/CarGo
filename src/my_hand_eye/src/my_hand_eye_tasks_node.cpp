#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char *argv[])
{
    if (argc < 2)
        return 1;
    ros::init(argc, argv, "my_hand_eye_tasks_node");
    ros::NodeHandle nh;
    ros::Publisher QR_code_pub = nh.advertise<std_msgs::String>("/barcode", 10);
    if (QR_code_pub.getNumSubscribers() > 0)
    {
        std_msgs::String msg;
        msg.data = argv[1];
        QR_code_pub.publish(msg);
    }
    else
    {
        ROS_WARN("QR code subscriber is not available yet.");
        return 1;
    }
    return 0;
}
