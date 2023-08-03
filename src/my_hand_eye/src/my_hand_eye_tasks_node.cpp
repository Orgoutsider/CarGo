#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char *argv[])
{
    if (argc < 2)
        return 1;
    ros::init(argc, argv, "my_hand_eye_tasks_node");
    ros::NodeHandle nh;
    ros::Publisher QR_code_pub = nh.advertise<std_msgs::String>("/barcode", 10);
    ros::Rate rate(3);
    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = argv[1];
        QR_code_pub.publish(msg);
        rate.sleep();
    }
    return 0;
}
