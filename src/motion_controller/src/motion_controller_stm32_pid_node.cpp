#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <motion_controller/params_PID_stm32Config.h>

ros::Publisher pub;

void cb(motion_controller::params_PID_stm32Config &config, uint32_t level)
{
    geometry_msgs::Twist v;
    if (config.motor_status)
        v.linear.x = config.target_speed;
    pub.publish(v);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motion_controller_stm32_pid_node");
    ros::NodeHandle nh;
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_line", 3);
    dynamic_reconfigure::Server<motion_controller::params_PID_stm32Config> srv;
    srv.setCallback(boost::bind(&cb, _1, _2));
    ros::spin();
    return 0;
}
