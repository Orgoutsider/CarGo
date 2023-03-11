#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <simple_follower/status.h>

#include "opencv2/opencv.hpp"

using namespace cv;

geometry_msgs::Twist cmd_vel_msg; // 速度控制信息数据
simple_follower::status stts;
bool flag_adj = false;// 有没有判断进入弯道调整状态
bool flag_cor = false;// 有没有判断进入弯道
double last_err;
/**************************************************************************
函数功能：底盘运动sub回调函数（原始数据）
入口参数：cmd_msg  auto_drive.cpp
返回  值：无
**************************************************************************/
void cmd_vel_ori_Callback(const geometry_msgs::TwistConstPtr &msg)
{
    cmd_vel_msg.linear.x = msg->linear.x;
    cmd_vel_msg.angular.z = msg->angular.z;
}

/**************************************************************************
函数功能：校正图片sub回调函数（原始数据）
入口参数：status_p  corners_judge.cpp
返回  值：无
**************************************************************************/
void judge_Callback(const simple_follower::statusConstPtr &status_p)
{
    if(flag_cor)return;
    last_err = stts.err;
    stts = *status_p;
    if(status_p->judge == status_p->ADJUSTING && !flag_adj)
    {
        flag_adj = true;
        last_err = status_p->err;
    }
    if(status_p->judge == status_p->CORNER && !flag_cor)
    {
        flag_cor = true;
    }
}

/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "corners"); // 初始化ROS节点

    ros::NodeHandle node; // 创建句柄
    ros::NodeHandle node_private("~"); // 创建句柄
    double kp = node_private.param<double>("kp",0.003);
    double kd = node_private.param<double>("kd",0.03);
    bool turn_left = node_private.param<bool>("turn_left",false);
    int sgn = turn_left ? 1 : -1;
    /***创建底盘速度控制话题发布者***/
    ros::Publisher cmd_vel_Pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    /***创建底盘运动话题订阅者***/
    ros::Subscriber vel_sub = node.subscribe<geometry_msgs::Twist>("cmd_vel_ori", 1, cmd_vel_ori_Callback);

    /***创建弯道检测话题订阅者***/
    ros::Subscriber judge_sub = node.subscribe<simple_follower::status>("corners_judge", 1, judge_Callback);

    double rate2 = 30; // 频率30Hz
    ros::Rate loopRate2(rate2);
    ros::Duration du(3.1);//持续3.1秒钟,参数是double类型的，以秒为单位
    while (ros::ok())
    {
        ros::spinOnce();
        switch (stts.judge)
        {
            case stts.NORMAL:
            cmd_vel_Pub.publish(cmd_vel_msg);break; // 将速度指令发送给机器人
            case stts.ADJUSTING:
            cmd_vel_msg = geometry_msgs::Twist();
            cmd_vel_msg.linear.x = kp * stts.err + kd * (stts.err - last_err);
            cmd_vel_Pub.publish(cmd_vel_msg);break;
            case stts.CORNER:
            if (flag_cor)
            {
                cmd_vel_msg = geometry_msgs::Twist();
                cmd_vel_msg.angular.z = sgn * 0.5;
                cmd_vel_Pub.publish(cmd_vel_msg);
                du.sleep();//按照指定的持续时间休眠
                cmd_vel_msg.angular.z = 0;cmd_vel_msg.linear.x = 0.1;
                cmd_vel_Pub.publish(cmd_vel_msg);
                flag_cor = false;
                flag_adj = false;
                du.sleep();
            }
            else
            {
                cmd_vel_Pub.publish(cmd_vel_msg);  
            }
            break;
            default:
            cmd_vel_Pub.publish(geometry_msgs::Twist());
            ROS_ERROR("Msg error!");break;
        }
        // ROS_INFO_STREAM(status);
        ros::spinOnce();
        loopRate2.sleep();
    }

    return 0;
}
