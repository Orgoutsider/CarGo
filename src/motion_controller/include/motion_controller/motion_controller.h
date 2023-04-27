#ifndef _MOTION_CONTROLLER_H_
#define _MOTION_CONTROLLER_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_controller/MoveAction.h>

typedef actionlib::SimpleActionClient<motion_controller::MoveAction> Client;

namespace motion_controller
{
    class MotionController
    {
    private:
        bool param_modification_; // 动态调参
        bool motor_status_;       // 调参，即停选项
        bool clockwise_;          // 移动方向，是否顺时针
        // 默认（320，240）图片
        const int width_ = 320;
        const int height_ = 240;
        // （320，240）图片截图
        int r_start_;
        int r_end_;
        int c_start_;
        int c_end_;
        int threshold_;                               // 根据实际调整，夜晚40，下午50
        cv::Scalar black_low_;                        // 黑色车道分割
        cv::Scalar black_up_;                         // 夜晚80左右，下午100
        int y_goal_;                                  // 目标位置
        int y_ground_;                                // 地平线对应的y
        int cnt_tolerance_;                           // 由于干扰，可能存在误判
        double distance_thr_;                         // 等效阈值小于此值时判定退出pid，转弯
        geometry_msgs::PointStamped point_footprint_; // 设置位置，车在footprint坐标系中真实位置
        std::shared_ptr<image_transport::ImageTransport> it_;
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;
        image_transport::Subscriber image_subscriber_; // 弯道检测订阅者
        ros::Publisher vision_publisher;               // 视觉信息发布者
        ros::Timer timer_;                             // 全局定位定时器
        Client client_;                                // move client
        void _turn(bool left);
        void _image_callback(const sensor_msgs::ImageConstPtr &image_rect);
        void _timer_callback(const ros::TimerEvent &event);

    public:
        // 设置当前位置
        bool set_position(double x, double y);
        // 获取当前位置
        bool get_position(double &x, double &y);
        MotionController(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    };

} // namespace motion_controller

#endif // !_MOTION_CONTROLLER_H_
