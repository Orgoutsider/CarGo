#ifndef _LINE_FOLLOWER_H_
#define _LINE_FOLLOWER_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <motion_controller/lineConfig.h>

namespace motion_controller
{
    class LineFollower
    {
    private:
        bool param_modification_; // 动态调参
        bool motor_status_;               // 调参，即停选项
        // 默认（320，240）图片
        const int width_ = 320;
        const int height_ = 240;
        // （320，240）图片截图
        int r_start_;
        int r_end_;
        int c_start_;
        int c_end_;
        int threshold_;  // 根据实际调整，夜晚40，下午50
        int judge_line_; // 判断线高度，0~(r_end_ - r_start_)
        double kp_;
        double kd_;
        double linear_velocity_; // 速度
        cv::Scalar black_low_;   // 黑色车道分割
        cv::Scalar black_up_;    // 夜晚80左右，下午100
        ros::Publisher cmd_vel_publisher_;
        std::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::Subscriber image_subscriber_;
        dynamic_reconfigure::Server<lineConfig> server_;
        void _image_callback(const sensor_msgs::ImageConstPtr &image_rect);
        void _dr_callback(lineConfig &config, uint32_t level);

    public:
        LineFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    };

} // namespace motion_controller

#endif // !_LINE_FOLLOWER_H_
