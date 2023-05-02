#ifndef _LINE_FOLLOWER_H_
#define _LINE_FOLLOWER_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <motion_controller/lineConfig.h>
#include <motion_controller/Start.h>

namespace motion_controller
{
    class LineFollower
    {
    private:
        bool param_modification_; // 动态调参
        bool motor_status_;       // 调参，即停选项
        bool start_image_sub_;    // 是否开启订阅
        std::string transport_hint_;
        // 默认（320，240）图片
        const int width_ = 320;
        const int height_ = 240;
        // （320，240）图片截图
        int r_start_;
        int r_end_;
        int c_start_;
        int c_end_;
        int mask_r_start_; // 图片掩膜
        int mask_c_start_;
        // int threshold_;
        int Hough_threshold_;    // 根据实际调整，夜晚40，下午50
        int judge_line_;         // 判断线高度，0~(r_end_ - r_start_)
        double linear_velocity_; // 速度
        double theta_thr_horizontal_;     // theta与90度差别少于此值时判定为横线
        double rho_thr_;         // 滤除直线时，rho阈值
        double theta_thr_;       // theta阈值（单位度）
        double kp_;
        double kd_;
        cv::Scalar black_low_; // 黑色车道分割
        cv::Scalar black_up_;  // 夜晚80左右，下午100
        ros::Publisher cmd_vel_publisher_;
        ros::ServiceServer start_server_; // 开关回调函数服务端
        std::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::Subscriber image_subscriber_;
        dynamic_reconfigure::Server<lineConfig> dr_server_;
        // 去除错误直线，重新求rho和theta平均值
        void _clean_lines(cv::Vec2f lines[], int num, double &rho_aver, double &theta_aver);
        // 利用颜色排除直线
        bool _find_lines(cv_bridge::CvImagePtr &cv_image, geometry_msgs::Twist &twist);
        // bool _find_road(cv_bridge::CvImagePtr &cv_image, geometry_msgs::Twist &twist);
        void _image_callback(const sensor_msgs::ImageConstPtr &image_rect);
        void _dr_callback(lineConfig &config, uint32_t level);
        bool _do_start_req(Start::Request &req, Start::Response &resp);

    public:
        LineFollower(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        // 绘制HoughLines直线
        void plot_line(cv::Mat &mat, double rho, double theta, cv::Scalar color, double width = 1); 
    };

} // namespace motion_controller

#endif // !_LINE_FOLLOWER_H_
