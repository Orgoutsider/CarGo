#ifndef _VISION_CONTROLLER_H_
#define _VISION_CONTROLLER_H_

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/server.h>
#include <my_hand_eye/ArmAction.h>
#include <motion_controller/MoveAction.h>
#include <motion_controller/cornersConfig.h>
#include <motion_controller/Start.h>
#include <motion_controller/Distance.h>
#include <motion_controller/Go.h>

#include "motion_controller/field_guide.h"

namespace motion_controller
{
    typedef actionlib::SimpleActionClient<motion_controller::MoveAction> MoveClient;

    typedef actionlib::SimpleActionClient<my_hand_eye::ArmAction> ArmClient;

    class VisionController : public FieldGuide
    {
    private:
        bool debug_; // 动态调参
        bool startup_;       // 调参，即停选项
        std::string transport_hint_;
        // 默认（320，240）图片
        const int width_ = 320;
        const int height_ = 240;
        int cnt_;
        // （320，240）图片截图
        int r_start_;
        int r_end_;
        int c_start_;
        int c_end_;
        int mask_c_start1_; // 图片掩膜
        int mask_c_start2_;
        int threshold_;         // 根据实际调整，夜晚40，下午50
        cv::Scalar black_low_;  // 黑色车道分割
        cv::Scalar black_up_;   // 夜晚80左右，下午100
        cv::Scalar yellow_low_; // 黄色
        cv::Scalar yellow_up_;
        cv::Scalar grey_low_; // 灰色
        cv::Scalar grey_up_;
        double theta_thr_;    // theta与90度差别少于此值时判定为横线
        int y_goal_;          // 目标位置
        int y_ground_;        // 地平线对应的y
        int cnt_tolerance_;   // 由于干扰，可能存在误判
        double y_thr_;        // 排除与平均值相差较大的线
        double distance_thr_; // 等效阈值小于此值时判定退出pid，转弯
        double delta_x_;      // 设置位置时，真实坐标与tf坐标转换关系的偏移量
        double delta_y_;
        double delta_theta_;
        bool finish_turning_; // 转弯之后关闭弯道视觉订阅
        std::shared_ptr<image_transport::ImageTransport> it_;
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;
        image_transport::Subscriber image_subscriber_; // 弯道检测订阅者
        ros::Publisher vision_publisher;               // 视觉信息发布者
        ros::Timer timer_;                             // 全局定位定时器
        MoveClient ac_move_;                           // 移动服务客户端
        ArmClient ac_arm_;
        dynamic_reconfigure::Server<motion_controller::cornersConfig> dr_server_;
        ros::ServiceServer go_client_;    // 一键式客户端
        ros::ServiceClient start_client_; // 开启循线客户端
        void _clean_lines(double y[], double &y_sum, int &tot);
        bool _color_judge(cv::Mat &img, bool note[], cv::Vec2f &line);
        // 转弯
        bool _turn();
        // 掉头，需要改变之后的转弯方向
        void _U_turn();
        // 通过全局定位信息转弯
        bool _turn_by_position();
        // 此算法要注意弯道外界，场地外有黑色物体的情况
        void _image_callback(const sensor_msgs::ImageConstPtr &image_rect);
        void _timer_callback(const ros::TimerEvent &event);
        void _dr_callback(motion_controller::cornersConfig &config, uint32_t level);
        void _done_callback(const actionlib::SimpleClientGoalState &state, const my_hand_eye::ArmResultConstPtr &result);
        void _active_callback();
        void _feedback_callback();

    public:
        VisionController(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        // 设置当前位置
        bool set_position(double x, double y, double theta);
        // 获取当前位置
        bool get_position();
        // 开/关循线回调
        bool start_vision_follower(bool start);
        void start_image_subscriber(bool start);
        bool go(Go::Request &req, Go::Response &resp);
        void plot_line(cv::Mat &mat, double rho, double theta, cv::Scalar color);
    };

} // namespace motion_controller

#endif // !_VISION_CONTROLLER_H_
