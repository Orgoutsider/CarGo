#ifndef _ARM_CONTROLLER_H_
#define _ARM_CONTROLLER_H_
#include "my_hand_eye/pose.hpp"

#include <XmlRpcException.h>
#include <cv_bridge/cv_bridge.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <image_transport/image_transport.h>

namespace my_hand_eye
{
    enum Color
    {
        color_red = 1,
        color_green,
        color_blue
    };

    class ArmController
    {
    private:
        int current_color_;
        double current_z_;
        bool fin_;       // 是否找到指定颜色物料
        bool emulation_; // 是否进行仿真
        int histSize_ = 200;
        float histRange_[2] = {0, 255};
        int channels_[2] = {0, 1};
        Pos ps_;
        SMS_STS sm_st_;
        SCSCL sc_;
        ros::ServiceClient cargo_client_; // mmdetection+颜色识别
        ros::ServiceClient plot_client_;  // 运动范围绘制
        ros::Time last_time_;
        cv::Rect default_roi_; // 截图矩形
        cv::Rect rect_;        // CamShift算法要求要把目标物体的矩形框传递进来
        cv_bridge::CvImage cv_image_;
        std::vector<double> cargo_x_;
        std::vector<double> cargo_y_;
        std::vector<cv::Point> pt_;
        const float *ranges_;

    public:
        ArmController();
        ~ArmController();
        const double z_floor = 3.5;     // 底盘距地6mm，物块高度一半35mm
        const double z_turntable = 6.4; // 转盘
        bool show_detections_;
        void init(ros::NodeHandle nh, ros::NodeHandle pnh, bool emulation);                         // 初始化
        bool add_image(const sensor_msgs::ImageConstPtr &image_rect, cv_bridge::CvImagePtr &image); // 添加图片
        bool take_picture();                                                                        // 拍照
        bool detect_cargo(const sensor_msgs::ImageConstPtr &image_rect, vision_msgs::BoundingBox2DArray &detections,
                          sensor_msgs::ImagePtr &debug_image, cv::Rect &rect); // 向物块检测服务器发送请求
        bool log_position_main(const sensor_msgs::ImageConstPtr &image_rect, double z,
                               sensor_msgs::ImagePtr &debug_image);
        bool find_with_color(vision_msgs::BoundingBox2DArray &objArray, const int color,
                             double z, double &x, double &y); // 处理接收的图片，通过颜色确定位置
        void average_position(double &x, double &y);          // 求得记录位置数据的平均值
        bool catch_straightly(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
                              bool &finish, sensor_msgs::ImagePtr &debug_image, bool midpoint = false);
        bool catch_with_2_steps(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
                                bool &finish, sensor_msgs::ImagePtr &debug_image);
        bool remember(double &x, double &y, double &z); // 记忆位置
        bool target_init(vision_msgs::BoundingBox2DArray &objArray, const int color,
                         cv::Mat &dstHist); // 目标初始化
        // 计算物体速度（像素/实际），将点push_back进pt_
        bool calculate_speed(double u, double v, double &speed);
        // CamShift算法，目标追踪
        bool target_tracking(const sensor_msgs::ImageConstPtr &image_rect, const int color,
                             double &u, double &v, bool &stop, sensor_msgs::ImagePtr &debug_image);
        double distance_min(vision_msgs::BoundingBox2DArray &objArray, const int color,
                            double x, double y, double z); // 障碍物最短距离
        bool find_points_with_height(double h, bool done, bool expand_y = false);
        // 椭圆识别
        bool ellipse_target_find(const sensor_msgs::ImageConstPtr &image_rect,
                                 sensor_msgs::ImagePtr &debug_image, cv::Rect &roi);
        bool put_with_ellipse(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
                              bool &finish, sensor_msgs::ImagePtr &debug_image);
    };

    // 椭圆圆心十字光标绘制，用于调试观察
    void draw_cross(cv::Mat &img, cv::Point2f point, cv::Scalar color, int size, int thickness);

} // namespace my_hand_eye

#endif // !_ARM_CONTROLLER_H_