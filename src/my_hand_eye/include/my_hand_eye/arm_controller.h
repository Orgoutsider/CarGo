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

    struct Ellipse
    {
        int color;
        double center_x;
    };

    class ArmController
    {
    private:
        int current_color_;
        double current_z_;
        double speed_standard_; // 速度标准，当速度小于此标准足够多次数时，判定为静止
        bool fin_;              // 是否找到指定颜色物料
        bool emulation_;        // 是否进行仿真
        int white_vmin_;
        int ellipse_color_order_[4];
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
        const int histSize_ = 200;
        const float histRange_[2] = {0, 255};
        const int channels_[2] = {0, 1};
        const float *ranges_;
        const int Gauss_size_ = 3; // 高斯平滑内核大小
        const int Canny_low_ = 50; // 第一次Canny边缘查找的第一滞后因子
        const int Canny_up_ = 100; // 第一次Canny边缘查找的第二滞后因子

        const int con_Area_min_ = 4500;   // 粗筛-最小面积阈值
        const int con_Point_cont_ = 20;   // 粗筛-图形最少点个数阈值，即连成某个封闭轮廓的点的个数，少于该阈值表明轮廓无效
        const int con_Area_max_ = 200000; // 粗筛-最大面积阈值

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
        // 处理接收的图片，通过颜色确定位置，注意objArray中的数据对应的是原图
        bool find_with_color(vision_msgs::BoundingBox2DArray &objArray, const int color,
                             double z, double &x, double &y);
        bool set_ellipse_color_order(vision_msgs::BoundingBox2DArray &objArray); // 处理接收的图片，设置椭圆颜色顺序
        void average_position(double &x, double &y);                             // 求得记录位置数据的平均值
        bool catch_straightly(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
                              bool &finish, sensor_msgs::ImagePtr &debug_image, bool midpoint = false);
        bool catch_with_2_steps(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
                                bool &finish, sensor_msgs::ImagePtr &debug_image);
        bool remember(double &x, double &y, double &z); // 记忆位置
        bool target_init(vision_msgs::BoundingBox2DArray &objArray, const int color,
                         cv::Mat &dstHist); // 目标初始化
        // 计算物体速度（像素/实际），将点push_back进pt_
        bool calculate_speed(double u, double v, double &speed);
        // 判断物块是否静止
        bool cargo_is_static(double speed, const int interval, bool reset);
        // CamShift算法，目标追踪
        bool target_tracking(const sensor_msgs::ImageConstPtr &image_rect, const int color,
                             double &u, double &v, bool &stop, sensor_msgs::ImagePtr &debug_image);
        double distance_min(vision_msgs::BoundingBox2DArray &objArray, const int color,
                            double x, double y, double z); // 障碍物最短距离
        bool find_points_with_height(double h, bool done, bool expand_y = false);
        // 椭圆识别
        bool ellipse_target_find(const sensor_msgs::ImageConstPtr &image_rect,
                                 cv::Rect &roi, vision_msgs::BoundingBox2DArray &objArray,
                                 sensor_msgs::ImagePtr &debug_image);
        bool put_with_ellipse(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
                              bool &finish, sensor_msgs::ImagePtr &debug_image);
    };

    // 将色相值映射为角度
    Angle hue_value(double h_val);
    // 色相平均值的计算
    double hue_value_tan(double y, double x);
    // 两色相的最小差值
    double hue_value_diff(double h_val1, double h_val2);
    // 椭圆圆心十字光标绘制，用于调试观察
    void draw_cross(cv::Mat &img, cv::Point2d point, cv::Scalar color, int size, int thickness);
    // 目标区域框选
    void generate_bounding_rect(int flag[], std::vector<cv::RotatedRect> &m_ellipses,
                                cv_bridge::CvImagePtr &cv_image, std::vector<cv::Rect> &RectTarget);
    double color_hypothesis(double h_val, int lower_bound, int upper_bound);
    // 颜色分类
    void color_classification(std::vector<cv::Rect> &RectTarget, cv_bridge::CvImagePtr &cv_image,
                              int white_vmin, std::vector<int> &color_id, std::vector<double> &hypothesis); // 中心点按从左往右排序
    bool ellipse_cmp(Ellipse e1, Ellipse e2);
} // namespace my_hand_eye

#endif // !_ARM_CONTROLLER_H_