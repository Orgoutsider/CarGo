#ifndef _ARM_CONTROLLER_H_
#define _ARM_CONTROLLER_H_
#include "my_hand_eye/ellipse.h"
#include "my_hand_eye/border.h"

#include <yolov5_ros/cargoSrv.h>
#include <XmlRpcException.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

namespace my_hand_eye
{
    class ArmController
    {
    private:
        int current_color_;
        double current_z_;
        double speed_standard_; // 速度标准，当速度小于此标准足够多次数时，判定为静止
        bool fin_;              // 是否找到指定颜色物料
        bool emulation_;        // 是否进行仿真或摄像头测试
        EllipseColor ellipse_color_order_[4];
        ColorTracker tracker_;
        Border border_;
        SMS_STS sm_st_;
        SCSCL sc_;
        ros::ServiceClient cargo_client_; // yolov5+颜色识别
        ros::ServiceClient plot_client_;  // 运动范围绘制
        cv::Rect default_roi_;            // 截图矩形
        cv_bridge::CvImage cv_image_;
        std::vector<double> cargo_x_;
        std::vector<double> cargo_y_;
        const int Gauss_size_ = 3; // 高斯平滑内核大小
        const int Canny_low_ = 50; // 第一次Canny边缘查找的第一滞后因子
        const int Canny_up_ = 100; // 第一次Canny边缘查找的第二滞后因子

        const int con_Area_min_ = 4500;   // 粗筛-最小面积阈值
        const int con_Point_cont_ = 20;   // 粗筛-图形最少点个数阈值，即连成某个封闭轮廓的点的个数，少于该阈值表明轮廓无效
        const int con_Area_max_ = 200000; // 粗筛-最大面积阈值
        bool add_image(const sensor_msgs::ImageConstPtr &image_rect,
                       cv_bridge::CvImagePtr &image); // 添加图片
        bool detect_cargo(const sensor_msgs::ImageConstPtr &image_rect, vision_msgs::BoundingBox2DArray &detections,
                          sensor_msgs::ImagePtr &debug_image, cv::Rect &rect); // 向物块检测服务器发送请求
        // 处理接收的图片，通过颜色确定位置，注意objArray中的数据对应的是原图
        bool find_with_color(vision_msgs::BoundingBox2DArray &objArray, const int color,
                             double z, double &x, double &y);
        // 计算物料转动半径
        bool calculate_radius_and_speed(double &u, double &v, double &radius, double &speed);
        bool take_picture(); // 拍照
        // 处理接收的图片，求3物料重心
        bool get_center(vision_msgs::BoundingBox2DArray &objArray, double &center_u, double &center_v,
                        double &center_x, double &center_y, bool read = true);
        // 中心点按从左往右排序
        bool set_ellipse_color_order(vision_msgs::BoundingBox2DArray &objArray); // 处理接收的图片，设置椭圆颜色顺序
        void average_position(double &x, double &y);                             // 求得记录位置数据的平均值
        double distance_min(vision_msgs::BoundingBox2DArray &objArray, const int color,
                            double x, double y, double z); // 障碍物最短距离
        // 判断物块是否静止
        bool cargo_is_static(double speed, bool reset);
        bool ellipse_target_find(const sensor_msgs::ImageConstPtr &image_rect,
                                 cv::Rect &roi, vision_msgs::BoundingBox2DArray &objArray,
                                 sensor_msgs::ImagePtr &debug_image);

    public:
        ArmController();
        ArmController(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        ~ArmController();
        Pos ps_;
        int white_vmin_;
        // double proportion_; // 杂色所占的比例
        // const double z_turntable = 16.4750; // 老转盘
        // const double z_turntable = 15.57;   // 新转盘
        const double z_turntable = 14.4654; // 比赛转盘，抓取范围17.3到34
        bool show_detections_;
        void init(ros::NodeHandle &nh, ros::NodeHandle &pnh); // 初始化
        bool log_position(const sensor_msgs::ImageConstPtr &image_rect, double z, int color,
                          sensor_msgs::ImagePtr &debug_image, bool center = false);
        bool log_extrinsics_correction(const sensor_msgs::ImageConstPtr &image_rect,
                                       double correct_x, double correct_y, double correct_z, int color,
                                       sensor_msgs::ImagePtr &debug_image);
        bool catch_straightly(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
                              bool &finish, sensor_msgs::ImagePtr &debug_image, bool left, bool hold = false,
                              bool midpoint = false);
        // bool catch_with_2_steps(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
        // bool &finish, sensor_msgs::ImagePtr &debug_image);
        bool remember(double &x, double &y, double &z); // 记忆位置
        // 目标检测到物料并目标追踪
        bool track(const sensor_msgs::ImageConstPtr &image_rect, const int color,
                   double &u, double &v, bool &stop, sensor_msgs::ImagePtr &debug_image);
        bool find_points_with_height(double h, bool done);
        // 椭圆识别，摄像头测试时z无效
        bool put_with_ellipse(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
                              bool &finish, sensor_msgs::ImagePtr &debug_image);
        // 查看边界线位置
        bool find_border(const sensor_msgs::ImageConstPtr &image_rect, double &distance, double &yaw,
                         bool &finish, sensor_msgs::ImagePtr &debug_image);
        // 计算圆盘中心点位置
        bool find_center(const sensor_msgs::ImageConstPtr &image_rect, double &x, double &y,
                         bool &finish, sensor_msgs::ImagePtr &debug_image);
    };
} // namespace my_hand_eye

#endif // !_ARM_CONTROLLER_H_
