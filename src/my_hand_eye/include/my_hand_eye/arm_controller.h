#ifndef _ARM_CONTROLLER_H_
#define _ARM_CONTROLLER_H_
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "my_hand_eye/square.h"
#include "my_hand_eye/ellipse.h"
#include "my_hand_eye/border.h"
#include "my_hand_eye/target_pose.h"

namespace my_hand_eye
{
    class ArmController : public BorderMethod
    {
    private:
        double speed_standard_static_; // 速度标准，当速度小于此标准足够多次数时，判定为静止
        double speed_standard_motion_; // 速度标准，当速度大于此标准足够多次数时，判定为运动
        bool emulation_;               // 是否进行仿真
        bool stop_;                    // 用于颜色追踪，物料是否已停
        bool can_catch_;               // 用于颜色追踪，物料是否可以抓取
        int white_vmin_;               // 用于滤除白色
        float fThScoreScore_;
        float fMinReliability_;
        Pos ps_;
        Color current_color_;
        EllipseColor color_order_[4]; // 颜色顺序（从左至右）
        std::map<Color, int> color_map_;
        ColorTracker tracker_;
        Border border_;
        SMS_STS sm_st_;
        SCSCL sc_;
        ros::NodeHandle *nh_;             // 节点句柄
        ros::ServiceClient cargo_client_; // yolov5+颜色识别
        ros::ServiceClient plot_client_;  // 运动范围绘制
        cv::Rect default_roi_;            // 默认截图矩形
        cv::Rect border_roi_;             // 边界截图矩形
        cv::Rect ellipse_roi_;            // 椭圆截图矩形
        cv_bridge::CvImage cv_image_;
        std::vector<double> cargo_x_;
        std::vector<double> cargo_y_;
        std::vector<double> cargo_theta_;
        cv::CEllipseDetectorYaed *yaed_;
        bool add_image(const sensor_msgs::ImageConstPtr &image_rect,
                       cv_bridge::CvImagePtr &image); // 添加图片
        bool detect_cargo(const sensor_msgs::ImageConstPtr &image_rect, vision_msgs::BoundingBox2DArray &detections,
                          sensor_msgs::ImagePtr &debug_image, cv::Rect &rect); // 向物块检测服务器发送请求
        bool detect_ellipse(const sensor_msgs::ImageConstPtr &image_rect, vision_msgs::BoundingBox2DArray &detections,
                            sensor_msgs::ImagePtr &debug_image, cv::Rect &rect);
        // 处理接收的图片，通过颜色确定位置，注意objArray中的数据对应的是原图
        bool find_with_color(vision_msgs::BoundingBox2DArray &objArray, const Color color,
                             double z, double &x, double &y);
        // 计算物料转动半径
        bool calculate_radius_and_speed(double &u, double &v, double &x, double &y, double &radius, double &speed);
        bool take_picture(); // 拍照
        // 处理接收的图片，求3物料重心
        bool get_center(vision_msgs::BoundingBox2DArray &objArray, double &center_u, double &center_v,
                        double &center_x, double &center_y, bool read = true);
        // 处理接收的图片，求相对椭圆位姿
        bool get_pose(vision_msgs::BoundingBox2DArray &objArray,
                      double z, geometry_msgs::Pose2D &pose, bool rst);
        bool set_color_order(vision_msgs::BoundingBox2DArray &objArray); // 处理接收的图片，设置椭圆颜色顺序中心点按从左往右排序
        void average_position(double &x, double &y);                     // 求得记录位置数据的平均值
        void average_theta(double &theta);                               // 求得记录位置数据的平均值
        void average_pose(geometry_msgs::Pose2D &pose);                  // 求得记录位置数据的平均值
        // 求平均位姿并存入变长数组
        void average_pose_once();
        // double distance_min(vision_msgs::BoundingBox2DArray &objArray, const Color color,
        //                     double x, double y, double z); // 障碍物最短距离
        // 判断物块是否静止
        bool cargo_is_static(double speed, bool reset, double x, double y);

    public:
        ArmController();
        ArmController(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        ~ArmController();
        TargetPose target_pose; // 用于视觉位姿调节
        bool catched;           // 是否已抓取过
        // double proportion_; // 杂色所占的比例
        const double z_turntable;
        const double z_ellipse;
        const double z_palletize;
        double z_parking_area;
        int threshold;
        bool show_detections;
        void init(ros::NodeHandle &nh, ros::NodeHandle &pnh); // 初始化
        bool log_cargo(const sensor_msgs::ImageConstPtr &image_rect, Color color, double z,
                       sensor_msgs::ImagePtr &debug_image, bool center = false, bool pose = false);
        // 校正外参
        bool log_extrinsics_correction(const sensor_msgs::ImageConstPtr &image_rect,
                                       double correct_x, double correct_y, double correct_z, Color color,
                                       sensor_msgs::ImagePtr &debug_image);
        // 直接抓取
        bool catch_straightly(const sensor_msgs::ImageConstPtr &image_rect, const Color color,
                              bool &finish, sensor_msgs::ImagePtr &debug_image, bool left, bool hold = false,
                              bool midpoint = false);
        bool remember(double &x, double &y, double &z, double &tightness); // 记忆位置
        // 目标检测到物料并目标追踪
        bool track(const sensor_msgs::ImageConstPtr &image_rect, const Color color, bool &first,
                   double &x, double &y, sensor_msgs::ImagePtr &debug_image);
        // 跟踪后抓取，配合catch()使用
        bool catch_after_tracking(double x, double y, const Color color, const Color color_next,
                                  bool left, bool &finish);
        bool find_points_with_height(double h, bool done);
        // 椭圆识别，摄像头测试
        bool log_ellipse(const sensor_msgs::ImageConstPtr &image_rect, const Color color,
                         sensor_msgs::ImagePtr &debug_image, bool pose = false);
        // 固定位置放置
        bool put(const Color color, bool pal, bool final);
        // 固定位置抓取
        bool catch_after_putting(const Color color, bool final);
        // 输出边界线位置
        bool log_border(const sensor_msgs::ImageConstPtr &image_rect, sensor_msgs::ImagePtr &debug_image);
        // 扫描二维码后准备抓取
        void ready(bool left);
        // 计算物料中心点位置或位姿
        bool find_cargo(const sensor_msgs::ImageConstPtr &image_rect, Pose2DMightEnd &msg,
                        sensor_msgs::ImagePtr &debug_image, bool pose, double theta, bool store = false);
        // 计算椭圆位置
        bool find_ellipse(const sensor_msgs::ImageConstPtr &image_rect, Pose2DMightEnd &msg,
                          sensor_msgs::ImagePtr &debug_image, bool store, double theta);
        // 计算边界线位置
        bool find_border(const sensor_msgs::ImageConstPtr &image_rect, Pose2DMightEnd &msg,
                         sensor_msgs::ImagePtr &debug_image);
        // 计算停车区位置
        bool find_parking_area(const sensor_msgs::ImageConstPtr &image_rect, Pose2DMightEnd &msg,
                               sensor_msgs::ImagePtr &debug_image);
    };
} // namespace my_hand_eye

#endif // !_ARM_CONTROLLER_H_
