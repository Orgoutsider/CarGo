#pragma once
#define ARM_JOINT1_POS_WHEN_DEG0 813.8
#define ARM_JOINT1_POS_WHEN_DEG180 200
#define ARM_JOINT234_POS_WHEN_DEG180 3071.25
#define ARM_JOINT234_POS_WHEN_DEG0 1023.75
#define ARM_JOINT5_POS_WHEN_CATCH 370
#define ARM_JOINT5_POS_WHEN_LOSEN 450
#define ARM_INFO_XYZ(Pos) ROS_INFO_STREAM("x:" << (Pos).x << " y:" << (Pos).y << " z:" << (Pos).z)
#include "opencv2/opencv.hpp"

#include "my_hand_eye/SCServo.h"
#include "my_hand_eye/backward_kinematics.hpp"
#include "mmdetection_ros/cargoSrv.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <XmlRpcException.h>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <image_transport/image_transport.h>

namespace my_hand_eye{
    struct ArmPose
    {
        cv::Mat R;
        cv::Mat t;
        bool empty;
        ArmPose();
    };
    
    class Pos : public Axis
    {
    private:
        bool cat;//cat=true抓
        bool look;//look=true观察
        s16 Position[6] = {0};//均是大小为6的数组
        u16 Speed[6] = {0};
        u8 ACC[6] = {0};
        u8 ID[6] = {0, 1, 2, 3, 4, 5};
        SMS_STS *sm_st_ptr;//舵机
        SCSCL *sc_ptr;
        double default_x, default_y, default_z;
        const double fx = 1097.2826519484;
        const double fy = 1093.696551235277;
        const double cx = 953.2646757356512;
        const double cy = 501.2671482091341;
        cv::Mat R_cam_to_end = (cv::Mat_<double>(3, 3) << 0.01762304284718308, 0.05031655330790039, 0.998577825121317,
                                                            0.9994569158454911, 0.02692675460875993, -0.01899534824262144,
                                                            -0.02784424050724299, 0.9983702691634265, -0.04981469583488352);
        cv::Mat T_cam_to_end = (cv::Mat_<double>(3, 1) << -4.76346677244081, -1.372151631737261, -60.00245432936754);
    public:
        Pos (SMS_STS *sm_st_ptr, SCSCL *sc_ptr, bool cat=false, bool look=true);//初始化
        bool begin(const char *argv);//打开串口
        void ping();
        void set_speed_and_acc(XmlRpc::XmlRpcValue& servo_descriptions);//获取速度加速度
        void set_default_position(XmlRpc::XmlRpcValue& default_action);
        bool calculate_position();
        //计算各joint运动的position
        bool go_to(double x, double y, double z, bool cat, bool look);//运动到指定位置，抓/不抓
        bool reset();
        bool read_position(int ID);//读指定舵机位置
        bool read_all_position();//读所有舵机正确位置
        bool refresh_xyz();//更新位置
        cv::Mat R_end_to_base();//机械臂末端到基底的·旋转矩阵（不保证实时性）
        cv::Mat T_end_to_base();//机械臂末端到基底的·平移向量（不保证实时性）
        cv::Mat Intrinsics();//内参
        ArmPose end_to_base_now();//更新位置，并返回旋转矩阵，平移向量
        bool calculate_cargo_position(double u, double v, double z, double &x, double &y);
        void end();
    };

    bool generate_valid_position(double deg1, double deg2, double deg3, double deg4, double &x, double &y, double &z, bool &look);//根据舵机角度生成位姿
    cv::Mat R_T2homogeneous_matrix(const cv::Mat& R,const cv::Mat& T);

    class ArmController
    {
    private:
        Pos ps_;
        SMS_STS sm_st_;
        SCSCL sc_;
        ros::ServiceClient cargo_client_;
        cv::Mat img_;
        const int red = 1, green = 2, blue = 3;
    public:
        ArmController();
        ~ArmController();
        bool show_detections_;
        void init(ros::NodeHandle nh, ros::NodeHandle pnh);
        bool draw_image(const cv_bridge::CvImagePtr& image);
        bool detect_cargo(const sensor_msgs::ImageConstPtr& image_rect, vision_msgs::BoundingBox2DArray& detections, sensor_msgs::ImagePtr& debug_image );
        bool log_position_main(const sensor_msgs::ImageConstPtr &image_rect, double z, sensor_msgs::ImagePtr &debug_image);
        bool find_with_color(vision_msgs::BoundingBox2DArray& objArray, const int color, double z, double &x, double &y);
    };
    
}