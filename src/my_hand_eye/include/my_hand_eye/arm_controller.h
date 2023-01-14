#ifndef _ARM_CONTROLLER_H_
#define _ARM_CONTROLLER_H_
#include "my_hand_eye/pose.hpp"

#include <XmlRpcException.h>
#include <cv_bridge/cv_bridge.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <image_transport/image_transport.h>

namespace my_hand_eye
{
    class ArmController
    {
    private:
        int current_color_; double current_z_;
        bool fin_;
        int histSize_ = 200;
        float histRange_[2] = { 0,255 };
        int channels_[2] = { 0,1 };
        Pos ps_;
        SMS_STS sm_st_;
        SCSCL sc_;
        ros::ServiceClient cargo_client_;
        cv::Rect default_roi_;//截图矩形
        cv::Rect rect_;//CamShift算法要求要把目标物体的矩形框传递进来
        cv_bridge::CvImage cv_image_;
        std::vector<double> cargo_x_;
        std::vector<double> cargo_y_;
        std::vector<cv::Point> pt_;
        const float* ranges_;
    public:
        ArmController();
        ~ArmController();
        const int red = 1, green = 2, blue = 3;
        bool show_detections_;
        void init(ros::NodeHandle nh, ros::NodeHandle pnh);//初始化
        bool add_image(const sensor_msgs::ImageConstPtr &image_rect, cv_bridge::CvImagePtr& image);//添加图片
        bool detect_cargo(const sensor_msgs::ImageConstPtr& image_rect, vision_msgs::BoundingBox2DArray& detections, 
                            sensor_msgs::ImagePtr& debug_image, cv::Rect& rect);//向物块检测服务器发送请求
        bool log_position_main(const sensor_msgs::ImageConstPtr &image_rect, double z, 
                            sensor_msgs::ImagePtr &debug_image);
        bool find_with_color(vision_msgs::BoundingBox2DArray& objArray, const int color, 
                            double z, double &x, double &y);//处理接收的图片，通过颜色确定位置
        void average_position(double &x, double &y);//求得记录位置数据的平均值
        bool catch_straightly(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
                            bool &finish, sensor_msgs::ImagePtr &debug_image, bool midpoint=false);
        bool catch_with_2_steps(const sensor_msgs::ImageConstPtr& image_rect, const int color, double z, 
                            bool& finish, sensor_msgs::ImagePtr &debug_image);
        bool remember(double &x, double &y, double &z);//记忆位置
        bool target_init(vision_msgs::BoundingBox2DArray& objArray, const int color, 
                            cv::Mat& dstHist);//目标初始化
        //CamShift算法，目标追踪
        bool target_tracking(const sensor_msgs::ImageConstPtr &image_rect, const int color, 
                            double& u, double &v, bool& stop, sensor_msgs::ImagePtr &debug_image);
    };
    
} // namespace my_hand_eye

#endif // !_ARM_CONTROLLER_H_