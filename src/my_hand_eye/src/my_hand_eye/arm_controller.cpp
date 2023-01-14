#include "my_hand_eye/arm_controller.h"

namespace my_hand_eye
{
    ArmController::ArmController() : ps_(&sm_st_, &sc_), default_roi_(480, 0, 960, 1080),
                                     ranges_(histRange_), fin_(false)
    {
        cargo_x_.reserve(10);
        cargo_y_.reserve(10);
        pt_.reserve(100);
    };

    ArmController::~ArmController()
    {
        ps_.end();
    }

    void ArmController::init(ros::NodeHandle nh, ros::NodeHandle pnh)
    {
        XmlRpc::XmlRpcValue servo_descriptions;
        XmlRpc::XmlRpcValue default_action;
        XmlRpc::XmlRpcValue put_action;
        if (!pnh.getParam("servo", servo_descriptions))
        {
            ROS_ERROR("No speed and acc specified");
        }
        if (!pnh.getParam("default_action", default_action))
        {
            ROS_ERROR("No default action specified");
        }
        if (!pnh.getParam("put_action", put_action))
        {
            ROS_ERROR("No put action specified");
        }
        std::string ft_servo;
        pnh.param<std::string>("ft_servo", ft_servo, "/dev/ft_servo");
        ROS_INFO_STREAM("serial:" << ft_servo);
        if (!ps_.begin(ft_servo.c_str()))
        {
            ROS_ERROR_STREAM("Cannot open ft servo at" << ft_servo);
        }
        ps_.ping();
        ps_.set_speed_and_acc(servo_descriptions);
        ps_.set_action(default_action);
        ps_.set_action(put_action, "put");
        ps_.show_voltage();

        cargo_client_ = nh.serviceClient<mmdetection_ros::cargoSrv>("cargoSrv");
    }

    bool ArmController::add_image(const sensor_msgs::ImageConstPtr &image_rect, cv_bridge::CvImagePtr& image)
    {
        try
        {
            image = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }
        if (image->image.empty())
        {
            ROS_ERROR("No data!");
            return false;
        }
        cv_image_ = *image;
        // cv::cvtColor(image->image, image->image, cv::COLOR_RGB2BGR);
        return true;
    }

    bool ArmController::detect_cargo(const sensor_msgs::ImageConstPtr &image_rect,
                                     vision_msgs::BoundingBox2DArray &detections, sensor_msgs::ImagePtr &debug_image, cv::Rect &roi)
    {
        if (!cargo_client_.exists())
            cargo_client_.waitForExistence();
        cv_bridge::CvImagePtr cv_image;
        bool flag = add_image(image_rect, cv_image);
        cv_image->image = cv_image->image(roi);
        mmdetection_ros::cargoSrv cargo;
        sensor_msgs::ImagePtr image = (*cv_image).toImageMsg();
        cargo.request.image = *image;
        // 发送请求,返回 bool 值，标记是否成功
        if (flag)
            flag = cargo_client_.call(cargo);
        if (flag)
        {
            detections = cargo.response.results;

            if (show_detections_)
            {
                // cv::cvtColor(cv_image->image, cv_image->image, cv::COLOR_RGB2BGR);
                if (cargo.response.results.boxes.size())
                {
                    for (int color = 1; color <= 3; color++)
                    {
                        if (!cargo.response.results.boxes[color].center.x)
                            continue;
                        cv::RotatedRect rect(cv::Point2d(cargo.response.results.boxes[color].center.x, cargo.response.results.boxes[color].center.y),
                                             cv::Size2d(cargo.response.results.boxes[color].size_x, cargo.response.results.boxes[color].size_y),
                                             cargo.response.results.boxes[color].center.theta);
                        cv::Point2f vtx[4]; // 矩形顶点容器
                        // cv::Mat dst = cv::Mat::zeros(cv_image->image.size(), CV_8UC3);//创建空白图像
                        rect.points(vtx);
                        cv::Scalar colors; // 确定旋转矩阵的四个顶点
                        switch (color)
                        {
                        case 1:
                            colors = cv::Scalar(0, 0, 255);
                            break;
                        case 2:
                            colors = cv::Scalar(0, 255, 0);
                            break;
                        case 3:
                            colors = cv::Scalar(255, 0, 0);
                            break;
                        default:
                            break;
                        }
                        for (int j = 0; j < 4; j++)
                        {
                            cv::line(cv_image->image, vtx[j], vtx[(j + 1) % 4], colors, 2); // 随机颜色绘制矩形
                        }
                    }
                }
                // imshow("det", cv_image->image);
                debug_image = (*cv_image).toImageMsg();
            }
            if (detections.boxes.size())
            {
                for (int color = 1; color <= 3; color++)
                {
                    if (!detections.boxes[color].center.x)
                        continue;
                    detections.boxes[color].center.x += roi.x;
                    detections.boxes[color].center.y += roi.y;
                }
            }
        }
        else
            ROS_WARN("responce invalid!");
        return flag;
    }

    bool ArmController::log_position_main(const sensor_msgs::ImageConstPtr &image_rect, double z, sensor_msgs::ImagePtr &debug_image)
    {
        vision_msgs::BoundingBox2DArray objArray;
        cv::Rect rect(480, 0, 960, 1080);
        bool valid = detect_cargo(image_rect, objArray, debug_image, rect);
        if (valid)
        {
            double x = 0, y = 0;
            if (find_with_color(objArray, green, z, x, y))
                ROS_INFO_STREAM("x:" << x << " y:" << y);
        }
        return valid;
    }

    bool ArmController::find_with_color(vision_msgs::BoundingBox2DArray &objArray, const int color, double z, double &x, double &y)
    {
        // if (ps_.wait_time_ && !objArray.header.stamp.isZero() && objArray.header.stamp.toSec() < ps_.wait_time_)
        // {
        //     ROS_INFO("wait...");
        //     return false;
        // }
        if (objArray.boxes.size() == 4)
        {
            if (!objArray.boxes[color].center.x)
                return false;
            double u = objArray.boxes[color].center.x;
            double v = objArray.boxes[color].center.y;
            return ps_.calculate_cargo_position(u, v, z, x, y);
        }
        return false;
    }

    void ArmController::average_position(double &x, double &y)
    {
        x = std::accumulate(std::begin(cargo_x_), std::end(cargo_x_), 0.0) / cargo_x_.size();
        y = std::accumulate(std::begin(cargo_y_), std::end(cargo_y_), 0.0) / cargo_y_.size();
    }

    bool ArmController::catch_straightly(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
                                         bool &finish, sensor_msgs::ImagePtr &debug_image, bool midpoint)
    {
        if (!cargo_x_.size())
        {
            current_color_ = color;
            current_z_ = z;
            ps_.reset();
        }
        else if (current_color_ != color || current_z_ != z || cargo_x_.size() >= 10)
        {
            cargo_x_.clear();
            cargo_y_.clear();
            current_color_ = color;
            current_z_ = z;
            ps_.reset();
        }
        finish = false;
        vision_msgs::BoundingBox2DArray objArray;
        bool valid = detect_cargo(image_rect, objArray, debug_image, default_roi_);
        if (valid)
        {
            double x = 0, y = 0;
            if (find_with_color(objArray, current_color_, z, x, y))
            {
                ROS_INFO_STREAM("x:" << x << " y:" << y);
                cargo_x_.push_back(x);
                cargo_y_.push_back(y);
                if (cargo_x_.size() == 10)
                {
                    double x_aver = 0, y_aver = 0;
                    average_position(x_aver, y_aver);
                    cargo_x_.clear();
                    cargo_y_.clear();
                    if (midpoint)
                        valid = ps_.go_to_by_midpoint(x_aver, y_aver, current_z_);
                    else
                        valid = ps_.go_to_and_wait(x_aver, y_aver, current_z_, true);
                    if (valid)
                    {
                        ps_.go_to(ps_.default_x, ps_.default_y, ps_.default_z, true, true);
                        if (ps_.go_to_and_wait(ps_.put_x, ps_.put_y, ps_.put_z, false))
                            ps_.reset();
                    }
                    finish = true;
                }
            }
            else
                return false;
        }
        return valid;
    }

    bool ArmController::catch_with_2_steps(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
                                           bool &finish, sensor_msgs::ImagePtr &debug_image)
    {
        if (!cargo_x_.size())
        {
            current_color_ = color;
            current_z_ = z;
            ps_.reset();
        }
        else if (current_color_ != color || current_z_ != z || cargo_x_.size() >= 10)
        {
            cargo_x_.clear();
            cargo_y_.clear();
            current_color_ = color;
            current_z_ = z;
            ps_.reset();
        }
        finish = false;
        vision_msgs::BoundingBox2DArray objArray;
        bool valid = detect_cargo(image_rect, objArray, debug_image, default_roi_);
        if (valid)
        {
            double x = 0, y = 0;
            if (find_with_color(objArray, current_color_, z, x, y))
            {
                ROS_INFO_STREAM("x:" << x << " y:" << y);
                cargo_x_.push_back(x);
                cargo_y_.push_back(y);
                if (cargo_x_.size() == 5)
                {
                    double x_aver = 0, y_aver = 0;
                    average_position(x_aver, y_aver);
                    ps_.do_first_step(x_aver, y_aver);
                }
                else if (cargo_x_.size() == 10)
                {
                    double x_aver = 0, y_aver = 0;
                    average_position(x_aver, y_aver);
                    cargo_x_.clear();
                    cargo_y_.clear();
                    ps_.go_to_and_wait(x_aver, y_aver, current_z_, true);
                    finish = true;
                }
            }
            else
                return false;
        }
        return valid;
    }

    bool ArmController::remember(double &x, double &y, double &z)
    {
        if (ps_.refresh_xyz())
            ARM_INFO_XYZ(ps_);
        else
            return false;
        x = ps_.x;
        y = ps_.y;
        z = ps_.z;
        return true;
    }

    bool ArmController::target_init(vision_msgs::BoundingBox2DArray &objArray, const int color, cv::Mat &dstHist)
    {
        using namespace cv;
        if (objArray.boxes.size() == 4)
        {
            if (!objArray.boxes[color].center.x)
                return false;
            double width = objArray.boxes[color].size_x;
            double height = objArray.boxes[color].size_y;
            rect_ = Rect(objArray.boxes[color].center.x - width / 2, objArray.boxes[color].center.y - height / 2, width, height);
            Mat rectImg, targetImgHSV;
            rectImg = cv_image_.image(rect_);
            imshow("target", rectImg);
            cvtColor(rectImg, targetImgHSV, COLOR_BGR2HSV);
            calcHist(&targetImgHSV, 2, channels_, Mat(), dstHist, 1, &histSize_, &ranges_, true, false);
            normalize(dstHist, dstHist, 0, 255, NORM_MINMAX);
            return true;
        }
        return false;
    }

    bool ArmController::target_tracking(const sensor_msgs::ImageConstPtr &image_rect, const int color,
                                         double& u, double &v, bool &stop, sensor_msgs::ImagePtr &debug_image)
    {
        using namespace cv;
        if (!fin_)
            current_color_ = color;
        else if (current_color_ != color)
        {
            ROS_WARN("Color changed!");
            fin_ = false;
            current_color_ = color;
            pt_.clear();            
        }
        static Mat dstHist;
        if (!fin_)
        {
            ps_.reset();
            vision_msgs::BoundingBox2DArray objArray;
            if (detect_cargo(image_rect, objArray, debug_image, default_roi_) && target_init(objArray, color, dstHist))
            {
                fin_ = true;
                if (pt_.size())
                    pt_.clear();
                u = rect_.x + rect_.width / 2;
                v = rect_.y + rect_.height / 2;
                ROS_INFO_STREAM("u:" << u << " v:" << v);
                pt_.push_back(Point(u, v));
            }
            else
                return false;
        }
        else
        {
            cv_bridge::CvImagePtr cv_image;
            if (!add_image(image_rect, cv_image))
                return false;
            // 核心代码段
            Mat imageHSV, targetImgHSV;
            Mat calcBackImage;
            cvtColor(cv_image->image, imageHSV, COLOR_BGR2HSV);
            calcBackProject(&imageHSV, 2, channels_, dstHist, calcBackImage, &ranges_); // 反向投影
            TermCriteria criteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 1000, 0.001);
            CamShift(calcBackImage, rect_, criteria); // 关键函数
            Mat imageROI = imageHSV(rect_);           // 更新模板
            targetImgHSV = imageHSV(rect_);
            calcHist(&imageROI, 2, channels_, Mat(), dstHist, 1, &histSize_, &ranges_);
            normalize(dstHist, dstHist, 0.0, 1.0, NORM_MINMAX); // 归一化
            u = rect_.x + rect_.width / 2;
            v = rect_.y + rect_.height / 2;
            ROS_INFO_STREAM("u:" << u << " v:" << v);
            pt_.push_back(Point(u, v));
            if (show_detections_)
            {
                // 目标绘制
                rectangle(cv_image->image, rect_, Scalar(255, 0, 0), 3);
                for (int i = 0; i < pt_.size() - 1; i++)
                {
                    line(cv_image->image, pt_[i], pt_[i + 1], Scalar(0, 255, 0), 2.5);
                }
                debug_image = cv_image->toImageMsg();
                imshow("img", cv_image->image);
                waitKey(1);
                // flag = true;
            }
        }
        return true;
    }
} // namespace my_hand_eye
