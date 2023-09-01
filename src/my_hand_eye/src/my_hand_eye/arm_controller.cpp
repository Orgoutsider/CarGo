#include <XmlRpcException.h>
#include <numeric>
#include <algorithm>
#include <yolov5_ros/cargoSrv.h>

#include "my_hand_eye/arm_controller.h"

namespace my_hand_eye
{
    ArmController::ArmController()
        : stop_(false), can_catch_(true),
          ps_(&sm_st_, &sc_),
          default_roi_(480, 0, 960, 1080),
          border_roi_(320, 0, 1280, 1080),
          ellipse_roi_(320, 540, 1280, 360),
          yaed_(new cv::CEllipseDetectorYaed()),
          threshold(50), catched(false),
          z_parking_area(1.40121),
          z_ellipse(4.58369),
          z_palletize(11.3079),
          z_turntable(13.33052) // 比赛转盘
    //   初始化列表记得复制一份到下面
    {
        cargo_x_.reserve(5);
        cargo_y_.reserve(5);
    };

    ArmController::ArmController(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : stop_(false), can_catch_(true),
          ps_(&sm_st_, &sc_),
          default_roi_(480, 0, 960, 1080),
          border_roi_(320, 0, 1280, 1080),
          ellipse_roi_(320, 540, 1280, 360),
          yaed_(new cv::CEllipseDetectorYaed()),
          threshold(50), catched(false),
          z_parking_area(1.40121),
          z_ellipse(4.58369),
          z_palletize(11.3079),
          z_turntable(13.33052) // 比赛转盘
    //   初始化列表记得复制一份到上面
    //   z_turntable(16.4750)// 老转盘（弃用）
    //   z_turntable(15.57)  // 新转盘（弃用）
    {
        init(nh, pnh);
    }

    ArmController::~ArmController()
    {
        if (!emulation_)
            ps_.end();
        delete yaed_;
    }

    void ArmController::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        emulation_ = pnh.param<bool>("emulation", false);
        if (emulation_)
        {
            plot_client_ = nh.serviceClient<my_hand_eye::Plot>("height_plot");
            return;
        }
        XmlRpc::XmlRpcValue servo_descriptions;
        XmlRpc::XmlRpcValue default_action;
        XmlRpc::XmlRpcValue back_action;
        XmlRpc::XmlRpcValue down_action;
        XmlRpc::XmlRpcValue put1_action;
        XmlRpc::XmlRpcValue put2_action;
        XmlRpc::XmlRpcValue put3_action;
        XmlRpc::XmlRpcValue palletize1_action;
        XmlRpc::XmlRpcValue palletize2_action;
        XmlRpc::XmlRpcValue palletize3_action;
        XmlRpc::XmlRpcValue loop0_enlarge;
        XmlRpc::XmlRpcValue loop1_enlarge;
        if (!pnh.getParam("servo", servo_descriptions))
        {
            ROS_ERROR("No speed and acc specified");
        }
        if (!pnh.getParam("default_action", default_action))
        {
            ROS_ERROR("No default action specified");
        }
        if (!pnh.getParam("back_action", back_action))
        {
            ROS_ERROR("No back action specified");
        }
        if (!pnh.getParam("down_action", down_action))
        {
            ROS_ERROR("No down action specified");
        }
        if (!pnh.getParam("put1_action", put1_action))
        {
            ROS_ERROR("No put1 action specified");
        }
        if (!pnh.getParam("put2_action", put2_action))
        {
            ROS_ERROR("No put2 action specified");
        }
        if (!pnh.getParam("put3_action", put3_action))
        {
            ROS_ERROR("No put3 action specified");
        }
        if (!pnh.getParam("palletize1_action", palletize1_action))
        {
            ROS_ERROR("No palletize1 action specified");
        }
        if (!pnh.getParam("palletize2_action", palletize2_action))
        {
            ROS_ERROR("No palletize2 action specified");
        }
        if (!pnh.getParam("palletize3_action", palletize3_action))
        {
            ROS_ERROR("No palletize3 action specified");
        }
        if (!pnh.getParam("loop0_enlarge", loop0_enlarge))
        {
            ROS_ERROR("No loop0 enlarge specified");
        }
        if (!pnh.getParam("loop1_enlarge", loop1_enlarge))
        {
            ROS_ERROR("No loop1 enlarge specified");
        }
        std::string ft_servo;
        pnh.param<std::string>("ft_servo", ft_servo, "/dev/ft_servo");
        ROS_INFO_STREAM("serial:" << ft_servo);
        white_vmin_ = pnh.param<int>("white_vmin", 170);
        speed_standard_static_ = pnh.param<double>("speed_standard_static", 0.16);
        speed_standard_motion_ = pnh.param<double>("speed_standard_motion", 0.12);
        tracker_.flag = pnh.param<bool>("flag", false);
        if (!ps_.begin(ft_servo.c_str()))
        {
            ROS_ERROR_STREAM("Cannot open ft servo at" << ft_servo);
        }
        ps_.ping();
        ps_.set_speed_and_acc(servo_descriptions);
        ps_.set_action(default_action);
        ps_.set_action(back_action, "back");
        ps_.set_action(down_action, "down");
        ps_.set_action(put1_action, "put1");
        ps_.set_action(put2_action, "put2");
        ps_.set_action(put3_action, "put3");
        ps_.set_action(palletize1_action, "palletize1");
        ps_.set_action(palletize2_action, "palletize2");
        ps_.set_action(palletize3_action, "palletize3");
        ps_.set_action(loop0_enlarge, "loop0");
        ps_.set_action(loop1_enlarge, "loop1");
        ps_.show_voltage();

        cargo_client_ = nh.serviceClient<yolov5_ros::cargoSrv>("cargoSrv");
        cargo_x_.reserve(3);
        cargo_y_.reserve(3);
        nh_ = &nh;
        // Parameters Settings (Sect. 4.2)
        int iThLength = 16;
        float fThObb = 3.0f;
        float fThPos = 1.0f;
        float fTaoCenters = 0.05f;
        int iNs = 16;
        float fMaxCenterDistance = sqrt(float(ellipse_roi_.width * ellipse_roi_.width + ellipse_roi_.height * ellipse_roi_.height)) * fTaoCenters;

        fThScoreScore_ = pnh.param<float>("fThScoreScore", 0.45f);

        // Other constant parameters settings.

        // Gaussian filter parameters, in pre-processing
        cv::Size szPreProcessingGaussKernelSize = cv::Size(5, 5);
        double dPreProcessingGaussSigma = 1.0;

        float fDistanceToEllipseContour = 0.1f; // (Sect. 3.3.1 - Validation)
        // Const parameters to discard bad ellipses
        fMinReliability_ = pnh.param<float>("fMinReliability", 0.45f);
        yaed_->SetParameters(szPreProcessingGaussKernelSize,
                             dPreProcessingGaussSigma,
                             fThPos,
                             fMaxCenterDistance,
                             iThLength,
                             fThObb,
                             fDistanceToEllipseContour,
                             fThScoreScore_,
                             fMinReliability_,
                             iNs);
    }

    bool ArmController::add_image(const sensor_msgs::ImageConstPtr &image_rect, cv_bridge::CvImagePtr &image)
    {
        try
        {
            image = cv_bridge::toCvCopy(image_rect, "bgr8");
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
        cv_image_.encoding = image->encoding;
        cv_image_.header = image->header;
        cv_image_.image = image->image.clone();
        return true;
    }

    bool ArmController::take_picture()
    {
        if (cv_image_.image.empty())
        {
            ROS_WARN("Empty image!");
            return false;
        }
        char str[50];
        static int n = 0;
        // 替换为保存图片的路径
        snprintf(str, sizeof(str), "/home/fu/apriltag_ws/src/my_hand_eye/img/%d.jpg", ++n);
        cv::imwrite(str, cv_image_.image);
        return true;
    }

    bool ArmController::detect_cargo(const sensor_msgs::ImageConstPtr &image_rect, vision_msgs::BoundingBox2DArray &detections,
                                     sensor_msgs::ImagePtr &debug_image, cv::Rect &rect)
    {
        if (cargo_client_.exists())
        {
            ROS_INFO_ONCE("Service is up and available.");
            cv_bridge::CvImagePtr cv_image;
            bool flag = add_image(image_rect, cv_image);
            cv_image->image = cv_image->image(rect).clone();
            yolov5_ros::cargoSrv cargo;
            sensor_msgs::ImagePtr image = cv_image->toImageMsg();
            cargo.request.image = *image;
            // 发送请求,返回 bool 值，标记是否成功
            if (flag)
                flag = cargo_client_.call(cargo);
            if (flag)
            {
                detections = cargo.response.results;
                if (show_detections && !cv_image->image.empty())
                {
                    if (cargo.response.results.boxes.size())
                    {
                        for (int color = color_red; color <= color_blue; color++)
                        {
                            if (!cargo.response.results.boxes[color].center.x)
                                continue;
                            cv::RotatedRect rect(cv::Point2d(cargo.response.results.boxes[color].center.x, cargo.response.results.boxes[color].center.y),
                                                 cv::Size2d(cargo.response.results.boxes[color].size_x, cargo.response.results.boxes[color].size_y),
                                                 cargo.response.results.boxes[color].center.theta);
                            cv::Point2f vtx[4]; // 矩形顶点容器
                            // cv::Mat dst = cv::Mat::zeros(cv_image->image.size(), CV_8UC3);//创建空白图像
                            rect.points(vtx); // 确定旋转矩阵的四个顶点
                            cv::Scalar colors;
                            switch (color)
                            {
                            case color_red:
                                colors = cv::Scalar(0, 0, 255);
                                break;
                            case color_green:
                                colors = cv::Scalar(0, 255, 0);
                                break;
                            case color_blue:
                                colors = cv::Scalar(255, 0, 0);
                                break;
                            default:
                                break;
                            }
                            for (int j = 0; j < 4; j++)
                            {
                                cv::line(cv_image->image, vtx[j], vtx[(j + 1) % 4], colors, 2);
                            }
                        }
                    }
                    // imshow("det", cv_image->image);
                    // cv::waitKey(10);
                }
                if (detections.boxes.size())
                {
                    for (int color = 1; color <= 3; color++)
                    {
                        if (!detections.boxes[color].center.x)
                            continue;
                        detections.boxes[color].center.x += rect.x;
                        detections.boxes[color].center.y += rect.y;
                    }
                }
            }
            else
                ROS_WARN("Responce invalid or failed to add image!");
            if (show_detections && !cv_image->image.empty())
                debug_image = cv_image->toImageMsg();
            return flag;
        }
        else
        {
            ROS_WARN_ONCE("Service is not up and available. Please wait.");
            return false;
        }
    }

    bool ArmController::log_cargo(const sensor_msgs::ImageConstPtr &image_rect, Color color, double z,
                                  sensor_msgs::ImagePtr &debug_image, bool center, bool pose)
    {
        if (pose && center)
        {
            ROS_WARN("log_cargo: Cannot set both of center and pose to true");
        }
        static bool flag = true; // 尚未初始化位姿
        static bool rst = true;
        if (flag)
        {
            flag = false;
            ps_.reset(pose || (z == z_ellipse));
            return false;
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        vision_msgs::BoundingBox2DArray objArray;
        bool valid = detect_cargo(image_rect, objArray, debug_image, default_roi_);
        if (valid)
        {
            double x = 0, y = 0;
            if (center)
            {
                double center_u = 0, center_v = 0;
                if (valid = get_center(objArray, center_u, center_v, x, y))
                {
                    ROS_INFO_STREAM("x:" << x << " y:" << y);
                    if (show_detections && !cv_image_.image.empty())
                    {
                        draw_cross(cv_image_.image, cv::Point(center_u, center_v), cv::Scalar(255, 255, 255), 30, 3);
                        debug_image = cv_image_.toImageMsg();
                    }
                }
            }
            else if (pose)
            {
                geometry_msgs::Pose2D p;
                if (valid = get_pose(objArray, z_ellipse, p, rst))
                {
                    ROS_INFO_STREAM("x:" << p.x << " y:" << p.y << " theta:" << Angle::degree(p.theta));
                    if (rst)
                        rst = false;
                }
            }
            else if (valid = find_with_color(objArray, color, z, x, y))
                ROS_INFO_STREAM("x:" << x << " y:" << y);
        }
        return valid;
    }

    bool ArmController::log_extrinsics_correction(const sensor_msgs::ImageConstPtr &image_rect,
                                                  double correct_x, double correct_y, double correct_z, Color color,
                                                  sensor_msgs::ImagePtr &debug_image)
    {
        static bool flag = true;
        // 尚未初始化位姿
        if (flag)
        {
            flag = false;
            ps_.reset();
            return false;
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        vision_msgs::BoundingBox2DArray objArray;
        bool valid = detect_cargo(image_rect, objArray, debug_image, default_roi_);
        if (valid)
        {
            double u = 0, v = 0;
            if (objArray.boxes.size() == 4)
            {
                if (!objArray.boxes[color].center.x)
                    return false;
                u = objArray.boxes[color].center.x;
                v = objArray.boxes[color].center.y;
            }
            else
                return false;
            valid = ps_.extrinsics_correction(u, v, correct_x, correct_y, correct_z);
        }
        return valid;
    }

    bool ArmController::find_with_color(vision_msgs::BoundingBox2DArray &objArray, const Color color, double z, double &x, double &y)
    {
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

    bool ArmController::calculate_radius_and_speed(double &u, double &v, double &x, double &y, double &radius, double &speed)
    {
        tracker_.get_center(u, v);
        bool valid = ps_.calculate_cargo_position(u, v, z_turntable, x, y, false);
        if (valid)
        {
            // ROS_INFO_STREAM("(" << x << ", " << y << ")");
            valid = tracker_.calculate_radius_and_speed(x, y, radius, speed_standard_static_, speed);
        }
        return valid;
    }

    bool ArmController::get_center(vision_msgs::BoundingBox2DArray &objArray, double &center_u, double &center_v,
                                   double &center_x, double &center_y, bool read)
    {
        if (objArray.boxes.size() == 4)
        {
            double u_sum = 0, v_sum = 0;
            for (int color = color_red; color <= color_blue; color++)
            {
                if (!objArray.boxes[color].center.x)
                {
                    ROS_WARN("get_center: Color %d cargo does not exist.", color);
                    return false;
                }
                u_sum += objArray.boxes[color].center.x;
                v_sum += objArray.boxes[color].center.y;
            }
            center_u = u_sum / 3;
            center_v = v_sum / 3;
            return ps_.calculate_cargo_position(center_u, center_v, z_turntable, center_x, center_y, read);
        }
        else
            return false;
    }

    bool ArmController::get_pose(vision_msgs::BoundingBox2DArray &objArray,
                                 double z, geometry_msgs::Pose2D &pose, bool rst)
    {
        if (objArray.boxes.size() == 4)
        {
            static geometry_msgs::Pose2D last_pose;
            if (rst)
            {
                if (!set_color_order(objArray))
                    return false;
            }
            int cnt = 0;
            double x[5] = {0}, y[5] = {0};
            bool flag = false;
            bool read = rst;
            for (int i = 1; i <= 3; i++)
            {
                if (!objArray.boxes[color_order_[i].color].center.x)
                {
                    ROS_WARN("get_pose: Color %d ellipse does not exist.",
                             color_order_[i].color);
                    continue;
                }
                if (!ps_.calculate_cargo_position(objArray.boxes[color_order_[i].color].center.x,
                                                  objArray.boxes[color_order_[i].color].center.y,
                                                  z, x[cnt], y[cnt], read))
                {
                    ROS_WARN("get_pose: Cannot calculate color %d ellipse's position",
                             color_order_[i].color);
                    continue;
                }
                if (read)
                    read = false;
                if (i == 2)
                {
                    pose.x = x[cnt];
                    pose.y = y[cnt];
                    flag = true;
                }
                cnt++;
            }
            if (!flag)
                return false;
            // 防止超出车道
            if (pose.x < -35.5)
                pose.x = -35.5;
            else if (pose.x > -27.5)
                pose.x = -27.5;
            if (pose.y < -ARM_P - 8)
                pose.y = -ARM_P - 8;
            else if (pose.y > -ARM_P + 8)
                pose.y = -ARM_P + 8;
            // 对象相对车体的偏角
            if (cnt == 3)
                pose.theta = -(atan((x[0] - x[1]) / (y[0] - y[1])) +
                               atan((x[1] - x[2]) / (y[1] - y[2])) +
                               atan((x[2] - x[0]) / (y[2] - y[0]))) /
                             3;
            else if (cnt == 2)
                pose.theta = -atan((x[0] - x[1]) / (y[0] - y[1]));
            else if (!rst)
                pose.theta = last_pose.theta;
            else
                return false;
            if (!rst)
            {
                if (abs(last_pose.x - pose.x) > 3)
                    pose.x = last_pose.x + cv::sgn(pose.x - last_pose.x);
                if (abs(last_pose.y - pose.y) > 3)
                    pose.y = last_pose.y + cv::sgn(pose.y - last_pose.y);
                if (Angle::degree(last_pose.theta - pose.theta) > 10)
                    pose.theta = last_pose.theta + Angle(3).rad() *
                                                       cv::sgn(pose.theta - last_pose.theta);
            }
            last_pose = pose;
            return true;
        }
        return false;
    }

    bool ArmController::set_color_order(vision_msgs::BoundingBox2DArray &objArray)
    {
        if (objArray.boxes.size() == 4)
        {
            for (int color = color_red; color <= color_blue; color++)
            {
                if (!objArray.boxes[color].center.x)
                {
                    ROS_WARN("set_color_order: Color %d ellipse does not exist.", color);
                    return false;
                }
                color_order_[color].center_x = objArray.boxes[color].center.x;
                color_order_[color].color = color;
            }
            std::sort(color_order_ + 1, color_order_ + 4, [](EllipseColor e1, EllipseColor e2)
                      { return e1.center_x < e2.center_x; });
            ROS_INFO("Color order: %d%d%d", color_order_[1].color,
                     color_order_[2].color, color_order_[3].color);
            for (int i = color_red; i <= color_blue; i++)
                switch (color_order_[i].color)
                {
                case color_red:
                    color_map_[color_red] = i;
                    break;

                case color_green:
                    color_map_[color_green] = i;
                    break;

                case color_blue:
                    color_map_[color_blue] = i;
                    break;

                default:
                    break;
                }
            return true;
        }
        return false;
    }

    void ArmController::average_position(double &x, double &y)
    {
        if (cargo_x_.empty() || cargo_y_.empty())
        {
            x = y = 0;
            return;
        }
        x = std::accumulate(std::begin(cargo_x_), std::end(cargo_x_), 0.0) / cargo_x_.size();
        y = std::accumulate(std::begin(cargo_y_), std::end(cargo_y_), 0.0) / cargo_y_.size();
    }

    void ArmController::average_theta(double &theta)
    {
        if (cargo_theta_.empty())
        {
            theta = 0;
            return;
        }
        theta = std::accumulate(std::begin(cargo_theta_), std::end(cargo_theta_), 0.0) /
                cargo_theta_.size();
    }

    void ArmController::average_pose(geometry_msgs::Pose2D &pose)
    {
        average_position(pose.x, pose.y);
        average_theta(pose.theta);
    }

    void ArmController::average_pose_once()
    {
        if (cargo_x_.size() <= 2 || cargo_y_.size() <= 2 || cargo_theta_.size() <= 2)
        {
            return;
        }
        // 去除一个最大值，一个最小值
        cargo_x_.erase(std::min_element(cargo_x_.begin(), cargo_x_.end()));
        cargo_x_.erase(std::max_element(cargo_x_.begin(), cargo_x_.end()));
        cargo_y_.erase(std::min_element(cargo_y_.begin(), cargo_y_.end()));
        cargo_y_.erase(std::max_element(cargo_y_.begin(), cargo_y_.end()));
        cargo_theta_.erase(std::min_element(cargo_theta_.begin(), cargo_theta_.end()));
        cargo_theta_.erase(std::max_element(cargo_theta_.begin(), cargo_theta_.end()));
        geometry_msgs::Pose2D pose;
        average_pose(pose);
        cargo_x_.clear();
        cargo_y_.clear();
        cargo_theta_.clear();
        cargo_x_.push_back(pose.x);
        cargo_y_.push_back(pose.y);
        cargo_theta_.push_back(pose.theta);
    }

    bool ArmController::catch_straightly(const sensor_msgs::ImageConstPtr &image_rect, const Color color,
                                         bool &finish, sensor_msgs::ImagePtr &debug_image,
                                         bool left, bool hold, bool midpoint)
    {
        const int MAX_SIZE = 5;
        static bool flag = true;
        if (flag)
        {
            current_color_ = color;
            flag = false;
            ps_.reset();
            return false;
        }
        else if (current_color_ != color || cargo_x_.size() >= MAX_SIZE)
        {
            cargo_x_.clear();
            cargo_y_.clear();
            current_color_ = color;
            ps_.reset();
            return false;
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        finish = false;
        vision_msgs::BoundingBox2DArray objArray;
        bool valid = detect_cargo(image_rect, objArray, debug_image, default_roi_);
        if (valid)
        {
            double x = 0, y = 0;
            if (find_with_color(objArray, current_color_, z_turntable, x, y))
            {
                ROS_INFO_STREAM("x:" << x << " y:" << y);
                cargo_x_.push_back(x);
                cargo_y_.push_back(y);
                if (cargo_x_.size() == MAX_SIZE)
                {
                    double x_aver = 0, y_aver = 0;
                    average_position(x_aver, y_aver);
                    cargo_x_.clear();
                    cargo_y_.clear();
                    if (midpoint)
                        valid = ps_.go_to_by_midpoint(x_aver, y_aver, z_turntable);
                    else
                        valid = ps_.go_to_and_wait(x_aver, y_aver, z_turntable, true);
                    if (!hold)
                    {
                        if (valid)
                        {
                            ps_.go_to_table(false, color, left);
                            ps_.reset(left);
                        }
                    }
                    finish = true;
                }
            }
            else
                return false;
        }
        return valid;
    }

    bool ArmController::remember(double &x, double &y, double &z, double &tightness)
    {
        if (ps_.refresh_xyz())
        {
            ARM_INFO_XYZ(ps_);
            ROS_INFO_STREAM("tightness: " << ps_.tightness);
            x = ps_.x;
            y = ps_.y;
            z = ps_.z;
            tightness = ps_.tightness;
        }
        else
            return false;
        return true;
    }

    bool ArmController::cargo_is_static(double speed, bool reset, double x, double y)
    {
        static int cnt_motion = 0;
        static bool motion_before = false;
        if (reset)
        {
            if (!cargo_x_.empty() || !cargo_y_.empty())
            {
                cargo_x_.clear();
                cargo_y_.clear();
            }
            cnt_motion = 0;
            motion_before = false;
            return false;
        }
        if (speed < 0) // -1
        {
            if (!cargo_x_.empty() || !cargo_y_.empty())
            {
                cargo_x_.clear();
                cargo_y_.clear();
            }
            cnt_motion = 0;
            motion_before = false;
            return false;
        }
        else
        {
            if (speed < speed_standard_static_)
            {
                cargo_x_.push_back(x);
                cargo_y_.push_back(y);
                if ((cargo_x_.size() >= 3) &&
                    (motion_before || catched)) // 放弃从开始就停止的块
                {
                    cnt_motion = 0;
                    motion_before = false;
                    if (!catched)
                        catched = true;
                    return true;
                }
            }
            else if (!cargo_x_.empty() || !cargo_y_.empty())
            {
                cargo_x_.clear();
                cargo_y_.clear();
            }
            if (speed > speed_standard_motion_ && !motion_before)
            {
                cnt_motion++;
                if (cnt_motion >= 3)
                {
                    ROS_INFO("Can catch after now.");
                    motion_before = true;
                }
            }
            else if (!motion_before && cnt_motion)
            {
                cnt_motion = 0;
            }
        }
        return false;
    }

    bool ArmController::track(const sensor_msgs::ImageConstPtr &image_rect, const Color color, bool &first,
                              double &x, double &y, sensor_msgs::ImagePtr &debug_image)
    {
        using namespace cv;
        const int TRACKING = 0, DETECTING = 1; // 1:重新进行目标检测
        static int state = DETECTING;
        const double PERMIT = 2.2;
        static double center_u = 0, center_v = 0, first_radius = 0;
        static std::vector<cv::Point> pt;
        double radius = 0, speed = -1, u = 0, v = 0, nx = 0, ny = 0;
        bool rst = false; // 用于重启静止检测
        cv_bridge::CvImagePtr cv_image;
        if (can_catch_ && !stop_) // 抓后重启或第一次
        {
            ps_.reset();
            current_color_ = color;
            can_catch_ = false;
            state = DETECTING;
            return false;
        }
        else if (!can_catch_ && stop_) // 停留不可抓
        {
            stop_ = false;
            state = TRACKING;
        }
        else if (can_catch_ && stop_) // 不正常状态
        {
            ROS_ERROR("track: Invalid status!");
            stop_ = false;
            can_catch_ = false;
            ps_.reset();
            state = DETECTING;
            current_color_ = color;
            return false;
        }
        else if (current_color_ != color)
        {
            ROS_WARN("Color has changed!");
            stop_ = false;
            can_catch_ = false;
            ps_.reset();
            state = DETECTING;
            current_color_ = color;
            return false;
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        if (state == DETECTING)
        {
            vision_msgs::BoundingBox2DArray objArray;
            static double center_x = 0, center_y = 0;
            if (detect_cargo(image_rect, objArray, debug_image, default_roi_))
            {
                if (first)
                {
                    if (!get_center(objArray, center_u, center_v, center_x, center_y, true) ||
                        !tracker_.target_init(*nh_, cv_image_, objArray, color, white_vmin_,
                                              center_x, center_y, show_detections) ||
                        !tracker_.order_init(objArray, ps_, z_turntable))
                    {
                        return false;
                    }
                    else
                    {
                        first = false;
                    }
                }
                else if (!tracker_.target_init(*nh_, cv_image_, objArray, color, white_vmin_,
                                               center_x, center_y, show_detections))
                    return false;
                calculate_radius_and_speed(u, v, nx, ny, radius, speed);
                first_radius = radius;
                state = TRACKING;
                rst = true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            if (!add_image(image_rect, cv_image))
                return false;
            if (!tracker_.target_track(*cv_image, ps_, z_turntable))
            {
                state = DETECTING;
                return false;
            }
            if (!calculate_radius_and_speed(u, v, nx, ny, radius, speed))
                return false;
            if (abs(radius - first_radius) > PERMIT)
                state = DETECTING;
            if (show_detections && !cv_image->image.empty())
            {
                // 目标绘制
                draw_cross(cv_image->image, cv::Point(u, v), Scalar(255, 255, 0), 30, 3);
                draw_cross(cv_image->image, cv::Point(center_u, center_v), Scalar(255, 255, 255), 30, 3);
                if (pt.size())
                {
                    for (int i = 0; i < pt.size() - 1; i++)
                    {
                        line(cv_image->image, pt.at(i), pt.at(i + 1), Scalar(0, 255, 0), 2.5);
                    }
                }
                debug_image = cv_image->toImageMsg();
                // imshow("img", cv_image->image);
                // waitKey(20);
            }
        }
        if (show_detections)
            pt.push_back(cv::Point(u, v));
        // ROS_INFO_STREAM("speed:" << speed << " radius:" << abs(radius - first_radius));
        if (cargo_is_static(speed, rst, nx, ny))
        {
            if (tracker_.no_obstacles())
            {
                stop_ = true;
                average_position(x, y);
                ROS_INFO("There is no obstacles");
            }
            else
                ROS_INFO("There are some obstacles.");
        }
        return true;
    }

    bool ArmController::catch_after_tracking(double x, double y, const Color color, const Color color_next,
                                             bool left, bool &finish)
    {
        if (!stop_)
            return false;
        else
        {
            ROS_INFO_STREAM("x:" << x << " y:" << y);
            bool valid = ps_.go_to_and_wait(x, y, z_turntable, true);
            if (valid)
            {
                ps_.go_to_table(false, color, left);
            }
            else
            {
                finish = false;
                return false;
            }
            can_catch_ = true;
            stop_ = false;
            finish = true;
            // 此后不再判断对应的颜色是否有障碍物
            if (tracker_.left_color == color_next)
            {
                tracker_.left_color = tracker_.right_color;
                tracker_.right_color = 0;
            }
            else if (tracker_.right_color == color_next)
            {
                tracker_.right_color = tracker_.left_color;
                tracker_.left_color = 0;
            }
            return true;
        }
    }

    bool ArmController::find_points_with_height(double h, bool done)
    {
        if (!plot_client_.exists())
        {
            plot_client_.waitForExistence();
        }
        my_hand_eye::Plot plt;
        bool valid = ps_.find_points_with_height(h, plt.request.arr);
        if (valid)
        {
            plt.request.done = done;
            valid = plot_client_.call<my_hand_eye::Plot>(plt);
            if (!valid)
                ROS_ERROR("Failed to call service");
        }
        return valid;
    }

    bool ArmController::detect_ellipse(const sensor_msgs::ImageConstPtr &image_rect, vision_msgs::BoundingBox2DArray &detections,
                                       sensor_msgs::ImagePtr &debug_image, cv::Rect &rect)
    {
        cv_bridge::CvImagePtr cv_image;
        if (!add_image(image_rect, cv_image))
            return false;
        cv_image->image = cv_image->image(rect);
        using namespace cv;
        std::vector<cv::Ellipse> ells;
        resize(cv_image->image, cv_image->image, Size(cv_image->image.cols / 2, cv_image->image.rows / 2)); // 重设大小，可选
        // 第一次预处理
        Mat sat = saturation(cv_image->image, 100);
        Mat hsv;
        cvtColor(sat, hsv, COLOR_BGR2HSV_FULL);
        std::vector<Mat1b> hsvs;
        split(hsv, hsvs);
        // imshow("h", hsvs[1]);
        // waitKey(10);
        std::vector<cv::Ellipse> ellsYaed;
        yaed_->Detect(hsvs[1], ellsYaed);
        EllipseArray arr;
        // 聚类、颜色标定
        if (!arr.clustering(ellsYaed, cv_image) ||
            !arr.color_classification(cv_image, white_vmin_))
            return false;
        detections.header = image_rect->header;
        if (!arr.detection(detections, rect, cv_image))
            return false;
        if (!cv_image_.image.empty())
        {
            for (int color = color_red; color <= color_blue; color++)
            {
                if (!detections.boxes[color].center.x)
                {
                    continue;
                }
                Rect rect(detections.boxes[color].center.x - detections.boxes[color].size_x / 2,
                          detections.boxes[color].center.y - detections.boxes[color].size_y / 2,
                          detections.boxes[color].size_x, detections.boxes[color].size_y);
                rect.x = std::max(rect.x, 0);
                rect.y = std::max(rect.y, 0);
                rect.width = std::min(rect.width, cv_image_.image.cols - rect.x);
                rect.height = std::min(rect.height, cv_image_.image.rows - rect.y);
                if (!rect.empty())
                {
                    Mat grey = cv_image_.image(rect).clone();
                    // imshow("src", grey);
                    cvtColor(grey, grey, CV_BGR2GRAY);
                    // imshow("grey", grey);
                    Mat thr;
                    cv::threshold(grey, thr, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
                    bitwise_not(thr, thr);
                    // imshow("threshold", thr);
                    if (countNonZero(thr))
                    {
                        Moments m = moments(thr, true);
                        detections.boxes[color].center.x = m.m10 / m.m00 + rect.x;
                        detections.boxes[color].center.y = m.m01 / m.m00 + rect.y;
                    }
                }
                if (show_detections)
                {
                    Scalar c;
                    switch (color)
                    {
                    case color_red:
                        c = Scalar(0, 0, 255);
                        break;

                    case color_green:
                        c = Scalar(0, 255, 0);
                        break;

                    case color_blue:
                        c = Scalar(255, 0, 0);
                        break;

                    default:
                        break;
                    }
                    rectangle(cv_image_.image, rect, Scalar(0, 0, 0), 2, 4);
                    draw_cross(cv_image_.image,
                               Point2d(detections.boxes[color].center.x, detections.boxes[color].center.y),
                               c, 30, 2);
                }
            }
            if (show_detections)
                debug_image = cv_image_.toImageMsg();
        }
        // if (show_detections && !cv_image->image.empty())
        // {
        //     Mat3b srcCopy = cv_image->image;
        //     yaed_->DrawDetectedEllipses(srcCopy, ellsYaed, 0, 1);
        //     // imshow("Yaed", srcCopy);
        //     // waitKey(10);
        //     debug_image = cv_image->toImageMsg();
        // }
        // std::vector<double> times = yaed_->GetTimes();
        // std::cout << "--------------------------------" << std::endl;
        // std::cout << "Execution Time: " << std::endl;
        // std::cout << "Edge Detection: \t" << times[0] << std::endl;
        // std::cout << "Pre processing: \t" << times[1] << std::endl;
        // std::cout << "Grouping:       \t" << times[2] << std::endl;
        // std::cout << "Estimation:     \t" << times[3] << std::endl;
        // std::cout << "Validation:     \t" << times[4] << std::endl;
        // std::cout << "Clustering:     \t" << times[5] << std::endl;
        // std::cout << "--------------------------------" << std::endl;
        // std::cout << "Total:	         \t" << yaed_->GetExecTime() << std::endl;
        // std::cout << "--------------------------------" << std::endl;
        return true;
    }

    bool ArmController::log_ellipse(const sensor_msgs::ImageConstPtr &image_rect, const Color color,
                                    sensor_msgs::ImagePtr &debug_image, bool pose)
    {
        static bool flag = true;
        static bool rst = true;
        if (flag)
        {
            flag = false;
            current_color_ = color;
            ps_.reset(true);
            return false;
        }
        else if (current_color_ != color)
        {
            ps_.reset(true);
            current_color_ = color;
            return false;
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        vision_msgs::BoundingBox2DArray objArray;
        bool valid = detect_ellipse(image_rect, objArray, debug_image, ellipse_roi_);
        if (valid)
        {
            if (pose)
            {
                geometry_msgs::Pose2D p;
                if (valid = get_pose(objArray, z_parking_area, p, rst))
                {
                    ROS_INFO_STREAM("x:" << p.x << " y:" << p.y << " theta:" << Angle::degree(p.theta));
                    if (rst)
                        rst = false;
                }
            }
            else
            {
                double x = 0, y = 0;
                if (valid = find_with_color(objArray, color, z_parking_area, x, y))
                {
                    ROS_INFO_STREAM("x:" << x << " y:" << y);
                }
            }
        }
        return valid;
    }

    // bool ArmController::log_z_correction(const sensor_msgs::ImageConstPtr &image_rect,
    //                                      std::vector<double> &&e1, std::vector<double> &&e2, std::vector<double> &&e3,
    //                                      sensor_msgs::ImagePtr &debug_image)
    // {
    //     double z_p_before = z_parking_area;
    //     if (!cargo_x_.empty())
    //         cargo_x_.clear();
    //     vision_msgs::BoundingBox2DArray objArray;
    //     bool valid = detect_ellipse(image_rect, objArray, debug_image, ellipse_roi_) &&
    //                  set_color_order(objArray);
    //     my_hand_eye::Pose2DMightEnd pme;
    //     if (valid)
    //         valid = get_pose(objArray, z_parking_area, pme, false);
    //     if (valid && pme.pose.theta != pme.not_change)
    //     {
    //         my_hand_eye::Pose2DMightEnd err;
    //         target_pose.calc(pme.pose, err);
    //         Action *ap = ps_.get_action_put();
    //         Action ap_target[4];
    //         ap_target[1] = ap[1].now2goal(err.pose, Action(enlarge_x_, enlarge_y_, 0));
    //         ap_target[2] = ap[2].now2goal(err.pose, Action(enlarge_x_, enlarge_y_, 0));
    //         ap_target[3] = ap[3].now2goal(err.pose, Action(enlarge_x_, enlarge_y_, 0));
    //         ap_target[1] += Action(e1[0], e1[1], 0);
    //         ap_target[2] += Action(e2[0], e2[1], 0);
    //         ap_target[3] += Action(e3[0], e3[1], 0);
    //         double dist_min = Action::normxy(Action(e1[0], e1[1], 0), Action()) +
    //                           Action::normxy(Action(e2[0], e2[1], 0), Action()) +
    //                           Action::normxy(Action(e3[0], e3[1], 0), Action());
    //         double z_min = z_parking_area;
    //         ROS_INFO_STREAM("z:" << z_parking_area << " dist:" << dist_min);
    //         for (z_parking_area = z_p_before + 5; z_parking_area > z_p_before - 5; z_parking_area -= 0.1)
    //         {
    //             if (get_pose(objArray, z_parking_area, pme, false))
    //             {
    //                 target_pose.calc(pme.pose, err);
    //                 Action ap_n[4];
    //                 ap_n[1] = ap[1].now2goal(err.pose, Action(enlarge_x_, enlarge_y_, 0));
    //                 ap_n[2] = ap[2].now2goal(err.pose, Action(enlarge_x_, enlarge_y_, 0));
    //                 ap_n[3] = ap[3].now2goal(err.pose, Action(enlarge_x_, enlarge_y_, 0));
    //                 double dist = Action::normxy(ap_n[1], ap_target[1]) +
    //                               Action::normxy(ap_n[2], ap_target[2]) +
    //                               Action::normxy(ap_n[3], ap_target[3]);
    //                 z_min = (dist < dist_min) ? z_parking_area : z_min;
    //                 dist_min = MIN(dist, dist_min);
    //                 ROS_INFO_STREAM("z:" << z_parking_area << " dist:" << dist);
    //             }
    //         }
    //         ROS_INFO_STREAM("z_min:" << z_min << " dist_min:" << dist_min);
    //     }
    //     z_parking_area = z_p_before;
    //     return valid;
    // }

    bool ArmController::put(const Color color, bool pal, bool final)
    {
        geometry_msgs::Pose2D err;
        average_pose(err);
        if (final)
        {
            cargo_x_.clear();
            cargo_y_.clear();
            cargo_theta_.clear();
        }
        bool valid = ps_.go_to_table(true, color, true) &&
                     ps_.put(color_map_[color], false, err, pal);
        ps_.reset(true);
        return valid;
    }

    bool ArmController::catch_after_putting(const Color color, bool final)
    {
        geometry_msgs::Pose2D err;
        average_pose(err);
        if (final)
        {
            cargo_x_.clear();
            cargo_y_.clear();
            cargo_theta_.clear();
        }
        bool valid = ps_.put(color_map_[color], true, err, false) &&
                     ps_.go_to_table(false, color, true);
        ps_.reset(true);
        return valid;
    }

    bool ArmController::log_border(const sensor_msgs::ImageConstPtr &image_rect,
                                   sensor_msgs::ImagePtr &debug_image)
    {
        static bool flag = true;
        double distance = 0, yaw = 0;
        if (flag)
        {
            // ps_.reset(true);
            ps_.look_down();
            flag = false;
            return false;
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        cv_bridge::CvImagePtr cv_image;
        if (!add_image(image_rect, cv_image))
            return false;
        cv_image->image = cv_image->image(border_roi_).clone();
        cv::Vec2f border;
        bool valid = border_.detect(cv_image, border, border_roi_,
                                    boost::bind(&ArmController::LBD_color_func, this, _1, _2, threshold),
                                    show_detections, debug_image);
        if (valid)
            valid = ps_.calculate_border_position(border, z_parking_area, distance, yaw);
        if (valid)
        {
            ROS_INFO_STREAM("distance: " << distance << " yaw: " << Angle::degree(yaw));
        }
        return valid;
    }

    void ArmController::ready_to_catch()
    {
        ps_.reset();
    }

    bool ArmController::find_cargo(const sensor_msgs::ImageConstPtr &image_rect, Pose2DMightEnd &msg,
                                   sensor_msgs::ImagePtr &debug_image, bool pose, bool store)
    {
        if (store && !pose)
        {
            ROS_WARN("find_cargo: Both of store and pose must be set to true.");
            return false;
        }
        static bool last_finish = true;
        static bool rst = false;
        const int MAX = 5; // 读取5次求平均位姿
        if (!msg.end && last_finish && !store)
        {
            ps_.reset(pose);
            last_finish = false;
            if (pose)
            {
                Action ellipse = Action(0, 20, 0).front2left().arm2footprint();
                target_pose.pose[target_pose.target_ellipse].x = ellipse.x;
                target_pose.pose[target_pose.target_ellipse].y = ellipse.y;
                cargo_theta_.clear();
                rst = true;
            }
            return false;
        }
        else if (msg.end && last_finish && store)
        {
            cargo_x_.clear();
            cargo_y_.clear();
            cargo_theta_.clear();
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        vision_msgs::BoundingBox2DArray objArray;
        geometry_msgs::Pose2D p;
        double center_u, center_v;
        bool valid = detect_cargo(image_rect, objArray, debug_image, default_roi_);
        if (valid)
        {
            if (pose)
            {
                valid = get_pose(objArray, z_ellipse, p, (rst && cargo_theta_.empty()));
                if (rst)
                {
                    if (valid)
                    {
                        cargo_theta_.push_back(p.theta);
                        ROS_INFO_STREAM("theta: " << p.theta);
                        if (cargo_theta_.size() == MAX)
                        {
                            cargo_theta_.erase(std::min_element(cargo_theta_.begin(), cargo_theta_.end()));
                            cargo_theta_.erase(std::max_element(cargo_theta_.begin(), cargo_theta_.end()));
                            average_theta(target_pose.pose[target_pose.target_ellipse].theta);
                            ROS_INFO_STREAM("Set theta to " << target_pose.pose[target_pose.target_ellipse].theta);
                            rst = false;
                        }
                        else
                            return false;
                    }
                    else
                        return false;
                }
            }
            else
                valid = get_center(objArray, center_u, center_v, p.x, p.y, true);
        }
        if (valid)
        {
            target_pose.calc(p, msg, MAX);
            if (store)
            {
                ROS_INFO("x:%lf y:%lf theta:%lf", msg.pose.x, msg.pose.y, msg.pose.theta);
                cargo_x_.push_back(msg.pose.x);
                cargo_y_.push_back(msg.pose.y);
                cargo_theta_.push_back(msg.pose.theta);
            }
            else
            {
                msg.header = image_rect->header;
                msg.header.frame_id = "base_footprint";
            }
        }
        if (valid && show_detections && !cv_image_.image.empty() && !pose)
        {
            draw_cross(cv_image_.image, cv::Point(center_u, center_v), cv::Scalar(255, 255, 255), 30, 3);
            debug_image = cv_image_.toImageMsg();
        }
        last_finish = store ? cargo_x_.size() >= MAX : msg.end;
        if (store && last_finish)
        {
            average_pose_once();
        }
        return store ? last_finish : valid;
    }

    bool ArmController::find_ellipse(const sensor_msgs::ImageConstPtr &image_rect, Pose2DMightEnd &msg,
                                     sensor_msgs::ImagePtr &debug_image, bool store)
    {
        static bool last_finish = true;
        static bool rst = false;
        const int MAX = 5; // 读取5次求平均位姿
        if (!msg.end && last_finish && !store)
        {
            last_finish = false;
            ps_.reset(true);
            cargo_theta_.clear();
            rst = true;
            return false;
        }
        else if (msg.end && last_finish && store)
        {
            cargo_x_.clear();
            cargo_y_.clear();
            cargo_theta_.clear();
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        vision_msgs::BoundingBox2DArray objArray;
        geometry_msgs::Pose2D p;
        bool valid = detect_ellipse(image_rect, objArray, debug_image, ellipse_roi_) &&
                     get_pose(objArray, z_parking_area, p, (rst && cargo_theta_.empty()));
        if (rst)
        {
            if (valid)
            {
                cargo_theta_.push_back(p.theta);
                ROS_INFO_STREAM("theta: " << p.theta);
                if (cargo_theta_.size() == MAX)
                {
                    cargo_theta_.erase(std::min_element(cargo_theta_.begin(), cargo_theta_.end()));
                    cargo_theta_.erase(std::max_element(cargo_theta_.begin(), cargo_theta_.end()));
                    average_theta(target_pose.pose[target_pose.target_ellipse].theta);
                    ROS_INFO_STREAM("Set theta to " << target_pose.pose[target_pose.target_ellipse].theta);
                    rst = false;
                }
                else
                    return false;
            }
            else
                return false;
        }
        if (valid)
        {
            target_pose.calc(p, msg, MAX);
            if (store)
            {
                ROS_INFO("x:%lf y:%lf theta:%lf", msg.pose.x, msg.pose.y, msg.pose.theta);
                cargo_x_.push_back(msg.pose.x);
                cargo_y_.push_back(msg.pose.y);
                cargo_theta_.push_back(msg.pose.theta);
            }
            else
            {
                if (rst)
                    rst = false;
                msg.header = image_rect->header;
                msg.header.frame_id = "base_footprint";
            }
        }
        last_finish = store ? cargo_x_.size() >= MAX : msg.end;
        if (store && last_finish)
        {
            average_pose_once();
        }
        return store ? last_finish : valid;
    }

    bool ArmController::find_border(const sensor_msgs::ImageConstPtr &image_rect, Pose2DMightEnd &msg,
                                    sensor_msgs::ImagePtr &debug_image)
    {
        static bool last_finish = true;
        if (!msg.end && last_finish)
        {
            // ps_.reset(true);
            ps_.look_down();
            last_finish = false;
            return false;
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        cv_bridge::CvImagePtr cv_image;
        if (!add_image(image_rect, cv_image))
            return false;
        cv_image->image = cv_image->image(border_roi_).clone();
        cv::Vec2f border;
        geometry_msgs::Pose2D p;
        bool valid = border_.detect(cv_image, border, border_roi_,
                                    boost::bind(&ArmController::LBD_color_func, this, _1, _2, threshold),
                                    show_detections, debug_image);
        if (valid)
            valid = ps_.calculate_border_position(border, z_parking_area, p.x, p.theta);
        if (valid)
        {
            target_pose.calc(p, msg);
            msg.header = image_rect->header;
            msg.header.frame_id = "base_footprint";
            if (msg.end)
                ps_.reset(true);
        }
        last_finish = msg.end;
        return valid;
    }

    bool ArmController::find_parking_area(const sensor_msgs::ImageConstPtr &image_rect, Pose2DMightEnd &msg,
                                          sensor_msgs::ImagePtr &debug_image)
    {
        using namespace cv;
        static bool flag = true;
        if (flag)
        {
            flag = false;
            ps_.reset();
            return false;
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        cv_bridge::CvImagePtr cv_image;
        if (!add_image(image_rect, cv_image))
            return false;
        if (ps_.refresh_xyz())
        {
            Mat M = ps_.transformation_matrix(z_parking_area);
            double ratio = z_parking_area / 80.0 * cv_image->image.rows; // 放大倍数
            M.rowRange(0, 2) *= ratio;                                   // 放大
            ratio /= (z_parking_area * 4);                               // 4是图片缩小倍数，用于还原真实坐标
            M.row(0) += M.row(2).clone() * cv_image->image.cols / 2.0;   // 平移
            // ROS_INFO_STREAM(M);
            warpPerspective(cv_image->image, cv_image->image, M, cv_image->image.size());
            // imshow("src", cv_image->image);
            // waitKey(100);
            resize(cv_image->image, cv_image->image, cv_image->image.size() / 2);
            pyrDown(cv_image->image, cv_image->image,
                    Size(cv_image->image.cols / 2, cv_image->image.rows / 2));
            Mat srcgray;
            srcgray = saturation(cv_image->image, 100);
            // imshow("saturation", srcgray);
            // waitKey(1);
            cvtColor(srcgray, srcgray, COLOR_BGR2GRAY); // 灰度转换
            // imshow("gray", srcgray);
            // waitKey(1);
            Mat srcbinary;
            cv::threshold(srcgray, srcbinary, 0, 255, THRESH_OTSU | THRESH_BINARY); // 阈值化
            // imshow("threshold", srcbinary);
            // waitKey(1);
            Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7), cv::Point(-1, -1));
            morphologyEx(srcbinary, srcbinary, MORPH_CLOSE, kernel, cv::Point(-1, -1), 1); // 闭操作去除噪点
            // imshow("MORPH_OPEN", srcbinary);
            // waitKey(1);
            Mat edges;
            Canny(srcbinary, edges, 0, 50, 3, false); // 查找边缘
            imshow("edges", edges);
            waitKey(1);
            std::vector<std::vector<cv::Point>> contours;
            std::vector<Vec4i> hierarchy;
            findContours(edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); // 只检测最外围的轮廓,只保留拐点的信息
            if (contours.size())
            {
                BestSquare s(contours, ratio);
                if (s.best.length)
                {
                    cv::Point2d center = s.best.center();
                    // 计算真实世界中坐标
                    msg.pose.x = (center.x - cv_image->image.cols / 2.0) / ratio;
                    msg.pose.y = center.y / ratio;
                    // msg.pose.theta
                    ROS_INFO_STREAM("x:" << msg.pose.x << " y:" << msg.pose.x);
                    if (show_detections && !cv_image->image.empty())
                    {
                        drawContours(cv_image->image, contours, s.best_id, cv::Scalar(0, 255, 0), 1); // 绘制矩形轮廓
                        draw_cross(cv_image->image, center, Scalar(255, 255, 255), 30, 2);
                        debug_image = cv_image->toImageMsg();
                    }
                }
                else
                {
                    ROS_WARN("Could not find parking area!");
                    return false;
                }
            }
            else
            {
                ROS_WARN("Could not find parking area!");
                return false;
            }
            return true;
        }
        else
            return false;
    }
} // namespace my_hand_eye
