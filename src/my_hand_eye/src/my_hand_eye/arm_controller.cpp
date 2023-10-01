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
          parking_area_roi_(320, 0, 1280, 720),
          yaed_(new cv::CEllipseDetectorYaed()),
          threshold(50), catched(false),
          z_parking_area(0.30121),
          z_ellipse(3.58369),
          z_turntable(11.77052) // 比赛转盘
    //   初始化列表记得复制一份到上面
    //   z_turntable(16.4750)// 老转盘（弃用）
    //   z_turntable(15.57)  // 新转盘（弃用）
    // z_palletize(10.2079),
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
          parking_area_roi_(320, 0, 1280, 720),
          yaed_(new cv::CEllipseDetectorYaed()),
          threshold(50), catched(false),
          z_parking_area(0.30121),
          z_ellipse(3.58369),
          z_turntable(11.77052) // 比赛转盘
    //   初始化列表记得复制一份到上面
    //   z_turntable(16.4750)// 老转盘（弃用）
    //   z_turntable(15.57)  // 新转盘（弃用）
    // z_palletize(10.2079),
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
        XmlRpc::XmlRpcValue catch_correct_action;
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
        factor_ = pnh.param<int>("factor", 40);
        speed_standard_static_ = pnh.param<double>("speed_standard_static", 0.16);
        speed_standard_motion_ = pnh.param<double>("speed_standard_motion", 0.12);
        tracker_.flag = pnh.param<bool>("flag", false);
        target_ellipse_theta_ = Angle(pnh.param<double>("target_ellipse_theta", -4.952260769)).rad();
        if (!ps_.begin(ft_servo.c_str()))
        {
            ROS_ERROR_STREAM("Cannot open ft servo at" << ft_servo);
        }
        ps_.ping();
        ps_.set_speed_and_acc(servo_descriptions);
        ps_.set_action(default_action);
        ps_.set_action(back_action, "back");
        ps_.set_action(down_action, "down");
        ps_.set_action(catch_correct_action, "catch_correct");
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
        cargo_x_.reserve(5);
        cargo_y_.reserve(5);
        cargo_theta_.reserve(5);
        left_x_.reserve(5);
        left_y_.reserve(5);
        right_x_.reserve(5);
        right_y_.reserve(5);
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
        cv_image_ = cv_bridge::CvImage(image->header, image->encoding, image->image.clone());
        return true;
    }

    bool ArmController::take_picture(const cv::Mat &img)
    {
        if (img.empty())
        {
            ROS_WARN("Empty image!");
            return false;
        }
        char str[80];
        static int n = 0;
        // 替换为保存图片的路径
        snprintf(str, sizeof(str), "/home/nano/car/car/car_ws/src/my_hand_eye/img/%d.jpg", ++n);
        cv::imwrite(str, img);
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
                    // 复制target_pose
                    Action a = Action(target_pose.pose[target_pose.target_ellipse].x,
                                      target_pose.pose[target_pose.target_ellipse].y, 0)
                                   .footprint2arm();
                    double u, v;
                    ps_.calculate_pixel_position(a.x, a.y, z_ellipse, u, v, true);
                    u -= rect.x;
                    v -= rect.y;
                    draw_cross(cv_image->image,
                               cv::Point2d(u, v),
                               cv::Scalar(255, 255, 255), 30, 2);
                    if (cargo.response.results.boxes.size())
                    {
                        for (int color = color_red; color <= color_blue; color++)
                        {
                            if (!cargo.response.results.boxes[color].center.x)
                                continue;
                            // cv::RotatedRect rect(cv::Point2d(cargo.response.results.boxes[color].center.x, cargo.response.results.boxes[color].center.y),
                            //                      cv::Size2d(cargo.response.results.boxes[color].size_x, cargo.response.results.boxes[color].size_y),
                            //                      cargo.response.results.boxes[color].center.theta);
                            // cv::Point2f vtx[4]; // 矩形顶点容器
                            // // cv::Mat dst = cv::Mat::zeros(cv_image->image.size(), CV_8UC3);//创建空白图像
                            // rect.points(vtx); // 确定旋转矩阵的四个顶点
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
                            draw_cross(cv_image->image,
                                       cv::Point2d(cargo.response.results.boxes[color].center.x,
                                                   cargo.response.results.boxes[color].center.y),
                                       colors, 30, 2);
                            // for (int j = 0; j < 4; j++)
                            // {
                            //     cv::line(cv_image->image, vtx[j], vtx[(j + 1) % 4], colors, 2);
                            // }
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
            return false;
        }
        static bool flag = true; // 尚未初始化位姿
        static bool rst = true;
        if (flag)
        {
            flag = false;
            ps_.reset(pose || (z == z_ellipse));
            if (pose)
            {
                Action ellipse = Action(0, 19.5, 0).front2left().arm2footprint();
                target_pose.pose[target_pose.target_ellipse].x = ellipse.x;
                target_pose.pose[target_pose.target_ellipse].y = ellipse.y;
            }
            else if (z != z_ellipse)
            {
                ps_.correction = true;
            }
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
                cv::Vec2f line;
                if (valid = get_position(objArray, z_ellipse, p.x, p.y, rst) &&
                            get_theta(objArray, z_ellipse, p.theta, line))
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

    bool ArmController::get_position(vision_msgs::BoundingBox2DArray &objArray,
                                     double z, double &ellipse_x, double &ellipse_y, bool rst, bool relative)
    {
        if (relative && rst)
        {
            ROS_ERROR("get_position: Cannot set both of relative and rst to true");
            return false;
        }
        if (objArray.boxes.size() == 4)
        {
            static double last_x, last_y;
            if (rst)
            {
                if (!set_color_order(objArray, z))
                    return false;
            }
            double x[5] = {0}, y[5] = {0};
            bool read = !rst;
            for (int i = 1; i <= 3; i++)
            {
                if (!relative && i != 2)
                    continue;
                if (!objArray.boxes[color_order_[i].color].center.x)
                {
                    ROS_WARN("get_position: Color %d ellipse does not exist.", color_order_[i].color);
                    return false;
                }
                if (!ps_.calculate_cargo_position(objArray.boxes[color_order_[i].color].center.x,
                                                  objArray.boxes[color_order_[i].color].center.y,
                                                  z, x[i], y[i], read))
                {
                    ROS_WARN("get_position: Cannot calculate color %d ellipse's position",
                             color_order_[i].color);
                    continue;
                }
                if (read)
                    read = false;
                if (i == 2)
                {
                    ellipse_x = x[i];
                    ellipse_y = y[i];
                }
            }
            // 防止超出车道
            if (ellipse_x < -34)
                ellipse_x = -34;
            else if (ellipse_x > -27.5)
                ellipse_x = -27.5;
            if (ellipse_y < -ARM_P - 8)
                ellipse_y = -ARM_P - 8;
            else if (ellipse_y > -ARM_P + 8)
                ellipse_y = -ARM_P + 8;
            // if (cnt == 3)
            //     pose.theta = -(atan((x[0] - x[1]) / (y[0] - y[1])) +
            //                    atan((x[1] - x[2]) / (y[1] - y[2])) +
            //                    atan((x[2] - x[0]) / (y[2] - y[0]))) /
            //                  3;
            // else if (cnt == 2)
            // {
            //     if (relative)
            //         return false;
            //     pose.theta = -atan((x[0] - x[1]) / (y[0] - y[1]));
            // }
            // else
            //     return false;
            if (relative)
                get_relative_position(x, y);
            if (!rst)
            {
                if (abs(last_x - ellipse_x) > 3)
                    ellipse_x = last_x + cv::sgn(ellipse_x - last_x);
                if (abs(last_y - ellipse_y) > 3)
                    ellipse_y = last_y + cv::sgn(ellipse_y - last_y);
            }
            last_x = ellipse_x;
            last_y = ellipse_y;
            return true;
        }
        return false;
    }

    bool ArmController::get_theta(vision_msgs::BoundingBox2DArray &objArray,
                                  double z, double &theta, cv::Vec2f &line)
    {
        int cnt = 0;
        double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_xy = 0;
        for (int i = 1; i <= 3; i++)
        {
            if (!objArray.boxes[color_order_[i].color].center.x)
            {
                ROS_WARN("get_theta: Color %d ellipse does not exist.", color_order_[i].color);
                continue;
            }
            cnt++;
            sum_x += objArray.boxes[color_order_[i].color].center.x;
            sum_y += objArray.boxes[color_order_[i].color].center.y;
            sum_x2 += objArray.boxes[color_order_[i].color].center.x *
                      objArray.boxes[color_order_[i].color].center.x;
            sum_xy += objArray.boxes[color_order_[i].color].center.x *
                      objArray.boxes[color_order_[i].color].center.y;
        }
        bool valid = true;
        // 对象相对车体的偏角
        if (cnt >= 2)
        {
            // y = a + bx
            double down = cnt * sum_x2 - sum_x * sum_x;
            double a = (sum_x2 * sum_y - sum_x * sum_xy) / down;
            double b = (cnt * sum_xy - sum_x * sum_y) / down;
            line[1] = atan(b) + CV_PI / 2;
            line[0] = abs(a * sin(line[1]));
            double dist;
            valid = ps_.calculate_border_position(line, z, dist, theta, false);
        }
        else
            return false;
        if (valid && abs(Angle::degree(theta - target_ellipse_theta_)) > 3)
        {
            theta = cv::sgn(theta -
                            target_ellipse_theta_) *
                        Angle(3).rad() +
                    target_ellipse_theta_;
            if (cnt != 2)
            {
                // 如果theta过大，尝试找到错误点
                cv::Vec2f line_try;
                for (int i = 1; i <= 3; i++)
                {
                    double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_xy = 0;
                    for (int j = 1; j <= 3; j++)
                    {
                        if (i == j)
                            continue;
                        sum_x += objArray.boxes[color_order_[j].color].center.x;
                        sum_y += objArray.boxes[color_order_[j].color].center.y;
                        sum_x2 += objArray.boxes[color_order_[j].color].center.x *
                                  objArray.boxes[color_order_[j].color].center.x;
                        sum_xy += objArray.boxes[color_order_[j].color].center.x *
                                  objArray.boxes[color_order_[j].color].center.y;
                    }
                    double down = 2 * sum_x2 - sum_x * sum_x;
                    double a = (sum_x2 * sum_y - sum_x * sum_xy) / down;
                    double b = (2 * sum_xy - sum_x * sum_y) / down;
                    line_try[1] = atan(b) + CV_PI / 2;
                    line_try[0] = abs(a * sin(line_try[1]));
                    double dist, theta_try;
                    if (ps_.calculate_border_position(line_try, z, dist, theta_try, false))
                    {
                        if (abs(theta_try - target_ellipse_theta_) <
                            abs(theta - target_ellipse_theta_))
                        {
                            ROS_WARN("get_theta: One point is invalid. Correct theta: %lf", theta);
                            theta = theta_try;
                        }
                    }
                }
            }
        }
        return valid;
    }

    bool ArmController::set_color_order(vision_msgs::BoundingBox2DArray &objArray, double z)
    {
        if (objArray.boxes.size() == 4)
        {
            bool read = true;
            double x;
            for (int color = color_red; color <= color_blue; color++)
            {
                if (!objArray.boxes[color].center.x)
                {
                    ROS_WARN("set_color_order: Color %d ellipse does not exist.", color);
                    return false;
                }
                if (!ps_.calculate_cargo_position(objArray.boxes[color].center.x,
                                                  objArray.boxes[color].center.y,
                                                  z, x, color_order_[color].center_y, read))
                {
                    ROS_WARN("set_color_order: Cannot calculate color %d ellipse's position",
                             color);
                    continue;
                }
                if (read)
                    read = false;
                color_order_[color].color = color;
            }
            std::sort(color_order_ + 1, color_order_ + 4, [](EllipseColor e1, EllipseColor e2)
                      { return e1.center_y < e2.center_y; });
            ROS_INFO("Color order: %d%d%d", color_order_[1].color,
                     color_order_[2].color, color_order_[3].color);
            ROS_INFO("Color y: %lf %lf %lf", color_order_[1].center_y,
                     color_order_[2].center_y, color_order_[3].center_y);
            if (abs(color_order_[2].center_y - color_order_[1].center_y - 15) > 3 ||
                abs(color_order_[3].center_y - color_order_[2].center_y - 15) > 3)
            {
                ROS_WARN("set_color_order: Invalid position!");
                return false;
            }
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

    void ArmController::get_relative_position(double x[], double y[])
    {
        Action left = Action(x[1] - x[2], y[1] - y[2], 0).arm2footprint();
        ROS_INFO("left:");
        ARM_INFO_XYZ(left);
        left_x_.push_back(left.x);
        left_y_.push_back(left.y);
        Action right = Action(x[3] - x[2], y[3] - y[2], 0).arm2footprint();
        ROS_INFO("right:");
        ARM_INFO_XYZ(right);
        right_x_.push_back(right.x);
        right_y_.push_back(right.y);
    }

    void ArmController::average_position(double &x, double &y, int order)
    {
        switch (order)
        {
        case 1:
            if (left_x_.empty() || left_y_.empty())
            {
                ROS_WARN_STREAM("left x is empty: " << left_x_.empty() << ", y is empty: " << left_y_.empty());
                x = y = 0;
                return;
            }
            x = std::accumulate(std::begin(left_x_), std::end(left_x_), 0.0) / left_x_.size();
            y = std::accumulate(std::begin(left_y_), std::end(left_y_), 0.0) / left_y_.size();
            break;

        case 2:
            if (cargo_x_.empty() || cargo_y_.empty())
            {
                ROS_WARN_STREAM("cargo x is empty: " << cargo_x_.empty() << ", y is empty: " << cargo_y_.empty());
                x = y = 0;
                return;
            }
            x = std::accumulate(std::begin(cargo_x_), std::end(cargo_x_), 0.0) / cargo_x_.size();
            y = std::accumulate(std::begin(cargo_y_), std::end(cargo_y_), 0.0) / cargo_y_.size();
            break;

        case 3:
            if (right_x_.empty() || right_y_.empty())
            {
                ROS_WARN_STREAM("right x is empty: " << right_x_.empty() << ", y is empty: " << right_y_.empty());
                x = y = 0;
                return;
            }
            x = std::accumulate(std::begin(right_x_), std::end(right_x_), 0.0) / right_x_.size();
            y = std::accumulate(std::begin(right_y_), std::end(right_y_), 0.0) / right_y_.size();
            break;

        default:
            ROS_ERROR("average_position: Invalid order %d", order);
            x = y = 0;
            break;
        }
    }

    void ArmController::average_theta(double &theta)
    {
        if (cargo_theta_.empty())
        {
            ROS_WARN("cargo theta is empty!");
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
        if (cargo_x_.size() <= 2 || cargo_y_.size() <= 2 || cargo_theta_.size() <= 2 ||
            left_x_.size() <= 2 || left_y_.size() <= 2 || right_x_.size() <= 2 || right_y_.size() <= 2)
        {
            ROS_WARN("Please set all vectors' size > 2");
            return;
        }
        // 去除一个最大值，一个最小值
        erase_max_min(cargo_x_);
        erase_max_min(cargo_y_);
        erase_max_min(cargo_theta_);
        erase_max_min(left_x_);
        erase_max_min(left_y_);
        erase_max_min(right_x_);
        erase_max_min(right_y_);
        geometry_msgs::Pose2D pose;
        average_pose(pose);
        clear(true, true);
        cargo_x_.push_back(pose.x);
        cargo_y_.push_back(pose.y);
        cargo_theta_.push_back(pose.theta);
        average_position(pose.x, pose.y, 1);
        clear(false, false, true, false);
        left_x_.push_back(pose.x);
        left_y_.push_back(pose.y);
        average_position(pose.x, pose.y, 3);
        clear(false, false, false, true);
        right_x_.push_back(pose.x);
        right_y_.push_back(pose.y);
    }

    void ArmController::clear(bool clear_mid, bool clear_theta,
                              bool clear_left, bool clear_right)
    {
        if (clear_mid)
        {
            cargo_x_.clear();
            cargo_y_.clear();
        }
        if (clear_theta)
            cargo_theta_.clear();
        if (clear_left)
        {
            left_x_.clear();
            left_y_.clear();
        }
        if (clear_right)
        {
            right_x_.clear();
            right_y_.clear();
        }
    }

    void ArmController::erase_max_min(std::vector<double> &vec)
    {
        if (vec.size() <= 2)
        {
            ROS_WARN("Please set vec's size > 2");
            return;
        }
        vec.erase(std::min_element(vec.begin(), vec.end()));
        vec.erase(std::max_element(vec.begin(), vec.end()));
    }

    void ArmController::error_position(const Color color, bool pal,
                                       double &err_x, double &err_y, double &err_theta)
    {
        geometry_msgs::Pose2D err;
        average_pose(err);
        if (color_map_[color] == 1)
        {
            double x, y;
            average_position(x, y, color_map_[color]);
            if (x && y)
            {
                double theta_tr = Angle(Angle::degree(
                                            (theta_turn == Pose2DMightEnd::not_change) ? 0 : theta_turn))
                                      .goal2now(ps_.enlarge_loop[pal])
                                      .rad();
                double ex = (sqrt(x * x + y * y) * cos(atan(y / x) - theta_tr) - (-x));
                double ey = -(-y - sqrt(x * x + y * y) * sin(atan(y / x) - theta_tr));
                // ROS_INFO_STREAM(x << " " << y << " " << sqrt(x * x + y * y) << " " << atan(y / x) << " " << theta_tr);
                ROS_INFO("err_x: %lf err_y: %lf order: %d", ex, ey, color_map_[color]);
                err_x += ex;
                err_y += ey;
            }
        }
        else if (color_map_[color] == 3)
        {
            double x, y;
            average_position(x, y, color_map_[color]);
            if (x && y)
            {
                double theta_tr = Angle(Angle::degree(
                                            (theta_turn == Pose2DMightEnd::not_change) ? 0 : theta_turn))
                                      .goal2now(ps_.enlarge_loop[pal])
                                      .rad();
                double ex = -(sqrt(x * x + y * y) * cos(atan(y / x) - theta_tr) - x);
                double ey = (y - sqrt(x * x + y * y) * sin(atan(y / x) - theta_tr));
                // ROS_INFO_STREAM(x << " " << y << " " << sqrt(x * x + y * y) << " " << atan(y / x) << " " << theta_tr);
                ROS_INFO("err_x: %lf err_y: %lf order: %d", ex, ey, color_map_[color]);
                err_x += ex;
                err_y += ey;
            }
        }
        err_x = err.x * cos(target_ellipse_theta_) +
                err.y * sin(target_ellipse_theta_);
        err_y = -err.x * sin(target_ellipse_theta_) +
                err.y * cos(target_ellipse_theta_);
        err_theta = Angle(Angle::degree(err.theta)).now2goal(ps_.enlarge_loop[pal]).rad();
        ROS_INFO("err_x: %lf err_y: %lf err_theta: %lf theta_turn:%lf", err_x, err_y, err_theta,
                 (theta_turn == Pose2DMightEnd::not_change) ? 0 : theta_turn);
        // err_x = 0;
        // err_y = 0;
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
            ps_.correction = true;
            return false;
        }
        else if (current_color_ != color || cargo_x_.size() >= MAX_SIZE)
        {
            clear(true, false);
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
                    clear(true, false);
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
                clear(true, false);
            }
            cnt_motion = 0;
            motion_before = false;
            return false;
        }
        if (speed < 0) // -1
        {
            if (!cargo_x_.empty() || !cargo_y_.empty())
            {
                clear(true, false);
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
                clear(true, false);
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
            ps_.correction = true;
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
            ps_.correction = true;
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
            static ros::Time stop;
            ROS_INFO_STREAM("x:" << x << " y:" << y);
            if (!stop.is_zero() && (ros::Time::now() - stop).toSec() < 4)
            {
                finish = false;
                return false;
            }
            bool valid = ps_.go_to_and_wait(x, y, z_turntable, true);
            if (valid)
            {
                ps_.go_to_table(false, color, left);
            }
            else
            {
                finish = false;
                stop = ros::Time::now();
                return false;
            }
            can_catch_ = true;
            stop_ = false;
            finish = true;
            ps_.correction = false;
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
        // 重设大小，可选
        resize(cv_image->image, cv_image->image, Size(cv_image->image.cols / 2, cv_image->image.rows / 2));
        //*******
        // 以下为使用roi区域有选择的调节暗部区域
        //*******

        // 转换为灰度图，寻找高光区域
        Mat Gray;
        cvtColor(cv_image->image, Gray, COLOR_BGR2GRAY);
        // 阈值化分离高光区域
        Mat Thresh_Gray;
        cv::threshold(Gray, Thresh_Gray, 120, 255, THRESH_BINARY);
        // 取反获得暗部区域
        bitwise_not(Thresh_Gray, Thresh_Gray);
        // cv::imshow("Thresh_origin", Thresh_Gray);

        // 膨胀操作，消除小噪点，也可以不用这一步，因为后面也会根据轮廓大小进行筛除小噪点
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3), cv::Point(-1, -1));
        dilate(Thresh_Gray, Thresh_Gray, kernel);
        // cv::imshow("Thresh_dilate", Thresh_Gray);

        // 轮廓查找
        std::vector<std::vector<cv::Point>> contours;
        std::vector<Vec4i> hierarchy;
        findContours(Thresh_Gray, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        // 按区域gramma矫正
        Mat Img_clone = cv_image->image.clone();
        for (size_t i = 0; i < contours.size(); i++)
        {
            // 滤除太小的区域
            if (contourArea(contours[i]) < 10000)
                continue;
            // 创建掩膜
            Rect roi = boundingRect(contours[i]);
            Mat mask = Img_clone(roi);
            // gramma矫正，并对每个区域进行颜色增强
            gramma_transform(factor_, mask);
            mask = saturation(mask, 40);
        }
        // *******
        // 也可以对整张图片进行全局gramma矫正，直接使用gramma_transform(int Factor, Mat &resImg)即可
        // *******
        // 全局颜色增强
        Img_clone = saturation(Img_clone, 10);
        // imshow("resImg", Img_clone);
        // take_picture(cv_image->image);
        Mat hsv;
        cvtColor(Img_clone, hsv, COLOR_BGR2HSV);
        std::vector<Mat1b> hsvs;
        split(hsv, hsvs);
        // imshow("s", hsvs[1]);
        // waitKey(20);
        std::vector<cv::Ellipse> ellsYaed;
        yaed_->Detect(hsvs[1], ellsYaed);
        EllipseArray arr;
        // 聚类、颜色标定
        if (!arr.clustering(ellsYaed, cv_image) ||
            !arr.color_classification(cv_image, white_vmin_))
        {
            if (show_detections && !cv_image->image.empty())
            {
                Mat3b srcCopy = cv_image->image;
                yaed_->DrawDetectedEllipses(srcCopy, ellsYaed, 0, 1);
                // imshow("Yaed", srcCopy);
                // waitKey(10);
                debug_image = cv_image->toImageMsg();
            }
            return false;
        }
        detections.header = image_rect->header;
        if (!arr.detection(detections, rect, cv_image))
        {
            if (show_detections && !cv_image->image.empty())
            {
                Mat3b srcCopy = cv_image->image;
                yaed_->DrawDetectedEllipses(srcCopy, ellsYaed, 0, 1);
                // imshow("Yaed", srcCopy);
                // waitKey(10);
                debug_image = cv_image->toImageMsg();
            }
            return false;
        }
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
                        Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
                        dilate(thr, thr, element);
                        Moments m = moments(thr, true);
                        detections.boxes[color].center.x = m.m10 / m.m00 + rect.x;
                        detections.boxes[color].center.y = m.m01 / m.m00 + rect.y;
                    }
                }
                if (show_detections && detections.boxes[color].center.x)
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
            {
                // 复制target_pose
                Action a = Action(target_pose.pose[target_pose.target_ellipse].x,
                                  target_pose.pose[target_pose.target_ellipse].y, 0)
                               .footprint2arm();
                double u, v;
                ps_.calculate_pixel_position(a.x, a.y, z_parking_area, u, v, false);
                draw_cross(cv_image_.image,
                           Point2d(u, v),
                           Scalar(255, 255, 255), 30, 2);
                debug_image = cv_image_.toImageMsg();
            }
            // Mat3b srcCopy = cv_image->image;
            // yaed_->DrawDetectedEllipses(srcCopy, ellsYaed, 0, 1);
            // // imshow("Yaed", srcCopy);
            // // waitKey(10);
            // debug_image = cv_image->toImageMsg();
        }
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

    bool ArmController::detect_parking_area(const sensor_msgs::ImageConstPtr &image_rect,
                                            geometry_msgs::Pose2D &pose,
                                            sensor_msgs::ImagePtr &debug_image, cv::Rect &rect)
    {
        cv_bridge::CvImagePtr cv_image;
        if (!add_image(image_rect, cv_image))
            return false;
        using namespace cv;
        if (ps_.refresh_xyz())
        {
            Mat M = ps_.transformation_matrix(z_parking_area);
            double ratio = z_parking_area / 80.0 * cv_image->image.rows; // 放大倍数
            M.rowRange(0, 2) *= ratio;                                   // 放大
            ratio /= (z_parking_area * 2);                               // 2是图片缩小倍数，用于还原真实坐标
            M.row(0) += M.row(2).clone() * cv_image->image.cols / 2.0;   // 平移
            // ROS_INFO_STREAM(M);
            warpPerspective(cv_image->image, cv_image->image, M, cv_image->image.size());
            // imshow("src", cv_image->image);
            // waitKey(100);
            cv_image->image = cv_image->image(rect).clone();
            // resize(cv_image->image, cv_image->image, cv_image->image.size() / 2);
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
            morphologyEx(srcbinary, srcbinary, MORPH_CLOSE, kernel, cv::Point(-1, -1), 2); // 闭操作去除噪点
            morphologyEx(srcbinary, srcbinary, MORPH_OPEN, kernel, cv::Point(-1, -1));     // 开操作去除缺口
            // imshow("MORPH_OPEN", srcbinary);
            // waitKey(1);
            Mat edges;
            Canny(srcbinary, edges, 0, 50, 3, false); // 查找边缘
            // imshow("edges", edges);
            // waitKey(1);
            std::vector<std::vector<cv::Point>> contours;
            std::vector<Vec4i> hierarchy;
            findContours(edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); // 只检测最外围的轮廓,只保留拐点的信息
            if (contours.size())
            {
                BestSquare s(contours, ratio);
                if (s.best.length && s.best.get_pose(pose))
                {
                    if (show_detections && !cv_image->image.empty())
                    {
                        // 绘制矩形轮廓
                        RotatedRect rotate_rect(Point2f(pose.x, pose.y), Size2f(s.best.length, s.best.length),
                                                Angle::degree(pose.theta));
                        // 获取旋转矩形的四个顶点
                        Point2f *vertices = new Point2f[4];
                        rotate_rect.points(vertices);
                        // 逐条边绘制
                        for (int j = 0; j < 4; j++)
                        {
                            cv::line(cv_image->image, vertices[j], vertices[(j + 1) % 4],
                                     cv::Scalar(0, 255, 0));
                        }
                        // 当前绘制白
                        draw_cross(cv_image->image, cv::Point2d(pose.x, pose.y), Scalar(255, 255, 255),
                                   30, 1);
                        Action a = Action(target_pose.pose[target_pose.target_parking_area].x,
                                          target_pose.pose[target_pose.target_parking_area].y, 0)
                                       .footprint2arm();
                        a *= ratio;
                        a.x += cv_image->image.cols / 2.0;
                        // 目标绘制红
                        draw_cross(cv_image->image, cv::Point2d(a.x, a.y), Scalar(0, 0, 255),
                                   30, 1);
                        debug_image = cv_image->toImageMsg();
                        delete[] vertices;
                    }
                    // 计算真实世界中坐标
                    pose.x = (pose.x - cv_image->image.cols / 2.0) / ratio;
                    pose.y = pose.y / ratio;
                }
                else
                {
                    if (show_detections && !cv_image->image.empty())
                    {
                        for (size_t i = 0; i < contours.size(); i++)
                        {
                            drawContours(cv_image->image, contours, i, cv::Scalar(0, 255, 0), 1); // 绘制矩形轮廓
                        }
                        debug_image = cv_image->toImageMsg();
                    }
                    ROS_WARN("Could not find parking area!");
                    return false;
                }
            }
            else
            {
                if (show_detections && !cv_image->image.empty())
                    debug_image = cv_image->toImageMsg();
                ROS_WARN("Could not find parking area!");
                return false;
            }
            return true;
        }
        return false;
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
                cv::Vec2f line;
                if (valid = get_position(objArray, z_parking_area, p.x, p.y, rst, false) &&
                            get_theta(objArray, z_parking_area, p.theta, line))
                {
                    ROS_INFO_STREAM("x:" << p.x << " y:" << p.y << " theta:" << Angle::degree(p.theta));
                    if (rst)
                        rst = false;
                    if (show_detections && !cv_image_.image.empty())
                    {
                        plot_line(cv_image_.image, line[0], line[1], cv::Scalar(255, 255, 255));
                        debug_image = cv_image_.toImageMsg();
                    }
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
    //         valid = get_position(objArray, z_parking_area, pme, false);
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
    //             if (get_position(objArray, z_parking_area, pme, false))
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
        double err_x, err_y, err_theta;
        error_position(color, pal, err_x, err_y, err_theta);
        if (final)
        {
            clear(true, true, true, true);
        }
        bool valid = ps_.go_to_table(true, color, true) &&
                     ps_.put(color_map_[color], false, err_x, err_y, err_theta, pal);
        ps_.reset(true);
        return valid;
    }

    bool ArmController::catch_after_putting(const Color color, bool final)
    {
        double err_x, err_y, err_theta;
        error_position(color, false, err_x, err_y, err_theta);
        if (final)
        {
            clear(true, true, true, true);
        }
        bool valid = ps_.put(color_map_[color], true, err_x, err_y, err_theta, false) &&
                     ps_.go_to_table(false, color, true);
        if (!final)
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
        Border::Detected detected;
        bool valid = border_.detect(cv_image, border, border_roi_, detected, threshold,
                                    show_detections, debug_image);
        if (valid)
        {
            if (detected == Border::detected_grey)
            {
                ROS_INFO("Grey color detected.");
                return valid;
            }
            else if (detected == Border::detected_yellow)
            {
                ROS_INFO("Yellow color detected.");
                return valid;
            }
            valid = ps_.calculate_border_position(border, z_parking_area, distance, yaw);
        }
        if (valid)
        {
            ROS_INFO_STREAM("distance: " << distance << " yaw: " << Angle::degree(yaw));
            if (show_detections && !cv_image_.image.empty())
            {
                cv::Vec2f border0;
                // 复制target_pose
                Action a = Action(target_pose.pose[target_pose.target_border].x,
                                  target_pose.pose[target_pose.target_border].y, 0)
                               .footprint2arm();
                valid = ps_.calculate_line_position(a.x,
                                                    target_pose.pose[target_pose.target_border].theta,
                                                    z_parking_area, border0, false);
                plot_line(cv_image_.image, border0[0], border0[1], cv::Scalar(0, 0, 255));
                // 红线是目标
                plot_line(cv_image_.image, border[0], border[1], cv::Scalar(255, 255, 255));
                // 白线是检测
                debug_image = cv_image_.toImageMsg();
            }
        }
        return valid;
    }

    bool ArmController::log_parking_area(const sensor_msgs::ImageConstPtr &image_rect,
                                         sensor_msgs::ImagePtr &debug_image)
    {
        static bool flag = true;
        if (flag)
        {
            ps_.reset(false);
            flag = false;
            return false;
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        geometry_msgs::Pose2D p;
        bool valid = detect_parking_area(image_rect, p, debug_image, parking_area_roi_);
        if (valid)
            ROS_INFO_STREAM("x:" << p.x << " y:" << p.y << " theta:" << Angle::degree(p.theta));
        return valid;
    }

    void ArmController::ready(bool left)
    {
        ps_.reset(left);
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
        static double target_x;
        static double target_y;
        const int MAX = 5; // 读取5次求平均位姿
        if (!msg.end && last_finish && !store)
        {
            ps_.reset(pose);
            last_finish = false;
            if (pose)
            {
                target_x = target_pose.pose[target_pose.target_ellipse].x;
                target_y = target_pose.pose[target_pose.target_ellipse].y;
                Action ellipse = Action(0, 19.5, 0).front2left().arm2footprint();
                target_pose.pose[target_pose.target_ellipse].x = ellipse.x;
                target_pose.pose[target_pose.target_ellipse].y = ellipse.y;
                rst = true;
            }
            else
            {
                ps_.correction = true;
            }
            return false;
        }
        else if (msg.end && !last_finish && !store)
        {
            last_finish = msg.end;
            if (pose)
            {
                clear(true, true, true, true);
                if (rst)
                {
                    // 尚未获得顺序，任意指定顺序
                    color_map_.insert(std::pair<Color, int>(color_red, 1));
                    color_map_.insert(std::pair<Color, int>(color_green, 2));
                    color_map_.insert(std::pair<Color, int>(color_blue, 3));
                    rst = false;
                }
            }
            else
            {
                ps_.correction = true;
            }
            return true;
        }
        else if (msg.end && last_finish && store)
        {
            clear(true, true, true, true);
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
                valid = get_position(objArray, z_ellipse, p.x, p.y, rst, store);
            }
            else
                valid = get_center(objArray, center_u, center_v, p.x, p.y, true);
        }
        if (valid)
        {
            target_pose.calc(p, msg, MAX);
            if (store)
            {
                cv::Vec2f line;
                if (get_theta(objArray, z_parking_area, p.theta, line))
                {
                    msg.pose.theta = p.theta - target_ellipse_theta_;
                    cargo_theta_.push_back(msg.pose.theta);
                }
                else
                    cargo_theta_.push_back(0);
                ROS_INFO("x:%lf y:%lf theta:%lf", msg.pose.x, msg.pose.y, msg.pose.theta);
                cargo_x_.push_back(msg.pose.x);
                cargo_y_.push_back(msg.pose.y);
            }
            else
            {
                if (rst)
                    rst = false;
                msg.header = image_rect->header;
                msg.header.frame_id = "base_footprint";
                cv::Vec2f line;
                if (pose && msg.end && get_theta(objArray, z_parking_area, p.theta, line))
                {
                    msg.pose.theta = p.theta - target_ellipse_theta_;
                }
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
            target_pose.pose[target_pose.target_ellipse].x = target_x;
            target_pose.pose[target_pose.target_ellipse].y = target_y;
            // average_pose(msg.pose);
            // msg.header = image_rect->header;
            // msg.header.frame_id = "base_footprint";
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
            rst = true;
            return false;
        }
        else if (msg.end && !last_finish && !store)
        {
            last_finish = msg.end;
            clear(true, true, true, true);
            if (rst)
            {
                // 尚未获得顺序，任意指定顺序
                color_map_.insert(std::pair<Color, int>(color_red, 1));
                color_map_.insert(std::pair<Color, int>(color_green, 2));
                color_map_.insert(std::pair<Color, int>(color_blue, 3));
                rst = false;
            }
            return true;
        }
        else if (msg.end && last_finish && store)
        {
            clear(true, true, true, true);
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        vision_msgs::BoundingBox2DArray objArray;
        geometry_msgs::Pose2D p;
        bool valid = detect_ellipse(image_rect, objArray, debug_image, ellipse_roi_) &&
                     get_position(objArray, z_parking_area, p.x, p.y, rst, store);
        if (valid)
        {
            target_pose.calc(p, msg, MAX);
            if (store)
            {
                if (valid)
                {
                    cv::Vec2f line;
                    if (get_theta(objArray, z_parking_area, p.theta, line))
                    {
                        msg.pose.theta = p.theta - target_ellipse_theta_;
                        cargo_theta_.push_back(msg.pose.theta);
                    }
                    else
                        cargo_theta_.push_back(0);
                    ROS_INFO("x:%lf y:%lf theta:%lf", msg.pose.x, msg.pose.y, msg.pose.theta);
                    cargo_x_.push_back(msg.pose.x);
                    cargo_y_.push_back(msg.pose.y);
                }
            }
            else
            {
                if (rst)
                    rst = false;
                msg.header = image_rect->header;
                msg.header.frame_id = "base_footprint";
                cv::Vec2f line;
                if (msg.end && get_theta(objArray, z_parking_area, p.theta, line))
                {
                    msg.pose.theta = p.theta - target_ellipse_theta_;
                }
            }
        }
        last_finish = store ? cargo_x_.size() >= MAX : msg.end;
        if (store && last_finish)
        {
            average_pose_once();
            // average_pose(msg.pose);
            // msg.header = image_rect->header;
            // msg.header.frame_id = "base_footprint";
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
            clear(true, true);
            return false;
        }
        else if (msg.end && !last_finish)
        {
            last_finish = msg.end;
            clear(true, true);
            return true;
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        cv_bridge::CvImagePtr cv_image;
        if (!add_image(image_rect, cv_image))
            return false;
        cv_image->image = cv_image->image(border_roi_).clone();
        cv::Vec2f border;
        geometry_msgs::Pose2D p;
        Border::Detected detected;
        bool valid = border_.detect(cv_image, border, border_roi_, detected, threshold,
                                    show_detections, debug_image);
        if (valid)
        {
            msg.header = image_rect->header;
            msg.header.frame_id = "base_footprint";
            if (abs(Angle::degree(msg.pose.theta - target_pose.pose[target_pose.target_border].theta)) > 3)
                msg.pose.theta = cv::sgn(msg.pose.theta -
                                         target_pose.pose[target_pose.target_border].theta) *
                                     Angle(3).rad() +
                                 target_pose.pose[target_pose.target_border].theta;
            if (detected == Border::detected_grey)
            {
                ROS_INFO("Grey color detected.");
                msg.pose.x = msg.not_change;
                msg.pose.y = 0.035;
                msg.pose.theta = msg.not_change;
                msg.end = false;
                return valid;
            }
            else if (detected == Border::detected_yellow)
            {
                ROS_INFO("Yellow color detected.");
                msg.pose.x = msg.not_change;
                msg.pose.y = -0.035;
                msg.pose.theta = msg.not_change;
                msg.end = false;
                return valid;
            }
            valid = ps_.calculate_border_position(border, z_parking_area, p.x, p.theta);
        }
        if (valid)
        {
            if (target_pose.calc(p, msg))
            {
                cargo_x_.push_back(0);
                cargo_y_.push_back(msg.pose.y);
                cargo_theta_.push_back(msg.pose.theta);
                if (msg.end)
                {
                    average_pose(msg.pose);
                    msg.pose.x = msg.not_change;
                }
                clear(true, true);
            }
        }
        else if (!cargo_x_.empty())
        {
            clear(true, true);
        }
        last_finish = msg.end;
        return valid;
    }

    bool ArmController::find_parking_area(const sensor_msgs::ImageConstPtr &image_rect, Pose2DMightEnd &msg,
                                          sensor_msgs::ImagePtr &debug_image)
    {
        static bool last_finish = true;
        if (!msg.end && last_finish)
        {
            ps_.reset();
            last_finish = false;
            clear(true, true);
            return false;
        }
        else if (msg.end && !last_finish)
        {
            last_finish = msg.end;
            clear(true, true);
            return true;
        }
        if (!ps_.check_stamp(image_rect->header.stamp))
            return false;
        geometry_msgs::Pose2D p;
        bool valid = detect_parking_area(image_rect, p, debug_image, parking_area_roi_);
        if (valid)
        {
            if (Angle::degree(msg.pose.theta - target_pose.pose[target_pose.target_parking_area].theta) > 3)
                msg.pose.theta = Angle(3).rad();
            else if (Angle::degree(msg.pose.theta - target_pose.pose[target_pose.target_parking_area].theta) < -1)
                msg.pose.theta = Angle(-2).rad();
            if (target_pose.calc(p, msg))
            {
                cargo_x_.push_back(msg.pose.x);
                cargo_y_.push_back(msg.pose.y);
                cargo_theta_.push_back(msg.pose.theta);
                if (msg.end)
                    average_pose(msg.pose);
                clear(true, true);
            }
            else if (!cargo_x_.empty())
            {
                clear(true, true);
            }
            msg.header = image_rect->header;
            msg.header.frame_id = "base_footprint";
        }
        last_finish = msg.end;
        return valid;
    }
} // namespace my_hand_eye
