#include "my_hand_eye/arm_controller.h"

namespace my_hand_eye
{
    EllipseArray::EllipseArray()
    {
        ellipse_.reserve(3);
    };

    bool EllipseArray::clustering(std::vector<cv::Point2d> &centers, std::vector<cv::RotatedRect> &ellipses)
    {
        if (centers.empty() || ellipses.empty())
            return false;
        if (!ellipse_.empty())
            ellipse_.clear();
        const int MAXN = centers.size() + 10;
        flag_.resize(MAXN, 0);
        // ROS_INFO_STREAM(centers.size());
        // 聚类
        for (int i = 0; i < centers.size(); i++)
        {
            if (flag_[i])
                continue;
            int x_temp = centers[i].x, y_temp = centers[i].y, count = 1, now = i;
            for (int j = i + 1; j < centers.size(); j++)
            {
                float thr = std::min<float>(std::min<float>(ellipses[i].size.width, ellipses[i].size.height),
                                            std::min<float>(ellipses[j].size.width, ellipses[j].size.height)) /
                            2;
                if ((abs(centers[i].x - centers[j].x) < thr) &&
                    (abs(centers[i].y - centers[j].y) < thr) &&
                    !flag_[j])
                {

                    flag_[now] = j;
                    now = j;
                    x_temp = x_temp + centers[j].x;
                    y_temp = y_temp + centers[j].y;
                    count++;
                }
                if (j == centers.size() - 1)
                    flag_[now] = -1;
            }
            if (i == centers.size() - 1)
                flag_[now] = -1;
            if (count > 2)
            {
                Ellipse e;
                // 平均数求聚类中心，感觉不太妥当，但是精度感觉还行，追求精度的话可以用 Weiszfeld 算法求中位中心，那个要迭代
                e.center = cv::Point(x_temp / count, y_temp / count);
                ellipse_.push_back(e);
            }
        }
        // ROS_INFO_STREAM(ellipse_.size());
        return true;
    };

    ArmController::ArmController()
        : ps_(&sm_st_, &sc_),
          default_roi_(480, 0, 960, 1080),
          fin_(false)
    {
        cargo_x_.reserve(10);
        cargo_y_.reserve(10);
    };

    ArmController::ArmController(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : ps_(&sm_st_, &sc_),
          default_roi_(0, 0, 1920, 1080),
          fin_(false)
    {
        init(nh, pnh);
    }

    ArmController::~ArmController()
    {
        if (!emulation_)
            ps_.end();
    }

    void ArmController::init(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        emulation_ = pnh.param<bool>("if_emulation", false);
        if (emulation_)
        {
            plot_client_ = nh.serviceClient<my_hand_eye::Plot>("height_plot");
            white_vmin_ = pnh.param<int>("white_vmin", 170);
            cargo_client_ = nh.serviceClient<yolov5_ros::cargoSrv>("cargoSrv");
            return;
        }
        XmlRpc::XmlRpcValue servo_descriptions;
        XmlRpc::XmlRpcValue default_action;
        XmlRpc::XmlRpcValue left_action;
        XmlRpc::XmlRpcValue front_action;
        if (!pnh.getParam("servo", servo_descriptions))
        {
            ROS_ERROR("No speed and acc specified");
        }
        if (!pnh.getParam("default_action", default_action))
        {
            ROS_ERROR("No default action specified");
        }
        if (!pnh.getParam("left_action", left_action))
        {
            ROS_ERROR("No left action specified");
        }
        if (!pnh.getParam("front_action", front_action))
        {
            ROS_ERROR("No front action specified");
        }
        std::string ft_servo;
        pnh.param<std::string>("ft_servo", ft_servo, "/dev/ft_servo");
        ROS_INFO_STREAM("serial:" << ft_servo);
        white_vmin_ = pnh.param<int>("white_vmin", 170);
        speed_standard_ = pnh.param<double>("speed_standard", 0.12);
        if (!ps_.begin(ft_servo.c_str()))
        {
            ROS_ERROR_STREAM("Cannot open ft servo at" << ft_servo);
        }
        ps_.ping();
        ps_.set_speed_and_acc(servo_descriptions);
        ps_.set_action(default_action);
        ps_.set_action(left_action, "left");
        ps_.set_action(front_action, "front");
        ps_.show_voltage();

        cargo_client_ = nh.serviceClient<yolov5_ros::cargoSrv>("cargoSrv");
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

    bool ArmController::detect_cargo(const sensor_msgs::ImageConstPtr &image_rect,
                                     vision_msgs::BoundingBox2DArray &detections, sensor_msgs::ImagePtr &debug_image, cv::Rect &roi)
    {
        if (cargo_client_.exists())
        {
            cv_bridge::CvImagePtr cv_image;
            bool flag = add_image(image_rect, cv_image);
            cv_image->image = cv_image->image(roi);
            yolov5_ros::cargoSrv cargo;
            sensor_msgs::ImagePtr image = (*cv_image).toImageMsg();
            cargo.request.image = *image;
            // 发送请求,返回 bool 值，标记是否成功
            if (flag)
                flag = cargo_client_.call(cargo);
            if (flag)
            {
                detections = cargo.response.results;
                if (show_detections_ && !cv_image->image.empty())
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
                            rect.points(vtx);
                            cv::Scalar colors; // 确定旋转矩阵的四个顶点
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
                ROS_WARN("Responce invalid or failed to add image!");
            return flag;
        }
        else
            return false;
    }

    bool ArmController::log_position_main(const sensor_msgs::ImageConstPtr &image_rect, double z, int color, sensor_msgs::ImagePtr &debug_image)
    {
        static bool flag = false; // 尚未初始化位姿
        if (!flag)
        {
            flag = true;
            ps_.reset();
        }
        vision_msgs::BoundingBox2DArray objArray;
        bool valid = detect_cargo(image_rect, objArray, debug_image, default_roi_);
        if (valid)
        {
            double x = 0, y = 0;
            if (find_with_color(objArray, color, z, x, y))
                ROS_INFO_STREAM("x:" << x << " y:" << y);
        }
        return valid;
    }

    bool ArmController::find_with_color(vision_msgs::BoundingBox2DArray &objArray, const int color, double z, double &x, double &y)
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

    bool ArmController::calculate_radius_and_speed(double u, double v, double center_u, double center_v, bool reset,
                                                   double &radius, double &speed)
    {
        static double center_x = 0, center_y = 0;
        double x, y;
        if (reset)
        {
            if (!ps_.calculate_cargo_position(center_u, center_v, current_z_, center_x, center_y))
                return false;
        }
        bool valid = ps_.calculate_cargo_position(u, v, current_z_, x, y);
        if (valid)
        {
            radius = sqrt((x - center_x) * (x - center_x) + (y - center_y) * (y - center_y));
            valid = tracker_.calculate_speed(x, y, center_x, center_y, speed_standard_, speed);
            // ROS_INFO_STREAM(radius);
        }
        return valid;
    }

    bool ArmController::get_ellipse_center(vision_msgs::BoundingBox2DArray &objArray,
                                           double &center_u, double &center_v)
    {
        if (objArray.boxes.size() == 4)
        {
            double u_sum = 0, v_sum = 0;
            for (size_t color = color_red; color <= color_blue; color++)
            {
                if (!objArray.boxes[color].center.x)
                    return false;
                u_sum += objArray.boxes[color].center.x;
                v_sum += objArray.boxes[color].center.y;
            }
            center_u = u_sum / 3;
            center_v = v_sum / 3;
            return true;
        }
        else
            return false;
    }

    bool ArmController::set_ellipse_color_order(vision_msgs::BoundingBox2DArray &objArray)
    {
        if (objArray.boxes.size() == 4)
        {
            for (size_t color = color_red; color <= color_blue; color++)
            {
                if (!objArray.boxes[color].center.x)
                {
                    return false;
                }
                ellipse_color_order_[color].center_x = objArray.boxes[color].center.x;
                ellipse_color_order_[color].color = color;
            }
            std::sort(ellipse_color_order_ + 1, ellipse_color_order_ + 3, [](EllipseColor e1, EllipseColor e2)
                      { return e1.center_x < e2.center_x; });
            return true;
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
        const int MAX_SIZE = 3;
        if (!cargo_x_.size())
        {
            current_color_ = color;
            current_z_ = z;
            ps_.reset();
        }
        else if (current_color_ != color || current_z_ != z || cargo_x_.size() >= MAX_SIZE)
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
                if (cargo_x_.size() == MAX_SIZE)
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
                        ps_.go_to(ps_.action_default.x, ps_.action_default.y, ps_.action_default.z,
                                  true, true);
                        // ps_.go_to_and_wait(ps_.put_x, ps_.put_y, ps_.put_z, false);
                    }
                    ps_.reset();
                    finish = true;
                }
            }
            else
                return false;
        }
        return valid;
    }

    // bool ArmController::catch_with_2_steps(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
    //                                        bool &finish, sensor_msgs::ImagePtr &debug_image)
    // {
    //     if (!cargo_x_.size())
    //     {
    //         current_color_ = color;
    //         current_z_ = z;
    //         ps_.reset();
    //     }
    //     else if (current_color_ != color || current_z_ != z || cargo_x_.size() >= 10)
    //     {
    //         cargo_x_.clear();
    //         cargo_y_.clear();
    //         current_color_ = color;
    //         current_z_ = z;
    //         ps_.reset();
    //     }
    //     finish = false;
    //     vision_msgs::BoundingBox2DArray objArray;
    //     bool valid = detect_cargo(image_rect, objArray, debug_image, default_roi_);
    //     if (valid)
    //     {
    //         double x = 0, y = 0;
    //         if (find_with_color(objArray, current_color_, z, x, y))
    //         {
    //             ROS_INFO_STREAM("x:" << x << " y:" << y);
    //             cargo_x_.push_back(x);
    //             cargo_y_.push_back(y);
    //             if (cargo_x_.size() == 5)
    //             {
    //                 double x_aver = 0, y_aver = 0;
    //                 average_position(x_aver, y_aver);
    //                 ps_.do_first_step(x_aver, y_aver);
    //             }
    //             else if (cargo_x_.size() == 10)
    //             {
    //                 double x_aver = 0, y_aver = 0;
    //                 average_position(x_aver, y_aver);
    //                 cargo_x_.clear();
    //                 cargo_y_.clear();
    //                 ps_.go_to_and_wait(x_aver, y_aver, current_z_, true);
    //                 finish = true;
    //             }
    //         }
    //         else
    //             return false;
    //     }
    //     return valid;
    // }

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

    bool ArmController::cargo_is_static(double speed, bool reset)
    {
        static int cnt = 0;
        if (reset)
        {
            cnt = 0;
            return false;
        }
        if (speed < 0) //-1
            return false;
        if (speed < speed_standard_)
        {
            cnt++;
            return cnt >= 3;
        }
        else
            cnt = 0;
        return false;
    }

    bool ArmController::track(const sensor_msgs::ImageConstPtr &image_rect, const int color, const int method,
                              double &u, double &v, bool &stop, sensor_msgs::ImagePtr &debug_image)
    {
        using namespace cv;
        static int cnt = 0;
        if (!fin_)
        {
            current_color_ = color;
            // // 丢弃前10张图
            // if (cnt < 10)
            // {
            //     cnt++;
            //     return true;
            // }
            // else
            // {
            //     cnt = 0;
            // }
        }
        else if (current_color_ != color)
        {
            ROS_WARN("Color changed!");
            fin_ = false;
            current_color_ = color;
        }
        const int INTERVAL = 50; // 间隔一段时间后重新进行目标检测
        const double PERMIT = 2.5;
        static double center_u = 0, center_v = 0, first_radius = 0;
        static std::vector<cv::Point> pt;
        double radius = 0, speed = -1;
        if (!fin_)
        {
            if (!emulation_)
                ps_.reset();
            vision_msgs::BoundingBox2DArray objArray;
            if (detect_cargo(image_rect, objArray, debug_image, default_roi_) &&
                get_ellipse_center(objArray, center_u, center_v) &&
                tracker_.target_init(cv_image_, objArray, color, method))
            {
                tracker_.get_center(u, v);
                if (!emulation_)
                {
                    calculate_radius_and_speed(u, v, center_u, center_v, true, radius, speed);
                    first_radius = radius;
                    cargo_is_static(speed, true);
                }
                fin_ = true;
                ROS_INFO("Succeeded to find cargo!");
                // ROS_INFO_STREAM("u:" << u << " v:" << v);
            }
            else
                return false;
        }
        else
        {
            cv_bridge::CvImagePtr cv_image;
            if ((++cnt) > INTERVAL) // 间隔一段时间后重新进行目标检测
            {
                // ROS_INFO("Target detect again...");
                vision_msgs::BoundingBox2DArray objArray;
                if (detect_cargo(image_rect, objArray, debug_image, default_roi_) &&
                    tracker_.target_init(cv_image_, objArray, color, method))
                {
                    tracker_.get_center(u, v);
                    if (!emulation_)
                    {
                        if (calculate_radius_and_speed(u, v, center_u, center_v, false, radius, speed) &&
                            abs(radius - first_radius) < PERMIT)
                        {
                            first_radius = radius;
                            cnt = 0;
                        }
                        else
                        {
                            cnt--;
                            return false;
                        }
                    }
                    else
                        cnt = 0;
                }
                else
                {
                    cnt--;
                    return false;
                }
            }
            else
            {
                if (!add_image(image_rect, cv_image))
                    return false;
                if (!tracker_.target_tracking(*cv_image))
                {
                    cnt = INTERVAL;
                    return false;
                }
                tracker_.get_center(u, v);
                if (!emulation_)
                {
                    if (!calculate_radius_and_speed(u, v, center_u, center_v, false, radius, speed))
                        return false;
                    if (abs(radius - first_radius) > PERMIT)
                        cnt = INTERVAL;
                }
                if (show_detections_ && !cv_image->image.empty())
                {
                    // 目标绘制
                    rectangle(cv_image->image, tracker_.rect_, Scalar(255, 0, 0), 3);
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
            pt.push_back(cv::Point(u, v));
            if (!emulation_)
            {

                ROS_INFO_STREAM("speed:" << speed);
                if (cargo_is_static(speed, false))
                {
                    ROS_INFO("static!");
                    stop = true;
                }
                else
                    stop = false;
            }
            else
                stop = false;
        }
        return true;
    }

    double ArmController::distance_min(vision_msgs::BoundingBox2DArray &objArray, const int color,
                                       double x, double y, double z)
    {
        double k = (ARM_P + y) / x;
        double dist_min = 0;
        for (int other_color = 1; other_color <= 3; other_color++)
        {
            if (other_color == color)
                continue;
            double ox = 0, oy = 0;
            bool valid = find_with_color(objArray, other_color, z, ox, oy);
            double dist = valid ? abs(ARM_P + oy - k * ox) / sqrt(1 + k * k) : 0;
            dist_min = dist_min == 0 ? dist : (dist < dist_min ? dist : dist_min);
        }
        return dist_min;
    }

    bool ArmController::find_points_with_height(double h, bool done, bool expand_y)
    {
        if (!plot_client_.exists())
        {
            plot_client_.waitForExistence(ros::Duration(1));
            return false;
        }
        ps_.expand_y = expand_y;
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

    bool ArmController::ellipse_target_find(const sensor_msgs::ImageConstPtr &image_rect,
                                            cv::Rect &roi, vision_msgs::BoundingBox2DArray &objArray,
                                            sensor_msgs::ImagePtr &debug_image)
    {
        cv_bridge::CvImagePtr cv_image;
        if (!add_image(image_rect, cv_image))
            return false;
        cv_image->image = cv_image->image(roi);
        using namespace cv;
        Mat srcdst, srcCopy;                        // 从相机传进来需要两张图片
        Point2d _center;                            // 椭圆中心
        std::vector<Point2d> centers;               // 椭圆中心容器
        std::vector<std::vector<Point2i>> contours; // 创建容器，存储轮廓
        std::vector<Vec4i> hierarchy;               // 寻找轮廓所需参数
        std::vector<RotatedRect> m_ellipses;        // 第一次初筛后椭圆容器
        EllipseArray arr;

        // 直线斜率处处相等原理的相关参数
        int line_Point[6] = {0, 0, 10, 20, 30, 40}; // 表示围成封闭轮廓点的序号，只要不太离谱即可
        const int line_threshold = 0.5;             // 判定阈值，小于即判定为直线

        // resize(cv_image->image, srcdst, cv_image->image.size()); // 重设大小，可选
        srcdst = cv_image->image.clone();
        srcCopy = srcdst.clone();

        // 第一次预处理
        GaussianBlur(srcdst, srcdst, Size(Gauss_size_, Gauss_size_), 0, 0);
        cvtColor(srcdst, srcdst, COLOR_BGR2GRAY);
        Canny(srcdst, srcdst, Canny_low_, Canny_up_, 3);
        // imshow("step1.", srcdst);//用于调试
        // ROI设置
        Mat mm = srcCopy(Rect(0, 0, srcCopy.cols, srcCopy.rows));
        mm = {Scalar(0, 0, 0)}; // 把ROI中的像素值改为黑色

        // 第一次轮廓查找
        findContours(srcdst, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

        // Mat imageContours = Mat::zeros(mm.size(), CV_8UC1);//创建轮廓展示图像，用于调试
        // 如果查找到了轮廓
        if (contours.size())
        {

            // // 轮廓展示，用于调试
            // for (int i = 0; i < contours.size(); i++)
            // {
            //     drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
            // }
            // imshow("Contours_1", imageContours);
            // 第一次排除
            for (std::vector<cv::Point> contour : contours)
            {
                // 初筛
                if (contourArea(contour) < con_Area_min_ ||
                    contour.size() < con_Point_cont_ || contourArea(contour) > con_Area_max_)
                    continue;
                // 利用直线斜率处处相等的原理
                Point2d pt[6];
                for (int i = 1; i < 5; i++)
                {
                    if (contours.size() - 1 < line_Point[5])
                    {
                        pt[i] = contour[(int)floor(contours.size() / 4.0) * (i - 1)];
                    }
                    else
                        pt[i] = contour[line_Point[i]];
                }
                if (abs(((pt[3].y - pt[1].y) * 1.0 / (pt[3].x - pt[1].x) -
                         (pt[2].y - pt[1].y) * 1.0 / (pt[2].x - pt[2].x))) < line_threshold)
                    continue;
                if (abs(((pt[5].y - pt[3].y) * 1.0 / (pt[5].x - pt[3].x) -
                         (pt[4].y - pt[3].y) * 1.0 / (pt[4].x - pt[3].x))) < line_threshold)
                    continue;
                // // 利用凹凸性的原理
                // if (!abs((contours[i][0].y + contours[i][20].y) / 2 - contours[i][10].y))
                //     continue;
                // drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
                RotatedRect m_ellipsetemp;           // 创建接收椭圆的容器
                m_ellipsetemp = fitEllipse(contour); // 找到的第一个轮廓，放置到m_ellipsetemp
                if (m_ellipsetemp.size.width / m_ellipsetemp.size.height < 0.2 ||
                    m_ellipsetemp.size.height / m_ellipsetemp.size.width < 0.2)
                    continue;
                ellipse(mm, m_ellipsetemp, cv::Scalar(255, 255, 255)); // 在图像中绘制椭圆，必要
                _center = m_ellipsetemp.center;                        // 读取椭圆中心，必要
                // drawCross(srcCopy, _center, Scalar(255, 0, 0), 30, 2);//绘制中心十字，用于调试
                // circle(cv_image->image, _center, 1, Scalar(0, 255, 0), -1); // 画半径为1的圆(画点）, 用于调试
                centers.push_back(_center);
                m_ellipses.push_back(m_ellipsetemp);
            }
            // imshow("Contours_1", imageContours);
            // imshow("mm", mm); // 显示第一次排除结果，用于调试
            // cv::waitKey(10);
            if (!arr.clustering(centers, m_ellipses))
                return false;
            if (!arr.generate_bounding_rect(m_ellipses, cv_image))
                return false;
            // 颜色标定
            if (!arr.color_classification(cv_image, white_vmin_))
                return false;
            objArray.header = image_rect->header;
            if (!arr.detection(objArray, roi, cv_image, show_detections_))
                return false;
        }
        if (show_detections_ && !cv_image->image.empty())
            debug_image = cv_image->toImageMsg();
        return true;
    }

    bool ArmController::put_with_ellipse(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
                                         bool &finish, sensor_msgs::ImagePtr &debug_image)
    {
        if (!cargo_x_.size())
        {
            current_color_ = color;
            current_z_ = z;
            if (!emulation_)
            {
                ps_.reset();
            }
        }
        else if (current_color_ != color || current_z_ != z || cargo_x_.size() >= 10)
        {
            cargo_x_.clear();
            cargo_y_.clear();
            if (!emulation_)
                ps_.reset();
            current_color_ = color;
            current_z_ = z;
        }
        finish = false;
        vision_msgs::BoundingBox2DArray objArray;
        bool valid = ellipse_target_find(image_rect, default_roi_, objArray, debug_image);
        if (valid)
        {
            double x = 0, y = 0;
            if (emulation_)
            {
                if (!cargo_x_.size())
                    cargo_x_.push_back(x);
                return valid;
            }
            if (find_with_color(objArray, color, current_z_, x, y))
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
                    valid = ps_.go_to_and_wait(x_aver, y_aver, current_z_, false);
                    ps_.reset();
                }
            }
        }
        return valid;
    }

    Angle hue_value(double h_val)
    {
        return Angle(h_val * 2);
    }

    double hue_value_tan(double y, double x)
    {
        return Angle(y, x)._get_degree() / 2;
    }

    double hue_value_diff(double h_val1, double h_val2)
    {
        Angle a1 = hue_value(h_val1);
        Angle a2 = hue_value(h_val2);
        double res = abs((a1 - a2)._get_degree());
        if (res > 180)
        {
            res = 360.0 - res;
        }
        return res / 2;
    }

    void draw_cross(cv::Mat &img, cv::Point2d point, cv::Scalar color, int size, int thickness)
    {
        if (!point.inside(cv::Rect2d(0, 0, img.cols, img.rows)))
            ROS_WARN("Point is out of range! point:(%lf, %lf)", point.x, point.y);
        // 绘制横线
        line(img, cv::Point(point.x - size / 2, point.y),
             cv::Point(point.x + size / 2, point.y), color, thickness, 8, 0);
        // 绘制竖线
        line(img, cv::Point(point.x, point.y - size / 2),
             cv::Point(point.x, point.y + size / 2), color, thickness, 8, 0);
    }

    bool EllipseArray::generate_bounding_rect(std::vector<cv::RotatedRect> &m_ellipses,
                                              cv_bridge::CvImagePtr &cv_image)
    {
        if (m_ellipses.empty() || ellipse_.empty())
            return false;
        bool note[flag_.size()] = {false};
        // for (int i = 0; i < m_ellipses.size(); i++)
        //     ROS_INFO_STREAM(flag_[i]);
        int num = 0;
        for (int i = 0; i < m_ellipses.size(); i++)
        {
            if (note[i])
                continue;
            int now = i, area_max = (m_ellipses[now].boundingRect()).area(), cnt = 1, ind_max = i;
            // ROS_INFO_STREAM(now);
            note[now] = true;
            while (flag_[now] != -1)
            {
                cnt++;
                now = flag_[now];
                note[now] = true;
                int area_temp = (m_ellipses[now].boundingRect()).area();
                area_max = (area_max > area_temp) ? area_max : area_temp;
                ind_max = (area_max > area_temp) ? ind_max : now;
            }
            if (cnt > 2)
            {
                cv::Rect rect = m_ellipses[ind_max].boundingRect();
                rect.x = std::max<int>(rect.x, 0);
                rect.y = std::max<int>(rect.y, 0);
                rect.width = std::min<int>(rect.width, cv_image->image.cols - rect.x);
                rect.height = std::min<int>(rect.height, cv_image->image.rows - rect.y);
                try
                {
                    ellipse_.at(num).rect_target = rect;
                }
                catch (const std::exception &e)
                {
                    ROS_ERROR("generate bounding rect error: %s", e.what());
                    return false;
                }
                num++;
            }
        }
        if (num != ellipse_.size())
        {
            ROS_ERROR("Incorrect num! num:%d, ellipse size:%ld", num, ellipse_.size());
            return false;
        }
        return true;
    }

    double color_hypothesis(double h_val, int lower_bound, int upper_bound)
    {
        double lower_diff = hue_value_diff(h_val, lower_bound);
        double upper_diff = hue_value_diff(h_val, upper_bound);
        double diff = hue_value_diff(upper_bound, lower_bound);
        if (abs(lower_diff + upper_diff - diff) < 1)
        {
            return 1;
        }
        else if (abs(lower_diff + upper_diff + diff - 180) < 1 ||
                 abs(std::min<double>(lower_diff, upper_diff) + diff -
                     std::max<double>(lower_diff, upper_diff)) < 1)
        {
            return exp(-std::min<double>(lower_diff, upper_diff) / diff);
        }
        else
        {
            ROS_ERROR("Diff error! diff:%lf lower_diff:%lf upper_diff:%lf", diff, lower_diff, upper_diff);
            ROS_ERROR("h_val:%lf lower_bound:%d upper_bound:%d", h_val, lower_bound, upper_bound);
            return 0;
        }
    }

    bool EllipseArray::color_classification(cv_bridge::CvImagePtr &cv_image,
                                            int white_vmin)
    {
        if (ellipse_.empty() || cv_image->image.empty())
            return false;
        const int WHITE_SMAX = 46;
        const int WHITE_VMIN = white_vmin;
        const int RED_HMIN = 10;
        const int RED_HMAX = 156;
        const int GREEN_HMIN = 35;
        const int GREEN_HMAX = 77;
        const int BLUE_HMIN = 100;
        const int BLUE_HMAX = 124;
        int upper_bound[] = {0, RED_HMAX, GREEN_HMAX, BLUE_HMAX};
        int lower_bound[] = {0, RED_HMIN, GREEN_HMIN, BLUE_HMIN};
        for (Ellipse &e : ellipse_)
        {
            if (e.rect_target.empty())
            {
                ROS_ERROR("rect target is empty!");
                return false;
            }
            cv::Mat mask = (cv_image->image(e.rect_target)).clone();
            cv::Mat mask_Img = mask.clone();        // 用于调试
            mask_Img = {cv::Scalar(255, 255, 255)}; // 用于调试
            cvtColor(mask, mask, cv::COLOR_BGR2HSV);
            // 设置像素遍历迭代器
            cv::MatConstIterator_<cv::Vec3b> maskStart = mask.begin<cv::Vec3b>();
            cv::MatConstIterator_<cv::Vec3b> maskEnd = mask.end<cv::Vec3b>();
            cv::MatIterator_<cv::Vec3b> mask_ImgStart = mask_Img.begin<cv::Vec3b>(); // 用于调试
            double x = 0, y = 0;
            int cnt = 0;
            for (; maskStart != maskEnd; maskStart++, mask_ImgStart++)
            {
                // 过滤白色
                if ((*maskStart)[1] <= WHITE_SMAX && (*maskStart)[2] >= WHITE_VMIN)
                {
                    // 用于调试
                    (*mask_ImgStart)[0] = 0;
                    (*mask_ImgStart)[1] = 0;
                    (*mask_ImgStart)[2] = 0;
                    continue;
                }
                int H_Val = (*maskStart)[0];
                Angle color = hue_value(H_Val);
                x += color.cos();
                y += color.sin();
                cnt++;
                // if (cnt % 200 == 0)
                // {
                //     ROS_INFO_STREAM(H_Val);
                //     usleep(1e5);
                // }
            }
            // cv::imshow("mask_Img", mask_Img); // 用于调试
            // cv::waitKey(10);
            double H_Average = hue_value_tan(y, x); // 保存当前区域色相H的平均值
            H_Average = (H_Average < 0) ? H_Average + 180 : H_Average;
            // ROS_INFO_STREAM("H_Average:" << H_Average << " cnt:" << cnt << " " << mask.cols * mask.rows);
            int id = color_red;
            double max_hyp = 0;
            for (int color = color_red; color <= color_blue; color++)
            {
                if (color_hypothesis(H_Average, lower_bound[color], upper_bound[color]) > max_hyp)
                {
                    id = color;
                    max_hyp = color_hypothesis(H_Average, lower_bound[color], upper_bound[color]);
                }
            }
            e.color = id;
            e.hypothesis = max_hyp;
        }
        return true;
    }

    bool EllipseArray::detection(vision_msgs::BoundingBox2DArray &objArray,
                                 cv::Rect &roi, cv_bridge::CvImagePtr &cv_image, bool show_detection)
    {
        if (ellipse_.empty())
            return false;
        if (!objArray.boxes.empty())
            objArray.boxes.clear();
        objArray.boxes.resize(4);
        double hyp_max[4] = {0};
        for (Ellipse &e : ellipse_)
        {
            // ROS_INFO_STREAM(e.hypothesis << hyp_max[e.color]);
            if (e.hypothesis > hyp_max[e.color])
            {
                vision_msgs::BoundingBox2D obj = vision_msgs::BoundingBox2D();
                obj.center.x = e.center.x + roi.x;
                obj.center.y = e.center.y + roi.y;
                obj.size_x = e.rect_target.width;
                obj.size_y = e.rect_target.height;
                objArray.boxes[e.color] = obj;
                hyp_max[e.color] = e.hypothesis;
                if (show_detection)
                {
                    // 绘制中心十字，用于调试
                    cv::Scalar c;
                    switch (e.color)
                    {
                    case color_red:
                        c = cv::Scalar(0, 0, 255);
                        break;

                    case color_green:
                        c = cv::Scalar(0, 255, 0);
                        break;

                    case color_blue:
                        c = cv::Scalar(255, 0, 0);
                        break;

                    default:
                        break;
                    }
                    draw_cross(cv_image->image,
                               e.center,
                               c, 30, 2);
                    // imshow("srcCopy", cv_image->image); // 用于调试
                    // cv::waitKey(60);
                }
            }
        }
        return true;
    };
} // namespace my_hand_eye
