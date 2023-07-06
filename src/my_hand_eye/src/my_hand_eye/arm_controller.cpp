#include "my_hand_eye/arm_controller.h"

namespace my_hand_eye
{
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
        XmlRpc::XmlRpcValue back_action;
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
        ps_.set_action(back_action, "back");
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

    bool ArmController::log_position(const sensor_msgs::ImageConstPtr &image_rect, double z, int color,
                                     sensor_msgs::ImagePtr &debug_image, bool center)
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
            if (center)
            {
                double center_u = 0, center_v = 0;
                if (get_center(objArray, center_u, center_v, x, y))
                {
                    ROS_INFO_STREAM("x:" << x << " y:" << y);
                    if (show_detections_ && !cv_image_.image.empty())
                    {
                        draw_cross(cv_image_.image, cv::Point(center_u, center_v), cv::Scalar(255, 255, 255), 30, 3);
                        debug_image = cv_image_.toImageMsg();
                    }
                }
            }
            else if (find_with_color(objArray, color, z, x, y))
                ROS_INFO_STREAM("x:" << x << " y:" << y);
        }
        return valid;
    }

    bool ArmController::log_extrinsics_correction(const sensor_msgs::ImageConstPtr &image_rect,
                                                  double correct_x, double correct_y, double correct_z, int color,
                                                  sensor_msgs::ImagePtr &debug_image)
    {
        // OpenCV(4.1.1) /home/nvidia/host/build_opencv/nv_opencv/modules/core/src/arithm.cpp:663: error: (-209:Sizes of input arguments do not match) The operation is neither 'array op array' (where arrays have the same size and the same number of channels), nor 'array op scalar', nor 'scalar op array' in function 'arithm_op'
        static bool flag = false;
        // 尚未初始化位姿
        if (!flag)
        {
            flag = true;
            ps_.reset();
        }
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

    bool ArmController::calculate_radius_and_speed(double &u, double &v, double &radius, double &speed)
    {
        tracker_.get_center(u, v);
        double x, y;
        bool valid = ps_.calculate_cargo_position(u, v, z_turntable, x, y);
        if (valid)
        {
            radius = sqrt((x - tracker_.center_x_) * (x - tracker_.center_x_) + (y - tracker_.center_y_) * (y - tracker_.center_y_));
            valid = tracker_.calculate_speed(x, y, speed_standard_, speed);
            // ROS_INFO_STREAM(radius);
        }
        return valid;
    }

    bool ArmController::get_center(vision_msgs::BoundingBox2DArray &objArray,
                                   double &center_u, double &center_v, double &center_x, double &center_y)
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
            return ps_.calculate_cargo_position(center_u, center_v, z_turntable, center_x, center_y);
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
                                         bool &finish, sensor_msgs::ImagePtr &debug_image,
                                         bool left, bool hold, bool midpoint)
    {
        const int MAX_SIZE = 5;
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
                    if (!hold)
                    {
                        if (valid)
                        {
                            ps_.go_to_table(false, color, left);
                        }
                        ps_.reset(left);
                    }
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

    bool ArmController::track(const sensor_msgs::ImageConstPtr &image_rect, const int color,
                              double &u, double &v, bool &stop, sensor_msgs::ImagePtr &debug_image)
    {
        using namespace cv;
        static int cnt = 0;
        if (!fin_)
        {
            current_color_ = color;
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
            ps_.reset();
            vision_msgs::BoundingBox2DArray objArray;
            double center_x, center_y;
            if (detect_cargo(image_rect, objArray, debug_image, default_roi_) &&
                get_center(objArray, center_u, center_v, center_x, center_y) &&
                tracker_.target_init(cv_image_, objArray, color, white_vmin_,
                                     center_x, center_y, proportion_))
            {
                calculate_radius_and_speed(radius, speed, u, v);
                first_radius = radius;
                cargo_is_static(speed, true);
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
                ROS_INFO("Target detect again...");
                vision_msgs::BoundingBox2DArray objArray;
                double center_x, center_y;
                if (detect_cargo(image_rect, objArray, debug_image, default_roi_) &&
                    get_center(objArray, center_u, center_v, center_x, center_y) &&
                    tracker_.target_init(cv_image_, objArray, color, white_vmin_,
                                         center_x, center_y, proportion_))
                {
                    if (calculate_radius_and_speed(radius, speed, u, v) &&
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
                {
                    cnt--;
                    return false;
                }
            }
            else
            {
                if (!add_image(image_rect, cv_image))
                    return false;
                if (!tracker_.target_track(*cv_image, ps_, z_turntable))
                {
                    cnt = INTERVAL;
                    return false;
                }
                if (!calculate_radius_and_speed(radius, speed, u, v))
                    return false;
                if (abs(radius - first_radius) > PERMIT)
                    cnt = INTERVAL;
                if (show_detections_ && !cv_image->image.empty())
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
            pt.push_back(cv::Point(u, v));
            ROS_INFO_STREAM("speed:" << speed);
            if (cargo_is_static(speed, false))
            {
                ROS_INFO("Cargo is static!");
                stop = true;
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

    bool ArmController::find_border(const sensor_msgs::ImageConstPtr &image_rect, double &distance, double &yaw,
                                    bool &finish, sensor_msgs::ImagePtr &debug_image)
    {
        cv_bridge::CvImagePtr cv_image;
        if (!add_image(image_rect, cv_image))
            return false;
        cv::Vec2f border;
        return border_.find(cv_image, border, LBD_color_func, show_detections_);
    }

    bool ArmController::find_center(const sensor_msgs::ImageConstPtr &image_rect, double &x, double &y,
                                    bool &finish, sensor_msgs::ImagePtr &debug_image)
    {
        static bool last_finish = true;
        if (!finish && last_finish)
            ps_.reset();
        last_finish = finish;
        vision_msgs::BoundingBox2DArray objArray;
        double center_u, center_v;
        bool valid = detect_cargo(image_rect, objArray, debug_image, default_roi_) &&
                     get_center(objArray, center_u, center_v, x, y);
        if (valid && show_detections_ && !cv_image_.image.empty())
        {
            draw_cross(cv_image_.image, cv::Point(center_u, center_v), cv::Scalar(255, 255, 255), 30, 3);
            debug_image = cv_image_.toImageMsg();
        }
        return valid;
    }
} // namespace my_hand_eye
