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
        if (!emulation_)
            ps_.end();
    }

    void ArmController::init(ros::NodeHandle nh, ros::NodeHandle pnh, bool emulation)
    {
        emulation_ = emulation;
        if (emulation_)
        {
            plot_client_ = nh.serviceClient<my_hand_eye::Plot>("height_plot");
            return;
        }
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

    bool ArmController::add_image(const sensor_msgs::ImageConstPtr &image_rect, cv_bridge::CvImagePtr &image)
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

    bool ArmController::take_picture()
    {
        if (cv_image_.image.empty())
        {
            ROS_WARN("Empty image!");
            return false;
        }
        char str[50];
        static int n = 0;
        snprintf(str, sizeof(str), "/home/fu/apriltag_ws/src/my_hand_eye/img/%d.jpg", ++n);
        cv::imwrite(str, cv_image_.image);
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
            if (find_with_color(objArray, color_green, z, x, y))
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
                        ps_.go_to_and_wait(ps_.put_x, ps_.put_y, ps_.put_z, false);
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

    bool ArmController::calculate_speed(double u, double v, double &speed)
    {
        if (!last_time_.is_zero() && !cv_image_.header.stamp.is_zero() && pt_.size())
        {
            std::vector<cv::Point>::const_iterator cit;
            cit = pt_.cend() - 1;
            double du = u - cit->x;
            double dv = v - cit->y;
            ros::Duration dr = cv_image_.header.stamp - last_time_;
            double dt = dr.toSec();
            speed = sqrt(du * du + dv * dv) / dt;
            last_time_ = cv_image_.header.stamp;
            pt_.push_back(cv::Point(u, v));
            return true;
        }
        else if (cv_image_.header.stamp.is_zero())
        {
            ROS_WARN("Stamp is zero!");
            return false;
        }
        else
        {
            last_time_ = cv_image_.header.stamp;
            pt_.push_back(cv::Point(u, v));
            return false;
        }
    }

    bool ArmController::target_tracking(const sensor_msgs::ImageConstPtr &image_rect, const int color,
                                        double &u, double &v, bool &stop, sensor_msgs::ImagePtr &debug_image)
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
        static int cnt = 0;
        if (!fin_)
        {
            ps_.reset();
            vision_msgs::BoundingBox2DArray objArray;
            if (detect_cargo(image_rect, objArray, debug_image, default_roi_) &&
                target_init(objArray, color, dstHist))
            {
                fin_ = true;
                if (pt_.size())
                    pt_.clear();
                u = rect_.x + rect_.width / 2;
                v = rect_.y + rect_.height / 2;
                // ROS_INFO_STREAM("u:" << u << " v:" << v);
                double speed = 100;
                calculate_speed(u, v, speed);
            }
            else
                return false;
        }
        else
        {            
            if ((++cnt) > 10)//间隔一段时间后重新进行目标检测
            {
                vision_msgs::BoundingBox2DArray objArray;
                if (detect_cargo(image_rect, objArray, debug_image, default_roi_) &&
                    target_init(objArray, color, dstHist))
                {
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
            u = rect_.x + rect_.width / 2;
            v = rect_.y + rect_.height / 2;
            // ROS_INFO_STREAM("u:" << u << " v:" << v);
            double speed = 100;
            if (calculate_speed(u, v, speed))
                ROS_INFO_STREAM("speed:" << speed);
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
            plot_client_.waitForExistence();
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
                                            sensor_msgs::ImagePtr &debug_image, cv::Rect &roi)
    {
        cv_bridge::CvImagePtr cv_image;
        if (!add_image(image_rect, cv_image))
            return false;
        cv_image->image = cv_image->image(roi);
        using namespace cv;
        Mat srcdst, srcCopy;          // 从相机传进来需要两张图片
        Point2d _center;              // 椭圆中心
        std::vector<Point2d> centers; // 椭圆中心容器
        std::vector<Point2d> center;  // 目标椭圆容器
        const int Gauss_size = 3;     // 高斯平滑内核大小
        const int Canny_low = 50;     // 第一次Canny边缘查找的第一滞后因子
        const int Canny_up = 100;     // 第一次Canny边缘查找的第二滞后因子

        const int con_Area_min = 4500;   // 粗筛-最小面积阈值
        const int con_Point_cont = 20;   // 粗筛-图形最少点个数阈值，即连成某个封闭轮廓的点的个数，少于该阈值表明轮廓无效
        const int con_Area_max = 200000; // 粗筛-最大面积阈值

        // 直线斜率处处相等原理的相关参数
        int line_Point[6] = {0, 0, 10, 20, 30, 40}; // 表示围成封闭轮廓点的序号，只要不太离谱即可
        const int line_threshold = 0.5;             // 判定阈值，小于即判定为直线

        resize(cv_image->image, srcdst, cv_image->image.size()); // 重设大小，可选
        srcCopy = srcdst.clone();

        // 第一次预处理
        GaussianBlur(srcdst, srcdst, Size(Gauss_size, Gauss_size), 0, 0);
        cvtColor(srcdst, srcdst, COLOR_BGR2GRAY);
        Canny(srcdst, srcdst, Canny_low, Canny_up, 3);
        // imshow("step1.", srcdst);//用于调试
        // ROI设置
        Mat mm = srcCopy(Rect(0, 0, srcCopy.cols, srcCopy.rows));
        mm = {Scalar(0, 0, 0)}; // 把ROI中的像素值改为黑色

        // 第一次轮廓查找
        std::vector<std::vector<Point2i>> contours; // 创建容器，存储轮廓
        std::vector<Vec4i> hierarchy;               // 寻找轮廓所需参数
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
            for (int i = 0; i < contours.size(); i++)
            {
                // 初筛
                if (contourArea(contours[i]) < con_Area_min ||
                    contours[i].size() < con_Point_cont || contourArea(contours[i]) > con_Area_max)
                    continue;
                // 利用直线斜率处处相等的原理
                Point2d pt[6];
                for (int j = 1; j < 5; j++)
                {
                    if (contours.size() - 1 < line_Point[5])
                    {
                        pt[j] = contours[j][(int)floor(contours.size() / 4.0) * (j - 1)];
                    }
                    else
                        pt[j] = contours[j][line_Point[j]];
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
                RotatedRect m_ellipsetemp;               // 创建接收椭圆的容器
                m_ellipsetemp = fitEllipse(contours[i]); // 找到的第一个轮廓，放置到m_ellipsetemp
                if (m_ellipsetemp.size.width / m_ellipsetemp.size.height < 0.2)
                    continue;
                ellipse(mm, m_ellipsetemp, cv::Scalar(255, 255, 255)); // 在图像中绘制椭圆，必要
                _center = m_ellipsetemp.center;                        // 读取椭圆中心，必要
                // drawCross(srcCopy, _center, Scalar(255, 0, 0), 30, 2);//绘制中心十字，用于调试
                circle(cv_image->image, _center, 1, Scalar(0, 255, 0), -1); // 画半径为1的圆(画点）, 用于调试
                centers.push_back(_center);
            }
            // imshow("Contours_1", imageContours);
            imshow("mm", mm); // 显示第一次排除结果，用于调试
            // // 第二次预处理
            // Mat mmdst;
            // GaussianBlur(mm, mmdst, Size(Gauss_size, Gauss_size), 0, 0);
            // cvtColor(mmdst, mmdst, COLOR_BGR2GRAY);
            // Canny(mmdst, mmdst, 50, 150, 3);
            // imshow("step2.", mmdst); // 用于调试
            // // 第二次轮廓查找
            // std::vector<std::vector<Point2i>> contours1; // 创建容器，存储轮廓
            // std::vector<Vec4i> hierarchy1;               // 寻找轮廓所需参数
            // findContours(mmdst, contours1, hierarchy1, RETR_CCOMP, CHAIN_APPROX_NONE);
            // Mat imageContours1 = Mat::zeros(mmdst.size(), CV_8UC1);//创建轮廓展示图像，用于调试
            // 如果查找到了轮廓
            // if (contours1.size())
            // {
            //     // 第二次筛除
            //     for (int i = 0; i < contours1.size(); i++)
            //     {
            //         // 初筛
            //         if (contours1[i].size() < 10 || contours1[i].size() > 1000)
            //             continue;
            //         // 利用凹凸性的原理
            //         // if (!abs((contours1[i][0].y + contours1[i][20].y) / 2 - contours1[i][10].y))
            //         //	continue;
            //         if (contourArea(contours1[i]) < 4500 || contourArea(contours1[i]) > 180000)
            //             continue;
            //         // drawContours(imageContours1, contours1, i, Scalar(255), 1, 8);//用于调试

            //         RotatedRect m_ellipsetemp1;
            //         m_ellipsetemp1 = fitEllipse(contours1[i]);
            //         ellipse(cv_image->image, m_ellipsetemp1, cv::Scalar(255, 0, 0)); // 绘制椭圆，用于调试
            //         _center = m_ellipsetemp1.center;                        // 读取椭圆中心，必要
            //         // drawCross(srcCopy, _center, Scalar(255, 0, 0), 30, 2);//绘制中心十字，用于调试
            //         circle(cv_image->image, _center, 1, Scalar(0, 255, 0), -1); // 画半径为1的圆(画点）, 用于调试
            //         centers.push_back(_center);
            //     }
            // imshow("Contours_2", imageContours1);//用于调试
            // cout << centers.size() << endl;
            const int MAXN = centers.size();
            bool flag[MAXN] = {0};
            // 聚类
            for (int i = 0; i < centers.size() - 1; i++)
            {
                flag[i] = true;
                int x_temp = centers[i].x, y_temp = centers[i].y, count = 1;
                for (int j = i + 1; j < centers.size(); j++)
                {
                    if (abs(centers[i].x - centers[j].x) < 10 && abs(centers[i].y - centers[j].y) < 10)
                    {
                        if (!flag[j])
                        {
                            flag[j] = true;
                            x_temp = x_temp + centers[j].x;
                            y_temp = y_temp + centers[j].y;
                            count++;
                        }
                        else
                            continue;
                    }
                }
                if (count > 2)
                {
                    // 平均数求聚类中心，感觉不太妥当，但是精度感觉还行，追求精度的话可以用 Weiszfeld 算法求中位中心，那个要迭代
                    center.push_back(cv::Point(x_temp / count, y_temp / count));
                }
            }
            ROS_INFO_STREAM("The number of ellipse is " << center.size());
            // 绘制中心十字，用于调试
            for (size_t i = 0; i < center.size(); i++)
            {
                draw_cross(cv_image->image, center[i], Scalar(0, 0, 255), 30, 2);
            }
        }
        // 颜色标定
        // 目前就差这一步了
        imshow("srcCopy", cv_image->image); // 用于调试
        cv::waitKey(60);
        if (show_detections_)
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
        if (!cargo_x_.size())
            cargo_x_.push_back(0);
        ellipse_target_find(image_rect, debug_image, default_roi_);
        return true;
    }

    void draw_cross(cv::Mat &img, cv::Point2f point, cv::Scalar color, int size, int thickness)
    {
        // 绘制横线
        line(img, cv::Point(point.x - size / 2, point.y),
             cv::Point(point.x + size / 2, point.y), color, thickness, 8, 0);
        // 绘制竖线
        line(img, cv::Point(point.x, point.y - size / 2),
             cv::Point(point.x, point.y + size / 2), color, thickness, 8, 0);
    }

} // namespace my_hand_eye
