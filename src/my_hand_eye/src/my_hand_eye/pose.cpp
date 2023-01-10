#include "my_hand_eye/pose.hpp"

namespace my_hand_eye
{
    ArmPose::ArmPose() : empty(true){};

    Pos::Pos(SMS_STS *sm_st_ptr, SCSCL *sc_ptr, bool cat, bool look)
    {
        this->cat = cat;
        this->look = look;
        this->sm_st_ptr = sm_st_ptr;
        this->sc_ptr = sc_ptr;
        x = 0;
        y = 0;
        z = 0;
    }

    bool Pos::begin(const char *argv)
    {
        if (!(sm_st_ptr->begin(115200, argv)) || !(sc_ptr->begin(115200, argv)))
        {
            ROS_ERROR("Failed to init motor!");
            return false;
        }
        else
            return true;
    }

    void Pos::set_speed_and_acc(XmlRpc::XmlRpcValue &servo_descriptions)
    {
        // Ensure the type is correct
        ROS_ASSERT(servo_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
        // Loop through all servo descriptions
        for (int32_t i = 0; i < servo_descriptions.size(); i++)
        {

            // i-th servo description
            XmlRpc::XmlRpcValue &servo_description = servo_descriptions[i];

            // Assert the servo description is a struct
            ROS_ASSERT(servo_description.getType() ==
                       XmlRpc::XmlRpcValue::TypeStruct);
            // Assert type of field "id" is an int
            ROS_ASSERT(servo_description["id"].getType() ==
                       XmlRpc::XmlRpcValue::TypeInt);
            // Assert type of field "speed" is a int
            ROS_ASSERT(servo_description["speed"].getType() ==
                       XmlRpc::XmlRpcValue::TypeInt);
            // Assert type of field "acc" is a int
            ROS_ASSERT(servo_description["acc"].getType() ==
                       XmlRpc::XmlRpcValue::TypeInt);

            int id = (int)servo_description["id"]; // tag id
            Speed[id] = (int)servo_description["speed"];
            ACC[id] = (int)servo_description["acc"];
            ROS_INFO_STREAM("Loaded servo desciptions id: " << id << ", speed: " << Speed[id] << ", acc: " << (int)ACC[id]);
            // 此处应注意
        }
    }

    void Pos::set_default_position(XmlRpc::XmlRpcValue &default_action)
    {
        ROS_ASSERT(default_action.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(default_action[0].getType() == XmlRpc::XmlRpcValue::TypeInt);
        ROS_ASSERT(default_action[1].getType() == XmlRpc::XmlRpcValue::TypeInt);
        ROS_ASSERT(default_action[2].getType() == XmlRpc::XmlRpcValue::TypeInt);
        default_x = (int)default_action[0];
        default_y = (int)default_action[1];
        default_z = (int)default_action[2];
    }

    bool Pos::calculate_position()
    {
        double deg1, deg2, deg3, deg4;
        deg1 = deg2 = deg3 = deg4 = 0;
        bool valid = test_ok(deg1, deg2, deg3, deg4, look);
        if (valid)
        {
            Position[1] = std::round(ARM_JOINT1_POS_WHEN_DEG0 + (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * deg1 / 180);
            Position[2] = std::round(ARM_JOINT234_POS_WHEN_DEG0 + (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * deg2 / 180);
            Position[3] = std::round(ARM_JOINT234_POS_WHEN_DEG0 + (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * deg3 / 180);
            Position[4] = std::round(ARM_JOINT234_POS_WHEN_DEG0 + (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * deg4 / 180);
            Position[5] = std::round(cat ? ARM_JOINT5_POS_WHEN_CATCH : ARM_JOINT5_POS_WHEN_LOSEN);
            // ROS_INFO_STREAM("position (from 1 to 5):" << Position[1] << " " << Position[2] << " " << Position[3] << " " << Position[4] << " " << Position[5]);
        }
        return valid;
    }

    bool Pos::go_to(double x, double y, double z, bool cat, bool look)
    {
        double time_max = 0;
        bool valid = read_all_position();
        s16 tmp[6] = {0};
        if (valid)
        {
            memcpy(tmp, Position, 6 * sizeof(s16));
        }
        this->x = x;
        this->y = y;
        this->z = z;
        this->cat = cat;
        this->look = look;
        if (valid)
            valid = calculate_position();
        if (valid)
        {
            for (int ID = 1; ID <= 5; ID++)
            {
                double time = calculate_time(ID, tmp[ID]);
                time_max = time > time_max ? time : time_max;
            }
            ROS_INFO("Done! Wait for %lf seconds", time_max);
            ros::Time now = ros::Time::now();
            ros::Duration du(time_max); // 以秒为单位
            ros::Time after_now = now + du;
            wait_time_ = after_now.toSec();
            sc_ptr->WritePos(1, (u16)Position[1], 0, Speed[1]);
            sm_st_ptr->SyncWritePosEx(Id + 2, 3, Position + 2, Speed + 2, ACC + 2);
            sc_ptr->WritePos(5, (u16)Position[5], 0, Speed[5]);
        }
        return valid;
    }

    double Pos::calculate_time(int ID, s16 goal)
    {
        // 时间（单位s）=[(位置-目标)/速度]+(速度/(加速度*100))
        double time = 10.0;
        if (ID == 5)
            time =  abs((Position[ID] - goal) * 1.0 / (Speed[ID] + 0.01));
        else if (ID == 3 || ID == 4)
            time =  abs((Position[ID] - goal) * 1.0 / (Speed[ID] + 0.01)) + Speed[ID] / (100.0 * ACC[ID] + 0.01);
        else if (ID == 1 || ID == 2)
            time = 0.8;
        else
            ROS_ERROR("ID error!");
        time = time > 10.0 ? 10.0 : time;
        ROS_INFO_STREAM("ID:" << ID << " time:" << time);
        return time;
    }

    bool Pos::do_first_step(double x, double y)
    {
        this->x = x;
        this->y = y;
        double deg1 = 0;
        bool valid = first_step(deg1);
        if (valid)
        {
            s16 goal = std::round(ARM_JOINT1_POS_WHEN_DEG0 + (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * deg1 / 180);
            double time = 10;
            if (read_position(1))
                time = calculate_time(1, goal);
            else
            {
                valid = false;
            }
            ROS_INFO("Done! Wait for %lf seconds", time);
            ros::Time now = ros::Time::now();
            ros::Duration du(time); // 以秒为单位
            ros::Time after_now = now + du;
            wait_time_ = after_now.toSec();
            sc_ptr->WritePos(1, (u16)Position[1], 0, Speed[1]);
        }
        return valid;
    }

    bool Pos::reset()
    {
        return go_to(default_x, default_y, default_z, false, true);
    }

    bool Pos::go_to_and_wait(double x, double y, double z, bool cat)
    {
        double time_max = 0;
        bool valid = read_all_position();
        s16 tmp[6] = {0};
        if (valid)
        {
            memcpy(tmp, Position, 6 * sizeof(s16));
        }
        this->x = x;
        this->y = y;
        this->z = z;
        this->cat = cat;
        this->look = false;
        if (valid)
            valid = calculate_position();
        if (valid)
        {
            double time = calculate_time(1, tmp[1]);
            time_max = time > time_max ? time : time_max;
            for (int ID = 3; ID <= 4; ID++)
            {
                time = calculate_time(ID, tmp[ID]);
                time_max = time > time_max ? time : time_max;
            }
            ROS_INFO("Done! Wait for %lf seconds", time_max);
            ros::Time now = ros::Time::now();
            ros::Duration du(time_max); // 以秒为单位
            ros::Time after_now = now + du;
            wait_time_ = after_now.toSec();
            sc_ptr->WritePos(1, (u16)Position[1], 0, Speed[1]);
            sm_st_ptr->SyncWritePosEx(Id + 3, 2, Position + 3, Speed + 3, ACC + 3);
            du.sleep();

            time = calculate_time(2, tmp[2]);
            ROS_INFO("Done! Wait for %lf seconds", time);
            now = ros::Time::now();
            ros::Duration du2(time); // 以秒为单位
            after_now = now + du2;
            wait_time_ = after_now.toSec();
            sm_st_ptr->WritePosEx(2, Position[2], Speed[2]);
            du2.sleep();

            time = calculate_time(5, tmp[5]);
            ROS_INFO("Done! Wait for %lf seconds", time);
            now = ros::Time::now();
            ros::Duration du3(time); // 以秒为单位
            after_now = now + du3;
            wait_time_ = after_now.toSec();
            sc_ptr->WritePos(5, (u16)Position[5], 0, Speed[5]);
        }
        return valid;
    }

    bool Pos::read_position(int ID)
    {
        if (ID == 1 || ID == 5)
            Position[ID] = (s16)sc_ptr->ReadPos(ID);
        else if (ID == 2 || ID == 3 || ID == 4)
            Position[ID] = (s16)sm_st_ptr->ReadPos(ID);
        else
        {
            ROS_ERROR("ID error!");
            return false;
        }
        if (Position[ID] != -1)
        {
            usleep(10 * 1000);
        }
        else
        {
            ROS_ERROR("Failed to read positon!");
            sleep(1);
            return false;
        }
        return true;
    }

    bool Pos::read_all_position()
    {
        bool valid = true;
        int ID = 1;
        while (valid)
        {
            valid = read_position(ID);
            if ((++ID) > 5)
                break;
        }
        return valid;
    }

    bool Pos::refresh_xyz()
    {
        bool valid = read_all_position();
        if (valid)
        {
            double deg1 = (Position[1] - ARM_JOINT1_POS_WHEN_DEG0) / (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * 180.0;
            double deg2 = (Position[2] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180.0;
            double deg3 = (Position[3] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180.0;
            double deg4 = (Position[4] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180.0;
            valid = forward_kinematics(deg1, deg2, deg3, deg4, x, y, z);
        }
        return valid;
    }

    void Pos::end()
    {
        sc_ptr->end();
        sm_st_ptr->end();
    }

    bool generate_valid_position(double deg1, double deg2, double deg3, double deg4, double &x, double &y, double &z, bool &look)
    {
        x = y = z = 0;
        look = deg4 < 90;
        bool valid = forward_kinematics(deg1, deg2, deg3, deg4, x, y, z);
        Axis ax;
        ax.x = x;
        ax.y = y;
        ax.z = z;
        if (valid)
        {
            valid = ax.test_ok(deg1, deg2, deg3, deg4, look);
        }
        return valid;
    }

    cv::Mat Pos::R_end_to_base()
    {
        double deg1 = (Position[1] - ARM_JOINT1_POS_WHEN_DEG0) / (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * 180;
        double deg2 = (Position[2] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180;
        double deg3 = (Position[3] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180;
        double deg4 = (Position[4] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180;
        Angle j1 = Angle(deg1);
        j1._j_degree_convert(1);
        Angle j2 = Angle(deg2);
        j2._j_degree_convert(2);
        Angle j3 = Angle(deg3);
        j3._j_degree_convert(3);
        Angle j4 = Angle(deg4);
        j4._j_degree_convert(4);
        Angle alpha = j2 + j3 + j4;
        double cj1 = j1.cos();
        double sj1 = j1.sin();
        double calpha = alpha.cos();
        double salpha = alpha.sin();
        cv::Mat R = (cv::Mat_<double>(3, 3) << cj1 * calpha, -sj1, cj1 * salpha, sj1 * calpha, cj1, sj1 * salpha, -salpha, 0.0, calpha);
        return R;
    }

    cv::Mat Pos::T_end_to_base()
    {
        Angle j1 = _calculate_j1();
        cv::Mat T = (cv::Mat_<double>(3, 1) << j1.cos() * length(), j1.sin() * length() - ARM_P, height());
        return T;
    }

    ArmPose Pos::end_to_base_now()
    {
        bool valid = refresh_xyz();
        ArmPose a;
        if (valid)
        {
            cv::Mat R = R_end_to_base();
            cv::Mat T = T_end_to_base();
            a.R = R;
            a.t = T;
            a.empty = false;
        }
        return a;
    }

    /**************************************************
     * @brief   将旋转矩阵与平移向量合成为齐次矩阵
     * @note
     * @param   Mat& R   3*3旋转矩阵
     * @param   Mat& T   3*1平移矩阵
     * @return  Mat      4*4齐次矩阵
     **************************************************/
    cv::Mat R_T2homogeneous_matrix(const cv::Mat &R, const cv::Mat &T)
    {
        cv::Mat HomoMtr;
        cv::Mat_<double> R1 = (cv::Mat_<double>(4, 3) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                               R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                               R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
                               0, 0, 0);
        cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) << T.at<double>(0, 0),
                               T.at<double>(1, 0),
                               T.at<double>(2, 0),
                               1);
        cv::hconcat(R1, T1, HomoMtr); // 矩阵拼接
        return HomoMtr;
    }

    void Pos::ping()
    {
        int ID = sc_ptr->Ping(1);
        if (ID != -1)
        {
            ROS_INFO_STREAM("ID:" << ID);
        }
        else
        {
            ROS_WARN("Ping servo ID error!");
        }
        ID = sm_st_ptr->Ping(2);
        if (ID != -1)
        {
            ROS_INFO_STREAM("ID:" << ID);
        }
        else
        {
            ROS_WARN("Ping servo ID error!");
        }
        ID = sm_st_ptr->Ping(3);
        if (ID != -1)
        {
            ROS_INFO_STREAM("ID:" << ID);
        }
        else
        {
            ROS_WARN("Ping servo ID error!");
        }
        ID = sm_st_ptr->Ping(4);
        if (ID != -1)
        {
            ROS_INFO_STREAM("ID:" << ID);
        }
        else
        {
            ROS_WARN("Ping servo ID error!");
        }
        ID = sc_ptr->Ping(5);
        if (ID != -1)
        {
            ROS_INFO_STREAM("ID:" << ID);
        }
        else
        {
            ROS_WARN("Ping servo ID error!");
        }
    }

    cv::Mat Pos::Intrinsics()
    {
        cv::Mat intrinsics = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0.0, 1);
        return intrinsics;
    }

    bool Pos::calculate_cargo_position(double u, double v, double z, double &x, double &y)
    {
        bool valid = refresh_xyz();
        if (valid)
        {
            cv::Mat intrinsics_inv = Intrinsics().inv();
            cv::Mat point_pixel = (cv::Mat_<double>(3, 1) << u, v, 1);
            cv::Mat point_temp = intrinsics_inv * point_pixel;
            // 单位统一为cm
            cv::Mat temp = R_T2homogeneous_matrix(R_end_to_base(), T_end_to_base()) * R_T2homogeneous_matrix(R_cam_to_end, T_cam_to_end * 0.1);
            double Z = (z - temp.at<double>(2, 3)) / temp.row(2).colRange(0, 3).clone().t().dot(point_temp);
            cv::Mat point = (cv::Mat_<double>(4, 1) << Z * point_temp.at<double>(0, 0), Z * point_temp.at<double>(1, 0), Z, 1);
            point = temp * point;
            x = point.at<double>(0, 0);
            y = point.at<double>(1, 0);
        }
        return valid;
    }

    ArmController::ArmController() : ps_(&sm_st_, &sc_), default_roi_(480, 0, 960, 1080)
    {
        cargo_x_.reserve(10);
        cargo_y_.reserve(10);
    };

    ArmController::~ArmController()
    {
        ps_.end();
    }

    void ArmController::init(ros::NodeHandle nh, ros::NodeHandle pnh)
    {
        XmlRpc::XmlRpcValue servo_descriptions;
        XmlRpc::XmlRpcValue default_action;
        if (!pnh.getParam("servo", servo_descriptions))
        {
            ROS_ERROR("No speed and acc specified");
        }
        if (!pnh.getParam("default_action", default_action))
        {
            ROS_ERROR("No default action specified");
        }
        std::string ft_servo;
        pnh.param<std::string>("ft_servo", ft_servo, "/dev/ttyUSB0");
        ROS_INFO_STREAM("serial:" << ft_servo);
        if (!ps_.begin(ft_servo.c_str()))
        {
            ROS_ERROR_STREAM("Cannot open ft servo at" << ft_servo);
        }
        ps_.ping();
        ps_.set_speed_and_acc(servo_descriptions);
        ps_.set_default_position(default_action);
        ps_.reset();

        cargo_client_ = nh.serviceClient<mmdetection_ros::cargoSrv>("cargoSrv");
    }

    bool ArmController::draw_image(const cv_bridge::CvImagePtr &image)
    {
        if (image->image.empty())
        {
            ROS_ERROR("No data!");
            return false;
        }
        // cv::cvtColor(image->image, image->image, cv::COLOR_RGB2BGR);
        img_ = image->image;
        return true;
    }

    bool ArmController::detect_cargo(const sensor_msgs::ImageConstPtr &image_rect, vision_msgs::BoundingBox2DArray &detections, sensor_msgs::ImagePtr &debug_image, cv::Rect &roi)
    {
        if (!cargo_client_.exists())
            cargo_client_.waitForExistence();
        cv_bridge::CvImagePtr cv_image;
        try
        {
            cv_image = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return false;
        }
        bool flag = draw_image(cv_image);
        cv_image->image = cv_image->image(roi);
        mmdetection_ros::cargoSrv cargo;
        debug_image = (*cv_image).toImageMsg();
        cargo.request.image = *debug_image;
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
                        cv::RotatedRect rect(cv::Point2f(cargo.response.results.boxes[color].center.x, cargo.response.results.boxes[color].center.y),
                                             cv::Size2f(cargo.response.results.boxes[color].size_x, cargo.response.results.boxes[color].size_y),
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

    bool ArmController::catch_with_2_steps(const sensor_msgs::ImageConstPtr &image_rect, const int color, double z,
                                           bool &finish, sensor_msgs::ImagePtr &debug_image)
    {
        if (!cargo_x_.size())
        {
            current_color_ = color;
            current_z_ = z;
            ps_.wait_time_ = 0;
        }
        else if (current_color_ != color || current_z_ != z || cargo_x_.size() >= 10)
        {
            cargo_x_.clear();
            cargo_y_.clear();
            current_color_ = color;
            current_z_ = z;
            ps_.wait_time_ = 0;
        }
        finish = false;
        ros::Time now = ros::Time::now();
        if (ps_.wait_time_ && now.toSec() < ps_.wait_time_)
            return true;
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
                    ps_.go_to_and_wait(x_aver, y_aver, current_z_, true);
                    cargo_x_.clear();
                    cargo_y_.clear();
                    finish = true;
                }
            }
            else
                return false;
        }
        return valid;
    }
}