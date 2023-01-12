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

    void Pos::set_action(XmlRpc::XmlRpcValue& action, std::string name)
    {
        ROS_ASSERT(action.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(action[0].getType() == XmlRpc::XmlRpcValue::TypeInt);
        ROS_ASSERT(action[1].getType() == XmlRpc::XmlRpcValue::TypeInt);
        ROS_ASSERT(action[2].getType() == XmlRpc::XmlRpcValue::TypeInt);
        if (name == "default")
        {
            default_x = (int)action[0];
            default_y = (int)action[1];
            default_z = (int)action[2];
        }
        else if (name == "put")
        {
            put_x = (int)action[0];
            put_y = (int)action[1];
            put_z = (int)action[2];
        }
        else
            ROS_ERROR("Name error!");
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
        // double time_max = 0;
        // bool valid = read_all_position();
        // s16 tmp[6] = {0};
        // if (valid)
        // {
        //     memcpy(tmp, Position, 6 * sizeof(s16));
        // }
        this->x = x;
        this->y = y;
        this->z = z;
        this->cat = cat;
        this->look = look;

        bool valid = calculate_position();
        if (valid)
        {
            // for (int ID = 1; ID <= 5; ID++)
            // {
            //     double time = calculate_time(ID, tmp[ID]);
            //     time_max = time > time_max ? time : time_max;
            // }
            // ROS_INFO("Done! Wait for %lf seconds", time_max);
            // ros::Time now = ros::Time::now();
            // ros::Duration du(time_max); // 以秒为单位
            // ros::Time after_now = now + du;
            // wait_time_ = after_now.toSec();
            int ID[] = {1, 2, 3, 4, 5};
            if (read_all_position() && arrive(ID, 5))
                return valid;
            sc_ptr->WritePos(1, (u16)Position[1], 0, Speed[1]);
            sm_st_ptr->SyncWritePosEx(Id + 2, 3, Position + 2, Speed + 2, ACC + 2);
            sc_ptr->WritePos(5, (u16)Position[5], 0, Speed[5]);
            wait_for_arriving(ID, 5);
            // du.sleep();
        }
        return valid;
    }

    double Pos::calculate_time(int ID)
    {
        // 时间（单位s）=[(位置-目标)/速度]+(速度/(加速度*100))
        double time = 15.0;
        if (ID == 5)
            time = abs((Position[ID] - Position_now[ID]) * 1.0 / (Speed[ID] + 0.01)) + 0.1;
        else if (ID == 3 || ID == 4)
            time = abs((Position[ID] - Position_now[ID]) * 1.0 / (Speed[ID] + 0.01)) + Speed[ID] / (100.0 * ACC[ID] + 0.01);
        else if (ID == 1)
            time = abs((Position[ID] - Position_now[ID]) * 0.2 / (Speed[ID] + 0.01)) + 0.1;
        else if (ID == 2)
            time = abs((Position[ID] - Position_now[ID]) * 0.02 / (Speed[ID] + 0.01)) + Speed[ID] / (100.0 * ACC[ID] + 0.01);
        else
            ROS_ERROR("ID error!");
        time = time > 15.0 ? 15.0 : time;
        ROS_INFO_STREAM("ID:" << ID << " time:" << time);
        return time;
    }

    bool Pos::arrive(int ID[], int IDn)
    {
        for (int i = 0; i < IDn; i++)
        {
            if (!read_position(ID[i]) || abs(Position[ID[i]] - Position_now[ID[i]]) > 2)
                return false;
        }
        return true;
    }

    void Pos::wait_for_arriving(int ID[], int IDn)
    {
        double time_max = 0;
        double b = 0.5;
        read_all_position();
        for (int i = 0; i < IDn; i++)
        {
            double time = calculate_time(ID[i]);
            time_max = time > time_max ? time : time_max;
        }
        ROS_INFO("Done! Wait for %lf seconds", time_max);
        ros::Duration du(time_max * b); // 以秒为单位
        du.sleep();
        ros::Time now = ros::Time::now();
        du = ros::Duration(time_max * (1 - b));
        ros::Time time_after_now = now + du;
        ros::Rate rt(7);
        while (ros::ok() && !arrive(ID, IDn) && ros::Time::now() < time_after_now)
        {
            rt.sleep();
        }
    }

    bool Pos::do_first_step(double x, double y)
    {
        this->x = x;
        this->y = y;
        double deg1 = 0;
        bool valid = first_step(deg1);
        if (valid)
        {
            Position[1] = (s16)std::round(ARM_JOINT1_POS_WHEN_DEG0 + (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * deg1 / 180);
            sc_ptr->WritePos(1, (u16)Position[1], 0, Speed[1]);
            int ID[] = {1};
            wait_for_arriving(ID, 1);
        }
        return valid;
    }

    bool Pos::reset()
    {
        bool valid = go_to(default_x, default_y, default_z, false, true);
        ros::Duration(1).sleep();
        return valid;
    }

    bool Pos::go_to_and_wait(double x, double y, double z, bool cat)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->cat = cat;
        this->look = false;
        bool valid = calculate_position();
        if (valid)
        {
            sc_ptr->WritePos(1, (u16)Position[1], 0, Speed[1]);
            sm_st_ptr->SyncWritePosEx(Id + 3, 2, Position + 3, Speed + 3, ACC + 3);
            int ID[] = {1, 3, 4};
            wait_for_arriving(ID, 3);

            sm_st_ptr->WritePosEx(2, Position[2], Speed[2]);
            int ID2[] = {2};
            wait_for_arriving(ID2, 1);

            sc_ptr->WritePos(5, (u16)Position[5], 0, Speed[5]);
            int ID3[] = {5};
            wait_for_arriving(ID3, 1);
        }
        return valid;
    }

    bool Pos::go_to_by_midpoint(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->cat = true;
        this->look = false;
        bool valid = calculate_position();
        if (valid)
        {
            s16 Position_goal[6] = {0};
            if (valid = find_a_midpoint(Position_goal))
            {
                sc_ptr->WritePos(1, (u16)Position[1], 0, Speed[1]);
                sm_st_ptr->SyncWritePosEx(Id + 3, 2, Position + 3, Speed + 3, ACC + 3);
                int ID[] = {1, 3, 4};
                wait_for_arriving(ID, 3);

                sm_st_ptr->WritePosEx(2, Position[2], Speed[2]);
                int ID2[] = {2};
                wait_for_arriving(ID2, 1);

                memcpy(Position, Position_goal, 6 * sizeof(s16));
                
                ros::Duration du1(0.5);
                ros::Duration du2(1);
                sm_st_ptr->WritePosEx(3, Position[3], Speed[3]);
                du1.sleep();
                sm_st_ptr->WritePosEx(4, Position[4], Speed[4]);
                du2.sleep();               
                sm_st_ptr->WritePosEx(2, Position[2], Speed[2]);
                int ID3[] = {2, 3, 4};
                wait_for_arriving(ID3, 3);

                sc_ptr->WritePos(5, (u16)Position[5], 0, Speed[5]);
                int ID4[] = {5};
                wait_for_arriving(ID4, 1);
            }
            else
                ROS_WARN("Failed to find a midpoint");
        }
        return valid;
    }

    bool Pos::read_position(int ID)
    {
        if (ID == 1 || ID == 5)
            Position_now[ID] = (s16)sc_ptr->ReadPos(ID);
        else if (ID == 2 || ID == 3 || ID == 4)
            Position_now[ID] = (s16)sm_st_ptr->ReadPos(ID);
        else
        {
            ROS_ERROR("ID error!");
            return false;
        }
        if (Position_now[ID] != -1)
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

    bool Pos::has_speed(int ID)
    {
        int speed_now;
        if (ID == 1 || ID == 5)
            speed_now = sc_ptr->ReadSpeed(ID);
        else if (ID == 2 || ID == 3 || ID == 4)
            speed_now = sm_st_ptr->ReadSpeed(ID);
        else
        {
            ROS_ERROR("ID error!");
            return true;
        }
        if (speed_now != -1)
        {
            // ROS_INFO_STREAM("speed:" << speed_now << "ID:" << ID);
            usleep(10 * 1000);
        }
        else
        {
            ROS_ERROR("Failed to read speed!");
            sleep(1);
            return true;
        }
        return false;
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

    bool Pos::is_moving()
    {
        bool flag = false;
        int ID = 1;
        while (!flag)
        {
            flag = has_speed(ID);
            if ((++ID) > 5)
                break;
        }
        return flag;
    }

    bool Pos::refresh_xyz(bool read)
    {
        bool valid = read ? read_all_position() : true;
        if (valid)
        {
            if (!read)
            {
                double deg1 = (Position[1] - ARM_JOINT1_POS_WHEN_DEG0) / (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * 180.0;
                double deg2 = (Position[2] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180.0;
                double deg3 = (Position[3] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180.0;
                double deg4 = (Position[4] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180.0;
                valid = forward_kinematics(deg1, deg2, deg3, deg4, x, y, z);
            }
            else
            {
                double deg1 = (Position_now[1] - ARM_JOINT1_POS_WHEN_DEG0) / (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * 180.0;
                double deg2 = (Position_now[2] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180.0;
                double deg3 = (Position_now[3] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180.0;
                double deg4 = (Position_now[4] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180.0;
                valid = forward_kinematics(deg1, deg2, deg3, deg4, x, y, z);
            }
        }
        return valid;
    }

    void Pos::wait_until_static()
    {
        double r = 10;
        ros::Rate rate(r);
        int tim = 0; // 计时器
        int cou = 0; // 静止计数
        while (ros::ok() && cou < 15)
        {
            if (is_moving())
                cou = 0;
            else
                cou++;
            rate.sleep();
            if (++tim > (15 * r))
            {
                ROS_WARN("Timeout!");
                break;
            }
        }
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
        double deg1 = (Position_now[1] - ARM_JOINT1_POS_WHEN_DEG0) / (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * 180;
        double deg2 = (Position_now[2] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180;
        double deg3 = (Position_now[3] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180;
        double deg4 = (Position_now[4] - ARM_JOINT234_POS_WHEN_DEG0) / (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * 180;
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

    double Pos::distance(double length_goal, double height_goal, double &k)
    {
        double length_sub = length() - length_goal;
        double height_sub = height() - height_goal;
        k = length_sub == 0 ? 0 : -height_sub / length_sub;
        // ROS_INFO_STREAM("k:" << k << " distance:" << sqrt(length_sub * length_sub + height_sub * height_sub));
        return sqrt(length_sub * length_sub + height_sub * height_sub) * (length_sub > 0 ? -1 : 1);
    }

    bool Pos::dfs(double length_goal, double height_goal)
    {
        double dist, k;
        double kbound = 0.5;
        static int ind[] = {4, 3, 2};
        static s16 sig[] = {1, -1, 1};
        if ((dist = distance(length_goal, height_goal, k)) > 8 || dist < -0.2 || k > 4 || k < -4)
            return false;
        else if (dist > 4 && k < kbound && k > 0)
        {
            ROS_INFO("Succeeded to find a midpoint!");
            return true;
        }
        double nx = x;
        double ny = y;
        double nz = z;
        for (int i = 0; i < 3; i++)
        {
            if (dist > 0 && k > kbound && sig[i] == 1 || dist > 0 && k < 0 && sig[i] == -1)
                continue;
            Position[ind[i]] += sig[i];
            if (!refresh_xyz(false))
            {
                continue;
            }
            if (dfs(length_goal, height_goal))
            {
                return true;
            }
            else
            {
                Position[ind[i]] -= sig[i];
                x = nx;
                y = ny;
                z = nz;
            }
        }
        return false;
    }

    bool Pos::find_a_midpoint(s16 Position_goal[])
    {
        memcpy(Position_goal, Position, 6 * sizeof(s16));
        ROS_INFO_STREAM("length:" << length() << " height" << height());
        bool fin = dfs(length(), height());
        if (fin)
        {
            double deg1 = 0, deg2 = 0, deg3 = 0, deg4 = 0;
            fin = backward_kinematics(deg1, deg2, deg3, deg4, false);
            if (fin)
            {
                Position[1] = std::round(ARM_JOINT1_POS_WHEN_DEG0 + (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * deg1 / 180);
                Position[2] = std::round(ARM_JOINT234_POS_WHEN_DEG0 + (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * deg2 / 180);
                Position[3] = std::round(ARM_JOINT234_POS_WHEN_DEG0 + (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * deg3 / 180);
                Position[4] = std::round(ARM_JOINT234_POS_WHEN_DEG0 + (ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0) * deg4 / 180);
            }
            ROS_INFO_STREAM("length:" << length() << " height" << height());
        }
        return fin;
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
        pnh.param<std::string>("ft_servo", ft_servo, "/dev/ttyUSB0");
        ROS_INFO_STREAM("serial:" << ft_servo);
        if (!ps_.begin(ft_servo.c_str()))
        {
            ROS_ERROR_STREAM("Cannot open ft servo at" << ft_servo);
        }
        ps_.ping();
        ps_.set_speed_and_acc(servo_descriptions);
        ps_.set_action(default_action);
        ps_.set_action(put_action, "put");

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
            ROS_INFO("3");
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
}