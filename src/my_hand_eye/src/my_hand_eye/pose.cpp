#include "my_hand_eye/pose.h"

namespace my_hand_eye
{
    ArmPose::ArmPose() : empty(true){};

    Pos::Pos(SMS_STS *sm_st_ptr, SCSCL *sc_ptr, bool cat, bool look) : cat_(cat), look_(look),
                                                                       sm_st_ptr_(sm_st_ptr), sc_ptr_(sc_ptr),
                                                                       cargo_table_(sm_st_ptr)
    {
        x = 0;
        y = 0;
        z = 0;
        expand_y = false;
        u8 ID[] = {0, 1, 2, 3, 4, 5};
        memcpy(Id, ID, sizeof(Id));
        memset(Position, 0, sizeof(Position));
        memset(Position_now, 0, sizeof(Position_now));
        memset(Speed, 0, sizeof(Speed));
        memset(ACC, 0, sizeof(ACC));
    }

    bool Pos::begin(const char *argv)
    {
        if (!(sm_st_ptr_->begin(115200, argv)) || !(sc_ptr_->begin(115200, argv)))
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
        for (int32_t i = 0; i < sizeof(Speed) / sizeof(u16); i++)
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
        }
        cargo_table_.set_speed_and_acc(servo_descriptions[5]);
    }

    void Pos::set_action(XmlRpc::XmlRpcValue &action, std::string name)
    {
        ROS_ASSERT(action.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(action[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(action[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(action[2].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        if (name == "default")
        {
            action_default.x = (double)action[0];
            action_default.y = (double)action[1];
            action_default.z = (double)action[2];
        }
        else if (name == "left")
        {
            action_left.x = (double)action[0];
            action_left.y = (double)action[1];
            action_left.z = (double)action[2];
        }
        else if (name == "right")
        {
            action_right.x = (double)action[0];
            action_right.y = (double)action[1];
            action_right.z = (double)action[2];
        }
        else
            ROS_ERROR("Name error!");
    }

    bool Pos::calculate_position()
    {
        double deg1, deg2, deg3, deg4;
        deg1 = deg2 = deg3 = deg4 = 0;
        bool valid = test_ok(deg1, deg2, deg3, deg4, look_);
        if (valid)
        {
            Position[1] = std::round(ARM_JOINT1_POS_WHEN_DEG0 + (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * deg1 / 180);
            Position[2] = std::round(ARM_JOINT2_POS_WHEN_DEG0 + (ARM_JOINT2_POS_WHEN_DEG180 - ARM_JOINT2_POS_WHEN_DEG0) * deg2 / 180);
            Position[3] = std::round(ARM_JOINT3_POS_WHEN_DEG0 + (ARM_JOINT3_POS_WHEN_DEG180 - ARM_JOINT3_POS_WHEN_DEG0) * deg3 / 180);
            Position[4] = std::round(ARM_JOINT4_POS_WHEN_DEG0 + (ARM_JOINT4_POS_WHEN_DEG180 - ARM_JOINT4_POS_WHEN_DEG0) * deg4 / 180);
            Position[5] = std::round(cat_ ? ARM_JOINT5_POS_WHEN_CATCH : ARM_JOINT5_POS_WHEN_LOSEN);
            // ROS_INFO_STREAM("position (from 1 to 5):" << Position[1] << " " << Position[2] << " " << Position[3] << " " << Position[4] << " " << Position[5]);
        }
        return valid;
    }

    bool Pos::go_to(double x, double y, double z, bool cat, bool look)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->cat_ = cat;
        this->look_ = look;

        bool valid = calculate_position();
        if (valid)
        {
            int ID[] = {1, 2, 3, 4, 5};
            if (arrive(ID, 5))
            {
                ROS_INFO("Pose has arrived");
                return valid;
            }
            sc_ptr_->WritePos(1, (u16)Position[1], 0, Speed[1]);
            sm_st_ptr_->SyncWritePosEx(Id + 2, 3, Position + 2, Speed + 2, ACC + 2);
            sc_ptr_->WritePos(5, (u16)Position[5], 0, Speed[5]);
            wait_until_static(ID, 5);
        }
        return valid;
    }

    double Pos::calculate_time(int ID)
    {
        // 时间（单位s）=[(位置-目标)/速度]+(速度/(加速度*100)) or 0.1
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
            if (!read_position(ID[i]) || abs(Position[ID[i]] - Position_now[ID[i]]) > 3)
                return false;
        }
        return true;
    }

    void Pos::wait_for_arriving(int ID[], int IDn)
    {
        double time_max = 0;
        double b = 0.5; // 在估计时间的特定比例开始采样
        if (read_all_position())
        {
            for (int i = 0; i < IDn; i++)
            {
                double time = calculate_time(ID[i]);
                time_max = time > time_max ? time : time_max;
            }
        }
        else
            time_max = 15;
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

    // bool Pos::do_first_step(double x, double y)
    // {
    //     this->x = x;
    //     this->y = y;
    //     double deg1 = 0;
    //     bool valid = first_step(deg1);
    //     if (valid)
    //     {
    //         Position[1] = (s16)std::round(ARM_JOINT1_POS_WHEN_DEG0 + (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * deg1 / 180);
    //         sc_ptr_->WritePos(1, (u16)Position[1], 0, Speed[1]);
    //         int ID[] = {1};
    //         wait_until_static(ID, 1);
    //     }
    //     return valid;
    // }

    bool Pos::reset()
    {
        bool valid = go_to(action_default.x, action_default.y, action_default.z,
                           false, true);
        ros::Duration(1).sleep();
        return valid;
    }

    bool Pos::go_to_and_wait(double x, double y, double z, bool cat)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->cat_ = cat;
        this->look_ = false;
        bool valid = calculate_position();
        if (valid)
        {
            sc_ptr_->WritePos(1, (u16)Position[1], 0, Speed[1]);
            sm_st_ptr_->SyncWritePosEx(Id + 3, 2, Position + 3, Speed + 3, ACC + 3);
            int ID[] = {1, 3, 4};
            wait_until_static(ID, 3);

            sm_st_ptr_->WritePosEx(2, Position[2], Speed[2]);
            int ID2[] = {2};
            wait_until_static(ID2, 1);

            sc_ptr_->WritePos(5, (u16)Position[5], 0, Speed[5]);
            int ID3[] = {5};
            wait_until_static(ID3, 1);
        }
        return valid;
    }

    bool Pos::go_to_by_midpoint(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->cat_ = true;
        this->look_ = false;
        bool valid = calculate_position();
        if (valid)
        {
            s16 Position_goal[6] = {0};
            double x_goal = 0, y_goal = 0, z_goal = 0;
            if (valid = find_a_midpoint(Position_goal, x_goal, y_goal, z_goal))
            {
                sc_ptr_->WritePos(1, (u16)Position[1], 0, Speed[1]);
                sm_st_ptr_->SyncWritePosEx(Id + 3, 2, Position + 3, Speed + 3, ACC + 3);
                int ID[] = {1, 3, 4};
                wait_until_static(ID, 3);

                sm_st_ptr_->WritePosEx(2, Position[2], Speed[2]);
                int ID2[] = {2};
                wait_until_static(ID2, 1);

                memcpy(Position, Position_goal, sizeof(Position));

                sm_st_ptr_->WritePosEx(3, Position[3], Speed[3]);
                sm_st_ptr_->WritePosEx(4, Position[4], Speed[4]);
                sm_st_ptr_->RegWritePosEx(2, Position[2], Speed[2]);
                wait_for_alpha_decrease(3);
                int ID3[] = {2, 3, 4};
                sm_st_ptr_->RegWriteAction(2);
                wait_until_static(ID3, 3);

                sc_ptr_->WritePos(5, (u16)Position[5], 0, Speed[5]);
                int ID4[] = {5};
                double load_max = wait_until_static(ID4, 1, true);
                if (load_max < 400)
                {
                    ROS_WARN("Failed to catch!");
                    return false;
                }
            }
            else
                ROS_WARN("Failed to find a midpoint");
        }
        return valid;
    }

    bool Pos::go_to_table(bool cat, int color, bool left)
    {
        this->x = left ? action_left.x : action_right.x;
        this->y = left ? action_left.y : action_right.y;
        this->z = left ? action_left.z : action_right.z;
        this->cat_ = cat;
        this->look_ = false;
        bool valid = calculate_position();
        if (valid)
        {
            sc_ptr_->WritePos(1, (u16)Position[1], 0, Speed[1]);
            sm_st_ptr_->SyncWritePosEx(Id + 3, 2, Position + 3, Speed + 3, ACC + 3);
            if (!cat_)
            {
                cargo_table_.put_next(color);
            }
            else
                cargo_table_.get_next();
            int ID[] = {3, 4, 6};
            wait_until_static(ID, 3);

            sm_st_ptr_->WritePosEx(2, Position[2], Speed[2]);
            int ID2[] = {2};
            wait_until_static(ID2, 1);

            sc_ptr_->WritePos(5, (u16)Position[5], 0, Speed[5]);
            int ID3[] = {5};
            wait_until_static(ID3, 1);
        }
        return valid;
    }

    bool Pos::read_position(int ID)
    {
        if (ID == 1 || ID == 5)
            Position_now[ID] = (s16)sc_ptr_->ReadPos(ID);
        else if (ID == 2 || ID == 3 || ID == 4)
            Position_now[ID] = (s16)sm_st_ptr_->ReadPos(ID);
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

    bool Pos::read_move(int ID)
    {
        int move_now;
        if (ID == 1 || ID == 5)
            move_now = sc_ptr_->ReadMove(ID);
        else if (ID == 2 || ID == 3 || ID == 4)
            move_now = sm_st_ptr_->ReadMove(ID);
        else
        {
            ROS_ERROR("ID error!");
            return true;
        }
        if (move_now != -1)
        {
            usleep(10 * 1000);
        }
        else
        {
            ROS_ERROR("Failed to read move!");
            sleep(1);
            return true;
        }
        return move_now;
    }

    int Pos::read_load(int ID)
    {
        int Load;
        if (ID == 1 || ID == 5)
            Load = sc_ptr_->ReadLoad(ID);
        else if (ID == 2 || ID == 3 || ID == 4)
            Load = sm_st_ptr_->ReadLoad(ID);
        else
        {
            ROS_ERROR("ID error!");
            return true;
        }
        if (Load != -1)
        {
            // ROS_INFO_STREAM("speed:" << speed_now << "ID:" << ID);
            usleep(10 * 1000);
        }
        else
        {
            ROS_ERROR("Failed to read load!");
            sleep(1);
            return Load;
        }
        return Load;
    }

    bool Pos::show_voltage()
    {
        int Volt = sm_st_ptr_->ReadVoltage(2);
        if (Volt != -1)
        {
            usleep(10 * 1000);
            if (Volt < 115)
            {
                ROS_WARN("Voltage %3.1lfV is not enough", Volt / 10.0);
                return false;
            }
            else
            {
                ROS_INFO("Voltage %3.1lfV is enough", Volt / 10.0);
                return true;
            }
        }
        else
        {
            ROS_ERROR("Failed to read move!");
            sleep(1);
            return false;
        }
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

    bool Pos::is_moving(int ID[], int IDn)
    {
        for (int i = 0; i < IDn; i++)
        {
            if (read_move(ID[i]))
                return false;
        }
        return true;
    }

    bool Pos::refresh_xyz(bool read)
    {
        bool valid = read ? read_all_position() : true;
        if (valid)
        {
            if (!read)
            {
                double deg1 = ((double)Position[1] - ARM_JOINT1_POS_WHEN_DEG0) / (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * 180.0;
                double deg2 = ((double)Position[2] - ARM_JOINT2_POS_WHEN_DEG0) / (ARM_JOINT2_POS_WHEN_DEG180 - ARM_JOINT2_POS_WHEN_DEG0) * 180.0;
                double deg3 = ((double)Position[3] - ARM_JOINT3_POS_WHEN_DEG0) / (ARM_JOINT3_POS_WHEN_DEG180 - ARM_JOINT3_POS_WHEN_DEG0) * 180.0;
                double deg4 = ((double)Position[4] - ARM_JOINT4_POS_WHEN_DEG0) / (ARM_JOINT4_POS_WHEN_DEG180 - ARM_JOINT4_POS_WHEN_DEG0) * 180.0;
                valid = forward_kinematics(deg1, deg2, deg3, deg4, x, y, z, true);
            }
            else
            {
                double deg1 = ((double)Position_now[1] - ARM_JOINT1_POS_WHEN_DEG0) / (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * 180.0;
                double deg2 = ((double)Position_now[2] - ARM_JOINT2_POS_WHEN_DEG0) / (ARM_JOINT2_POS_WHEN_DEG180 - ARM_JOINT2_POS_WHEN_DEG0) * 180.0;
                double deg3 = ((double)Position_now[3] - ARM_JOINT3_POS_WHEN_DEG0) / (ARM_JOINT3_POS_WHEN_DEG180 - ARM_JOINT3_POS_WHEN_DEG0) * 180.0;
                double deg4 = ((double)Position_now[4] - ARM_JOINT4_POS_WHEN_DEG0) / (ARM_JOINT4_POS_WHEN_DEG180 - ARM_JOINT4_POS_WHEN_DEG0) * 180.0;
                valid = forward_kinematics(deg1, deg2, deg3, deg4, x, y, z, true);
            }
        }
        return valid;
    }

    double Pos::wait_until_static(int ID[], int IDn, bool show_load)
    {
        double time_max = 0;
        double load_max = 0;
        double b = 0.5; // 在估计时间的特定比例开始采样
        if (read_all_position())
        {
            for (int i = 0; i < IDn; i++)
            {
                double time = (ID[i] == 6) ? cargo_table_.calculate_time() : calculate_time(ID[i]);
                time_max = time > time_max ? time : time_max;
            }
        }
        else
            time_max = 15;
        ROS_INFO("Done! Wait for %lf seconds", time_max);
        ros::Duration du(time_max * b); // 以秒为单位
        du.sleep();
        ros::Time now = ros::Time::now();
        du = ros::Duration(time_max * (1 - b));
        ros::Time time_after_now = now + du;
        ros::Rate rt(7);
        while (ros::ok() && is_moving(ID, IDn) && ros::Time::now() < time_after_now)
        {
            rt.sleep();
            if (show_load)
            {
                for (int i = 0; i < IDn; i++)
                {
                    int load = read_load(ID[i]);
                    ROS_INFO("ID:%d, load:%d", ID[i], load);
                    load_max = load > load_max ? load : load_max;
                }
            }
        }
        return load_max;
    }

    void Pos::end()
    {
        sc_ptr_->end();
        sm_st_ptr_->end();
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

    cv::Mat Pos::R_cam_to_end()
    {
        return (cv::Mat_<double>(3, 3) << 0.04981894561782396, -0.1957598703411149, 0.9793855960864229,
                -0.9942011865146739, 0.08384746816892941, 0.06733203408835098,
                -0.09529991285590336, -0.977060732629088, -0.1904475029081951);
    }

    cv::Mat Pos::T_cam_to_end()
    {
        return (cv::Mat_<double>(3, 1) << 1.887243879353053, 0, -31.95990161597666) * 0.1;
    }

    cv::Mat Pos::R_end_to_base()
    {
        double deg1 = ((double)Position_now[1] - ARM_JOINT1_POS_WHEN_DEG0) / (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * 180;
        double deg2 = ((double)Position_now[2] - ARM_JOINT2_POS_WHEN_DEG0) / (ARM_JOINT2_POS_WHEN_DEG180 - ARM_JOINT2_POS_WHEN_DEG0) * 180;
        double deg3 = ((double)Position_now[3] - ARM_JOINT3_POS_WHEN_DEG0) / (ARM_JOINT3_POS_WHEN_DEG180 - ARM_JOINT3_POS_WHEN_DEG0) * 180;
        double deg4 = ((double)Position_now[4] - ARM_JOINT4_POS_WHEN_DEG0) / (ARM_JOINT4_POS_WHEN_DEG180 - ARM_JOINT4_POS_WHEN_DEG0) * 180;
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
        cv::Mat R = (cv::Mat_<double>(3, 3) << cj1 * calpha, -sj1, cj1 * salpha,
                     sj1 * calpha, cj1, sj1 * salpha,
                     -salpha, 0.0, calpha);
        return R;
    }

    cv::Mat Pos::T_end_to_base()
    {
        cv::Mat T = (cv::Mat_<double>(3, 1) << x, y, z);
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
        int ID = sc_ptr_->Ping(1);
        if (ID != -1)
        {
            ROS_INFO_STREAM("ID:" << ID);
        }
        else
        {
            ROS_WARN("Ping servo ID error!");
        }
        ID = sm_st_ptr_->Ping(2);
        if (ID != -1)
        {
            ROS_INFO_STREAM("ID:" << ID);
        }
        else
        {
            ROS_WARN("Ping servo ID error!");
        }
        ID = sm_st_ptr_->Ping(3);
        if (ID != -1)
        {
            ROS_INFO_STREAM("ID:" << ID);
        }
        else
        {
            ROS_WARN("Ping servo ID error!");
        }
        ID = sm_st_ptr_->Ping(4);
        if (ID != -1)
        {
            ROS_INFO_STREAM("ID:" << ID);
        }
        else
        {
            ROS_WARN("Ping servo ID error!");
        }
        ID = sc_ptr_->Ping(5);
        if (ID != -1)
        {
            ROS_INFO_STREAM("ID:" << ID);
        }
        else
        {
            ROS_WARN("Ping servo ID error!");
        }
        ID = sm_st_ptr_->Ping(6);
        if (ID != -1)
        {
            ROS_INFO_STREAM("ID:" << ID);
        }
        else
        {
            ROS_WARN("Ping servo ID error!");
        }
    }

    cv::Mat Pos::intrinsics()
    {
        cv::Mat intrinsics = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0.0, 1);
        return intrinsics;
    }

    cv::Mat Pos::extrinsics()
    {
        return R_T2homogeneous_matrix(R_end_to_base(), T_end_to_base()) *
               R_T2homogeneous_matrix(R_cam_to_end(), T_cam_to_end());
    }

    bool Pos::calculate_cargo_position(double u, double v, double z, double &x, double &y)
    {
        bool valid = refresh_xyz();
        if (valid)
        {
            cv::Mat intrinsics_inv = intrinsics().inv();
            cv::Mat point_pixel = (cv::Mat_<double>(3, 1) << u, v, 1);
            cv::Mat point_temp = intrinsics_inv * point_pixel;
            // 单位统一为cm
            cv::Mat temp = extrinsics();
            double Z = (z - temp.at<double>(2, 3)) / temp.row(2).colRange(0, 3).clone().t().dot(point_temp);
            cv::Mat point = (cv::Mat_<double>(4, 1) << Z * point_temp.at<double>(0, 0),
                             Z * point_temp.at<double>(1, 0), Z, 1);
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

    bool Pos::dfs_midpoint(double length_goal, double height_goal)
    {
        double dist, k;
        double kbound = 0.55;
        static int ind[] = {4, 3, 2};
        static s16 sig[] = {1, -1, 1};
        if ((dist = distance(length_goal, height_goal, k)) > 8 || dist < -0.2 || k > 4 || k < -4)
            return false;
        else if (dist > 4 && k < kbound && k > 0)
        {
            ROS_INFO("Succeeded to find a midpoint!");
            return true;
        }
        else if (dist < 0)
            return false;
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
            // usleep(1e5);
            // ARM_ERROR_XYZ(*this);
            // ROS_ERROR_STREAM(i << " " << dist << " " << k);
            if (dfs_midpoint(length_goal, height_goal))
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

    bool Pos::find_a_midpoint(s16 Position_goal[], double &x_goal, double &y_goal, double &z_goal)
    {
        memcpy(Position_goal, Position, sizeof(Position));
        x_goal = x;
        y_goal = y;
        z_goal = z;
        if (y < 0 || z >= 20)
        {
            ROS_WARN("Set y >= 0 and z < 20 when using find_a_midpoint!");
            return false;
        }
        // ROS_INFO_STREAM("length:" << length() << " height:" << height());
        bool fin = dfs_midpoint(length(), height());
        if (fin)
        {
            double deg1 = 0, deg2 = 0, deg3 = 0, deg4 = 0;
            fin = backward_kinematics(deg1, deg2, deg3, deg4, false);
            if (fin)
            {
                Position[1] = std::round(ARM_JOINT1_POS_WHEN_DEG0 + (ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0) * deg1 / 180);
                Position[2] = std::round(ARM_JOINT2_POS_WHEN_DEG0 + (ARM_JOINT2_POS_WHEN_DEG180 - ARM_JOINT2_POS_WHEN_DEG0) * deg2 / 180);
                Position[3] = std::round(ARM_JOINT3_POS_WHEN_DEG0 + (ARM_JOINT3_POS_WHEN_DEG180 - ARM_JOINT3_POS_WHEN_DEG0) * deg3 / 180);
                Position[4] = std::round(ARM_JOINT4_POS_WHEN_DEG0 + (ARM_JOINT4_POS_WHEN_DEG180 - ARM_JOINT4_POS_WHEN_DEG0) * deg4 / 180);
            }
            // ROS_INFO_STREAM("length:" << length() << " height:" << height());
        }
        return fin;
    }

    void Pos::get_points(double h, my_hand_eye::PointArray &arr)
    {
        s16 Position_goal[6] = {0};
        double x_goal = 0, y_goal = 0, z_goal = 0;
        z = h;
        double length_max = length();
        // 储存上一步状态
        bool last_ok = calculate_position();
        bool last_mid = last_ok ? find_a_midpoint(Position_goal, x_goal, y_goal, z_goal) : false; // last_ok;
        if (last_ok)
        {
            memcpy(Position, Position_goal, sizeof(Position));
            x = x_goal;
            y = y_goal;
            z = z_goal;
        }
        while (ros::ok() && (y > 0 || (expand_y && y > -length_max - 2 * ARM_P)))
        {
            double dy = 1; // 步长
            y -= dy;
            // ROS_ERROR_STREAM(y);
            // usleep(1e5);
            if (y > 0 || (expand_y && y > -length_max - 2 * ARM_P))
            {
                bool ok = calculate_position();
                bool mid = ok ? find_a_midpoint(Position_goal, x_goal, y_goal, z_goal) : false; // ok;
                if (ok)
                {
                    memcpy(Position, Position_goal, sizeof(Position));
                    x = x_goal;
                    y = y_goal;
                    z = z_goal;
                }
                geometry_msgs::Point pt[6];
                if (ok == last_ok && ok) // 和上一步判断得ok状态相同，且为真
                {
                    memcpy(Position_goal, Position, sizeof(Position));
                    for (int i = 1; i <= 5; i++)
                    {
                        y += dy / 5;
                        pt[i].x = x;
                        pt[i].y = y;
                        pt[i].z = z;
                        double deg1, deg2, deg3, deg4;
                        deg1 = deg2 = deg3 = deg4 = 0;
                        if (test_ok(deg1, deg2, deg3, deg4, look_))
                        {
                            Angle j2 = Angle(deg2);
                            Angle j3 = Angle(deg3);
                            Angle j4 = Angle(deg4);
                            j2._j_degree_convert(2);
                            j3._j_degree_convert(3);
                            j4._j_degree_convert(4);
                            double alpha = j2._get_degree() + j3._get_degree() + j4._get_degree();
                            // ROS_ERROR_STREAM(deg3 << " " << deg4);
                            // ROS_ERROR_STREAM(alpha);
                            if (cvRound(alpha) == -90 && abs(length() - 4.46103) < 0.1)
                            {
                                ROS_ERROR_STREAM(length());
                                ARM_ERROR_XYZ(*this);
                            }
                        }
                    }
                    y -= dy;
                    // ROS_ERROR_STREAM("c" << y);
                    memcpy(Position, Position_goal, sizeof(Position));
                    if (mid == last_mid) // 和上一步判断得midpoint状态相同
                    {
                        for (int i = 1; i <= 5; i++)
                        {
                            my_hand_eye::Point point;
                            point.has_midpoint = mid;
                            point.point = pt[i];
                            // ROS_ERROR_STREAM(pt[i].y << " 1");
                            arr.points.push_back(point);
                        }
                    }
                    else // 和上一步判断得midpoint状态不同，降低步长继续判断
                    {
                        for (int i = 1; i <= 5; i++)
                        {
                            my_hand_eye::Point point;
                            point.point = pt[i];
                            y = pt[i].y;
                            if (calculate_position())
                            {
                                // ARM_INFO_XYZ(*this);
                                point.has_midpoint = find_a_midpoint(Position_goal, x_goal,
                                                                     y_goal, z_goal);
                                x = x_goal;
                                y = y_goal;
                                z = z_goal;
                                memcpy(Position, Position_goal, sizeof(Position));
                                // ROS_ERROR_STREAM(pt[i].y << " 2");
                                arr.points.push_back(point);
                            }
                        }
                    }
                }
                else if (ok != last_ok) // 和上一步判断得ok状态不同，降低步长继续判断
                {
                    for (int i = 1; i <= 5; i++)
                    {
                        y += dy / 5;
                        // ROS_ERROR_STREAM(y << " 3");
                        if (calculate_position())
                        {
                            my_hand_eye::Point point;
                            pt[i].x = x;
                            pt[i].y = y;
                            pt[i].z = z;
                            point.point = pt[i];
                            point.has_midpoint = find_a_midpoint(Position_goal, x_goal, y_goal, z_goal);
                            x = x_goal;
                            y = y_goal;
                            z = z_goal;
                            (Position, Position_goal, 6 * sizeof(s16));
                            // ROS_ERROR_STREAM(pt[i].y << " 3");
                            arr.points.push_back(point);
                        }
                    }
                    y -= dy;
                }
                // ROS_ERROR_STREAM(y << "b");
                last_ok = ok;
                last_mid = mid;
            }
        }
    }

    bool Pos::find_points_with_height(double h, my_hand_eye::PointArray &arr)
    {
        Position[1] = round((ARM_JOINT1_POS_WHEN_DEG0 + ARM_JOINT1_POS_WHEN_DEG180) / 2);
        Position[2] = round(ARM_JOINT2_POS_WHEN_DEG0);
        Position[3] = round((ARM_JOINT3_POS_WHEN_DEG0 + ARM_JOINT3_POS_WHEN_DEG180) / 2);
        Position[4] = round((ARM_JOINT4_POS_WHEN_DEG0 + ARM_JOINT4_POS_WHEN_DEG180) / 2);
        double err = 0.1;
        if (!refresh_xyz(false))
            return false;
        if (h < 0)
        {
            ROS_ERROR("h cannot less than 0.");
            return false;
        }
        if (z > h)
        {
            while (ros::ok() && refresh_xyz(false) && z > h + err)
            {
                Position[3]--;
            }
        }
        else
        {
            while (ros::ok() && refresh_xyz(false) && z < h - err)
            {
                Position[2]++;
            }
        }
        if (refresh_xyz(false) && abs(z - h) <= err)
        {
            get_points(h, arr);
            // ROS_ERROR_STREAM("size:" << arr.points.size());
            if (arr.points.size())
            {
                my_hand_eye::Plot plt;
                return true;
            }
            else
            {
                ROS_ERROR("arr has no size!");
                return false;
            }
        }
        else
            return false;
    }

    void Pos::wait_for_alpha_decrease(double alpha_bound)
    {
        ros::Rate rate(7);
        ros::Duration du(0.8);
        ros::Time after_now = ros::Time::now() + du;
        bool flag = false;
        while (ros::ok() && ros::Time::now() < after_now &&
               read_position(2) && read_position(3) && read_position(4))
        {
            double deg2 = (Position_now[2] - ARM_JOINT2_POS_WHEN_DEG0) / (ARM_JOINT2_POS_WHEN_DEG180 - ARM_JOINT2_POS_WHEN_DEG0) * 180.0;
            double deg3 = (Position_now[3] - ARM_JOINT3_POS_WHEN_DEG0) / (ARM_JOINT3_POS_WHEN_DEG180 - ARM_JOINT3_POS_WHEN_DEG0) * 180.0;
            double deg4 = (Position_now[4] - ARM_JOINT4_POS_WHEN_DEG0) / (ARM_JOINT4_POS_WHEN_DEG180 - ARM_JOINT4_POS_WHEN_DEG0) * 180.0;
            Angle j2 = Angle(deg2);
            Angle j3 = Angle(deg3);
            Angle j4 = Angle(deg4);
            if (!(j2._valid_degree(2) && j3._valid_degree(3) && j4._valid_degree(4)))
            {
                ROS_WARN("invalid degree!");
                return;
            }
            j2._j_degree_convert(2);
            j3._j_degree_convert(3);
            j4._j_degree_convert(4);
            double alpha = (j2 + j3 + j4)._get_degree();
            ROS_INFO_STREAM("alpha:" << alpha);
            if (!flag)
            {
                alpha_bound = alpha - alpha_bound;
                flag = true;
            }
            else if (alpha < alpha_bound)
            {
                ROS_INFO("Stop waiting because of alpha");
                break;
            }
            rate.sleep();
        }
    }
}