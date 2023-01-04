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
        x = ARM_DEFAULT_X;
        y = ARM_DEFAULT_Y;
        z = ARM_DEFAULT_Z;
    }

    bool Pos::begin(const char *argv)
    {
        if (!(sm_st_ptr->begin(115200, argv)) || !(sc_ptr->begin(115200, argv)))
        {
            ROS_ERROR("Failed to init motor!");
            return 0;
        }
        else
            return 1;
    }

    void Pos::get_speed_and_acc(XmlRpc::XmlRpcValue &servo_descriptions)
    {
        // Ensure the type is correct
        ROS_ASSERT(servo_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
        // Loop through all servo descriptions
        for (int32_t i = 0; i < servo_descriptions.size(); i++)
        {

            // i-th servo description
            XmlRpc::XmlRpcValue &servo_description = servo_descriptions[i];

            // Assert the servo description is a struct
            ROS_ASSERT(servo_descriptions.getType() ==
                       XmlRpc::XmlRpcValue::TypeStruct);
            // Assert type of field "id" is an int
            ROS_ASSERT(servo_description["id"].getType() ==
                       XmlRpc::XmlRpcValue::TypeInt);
            // Assert type of field "speed" is a int
            ROS_ASSERT(servo_description["size"].getType() ==
                       XmlRpc::XmlRpcValue::TypeInt);
            // Assert type of field "acc" is a int
            ROS_ASSERT(servo_description["acc"].getType() ==
                       XmlRpc::XmlRpcValue::TypeInt);

            int id = (int)servo_description["id"]; // tag id
            Speed[id] = (int)servo_description["speed"];
            ACC[id] = (int)servo_description["acc"];
            ROS_INFO_STREAM("Loaded servo desciptions id: " << id << ", speed: " << Speed[id] << ", acc: " << ACC[id]);
            // 此处应注意
        }
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
        this->x = x;
        this->y = y;
        this->z = z;
        this->cat = cat;
        this->look = look;
        bool valid = calculate_position();
        if (valid)
        {
            sc_ptr->WritePos(1, (u16)Position[1], 0, Speed[1]);
            sm_st_ptr->SyncWritePosEx(ID + 2, 3, Position + 2, Speed + 2, ACC + 2);
            sc_ptr->WritePos(5, (u16)Position[5], 0, Speed[5]);
            ROS_INFO("Done!");
        }
        return valid;
    }

    bool Pos::reset()
    {
        return go_to(ARM_DEFAULT_X, ARM_DEFAULT_Y, ARM_DEFAULT_Z, false, true);
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
    cv::Mat R_T2homogeneous_matrix(const cv::Mat& R,const cv::Mat& T)
    {
        cv::Mat HomoMtr;
        cv::Mat_<double> R1 = (cv::Mat_<double>(4, 3) << 
                                            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                                            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                                            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
                                            0, 0, 0);
        cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) <<
                                            T.at<double>(0,0),
                                            T.at<double>(1,0),
                                            T.at<double>(2,0),
                                            1);
        cv::hconcat(R1, T1, HomoMtr);		//矩阵拼接
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
            cv::Mat temp1 = (cv::Mat_<double>(3, 1) << 0, 0, 0);
            cv::Mat temp2 = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
            cv::hconcat(intrinsics_inv, temp1, intrinsics_inv);
            cv::vconcat(intrinsics_inv, temp2, intrinsics_inv);
            cv::Mat point_pixel = (cv::Mat_<double>(4, 1) << u, v, 1, 1);
            cv::Mat point_temp = R_T2homogeneous_matrix(R_end_to_base(), T_end_to_base())
                                *R_T2homogeneous_matrix(R_cam_to_end, T_cam_to_end) * intrinsics_inv * point_pixel;
            double tmp = z / point_pixel.at<double>(2, 0);
            x = point_pixel.at<double>(0, 0) * tmp;
            y = point_pixel.at<double>(1, 0) * tmp;
        }
        return valid;
    }

    ArmController::ArmController(ros::NodeHandle pnh) : ps_(&sm_st_, &sc_)
    {
        XmlRpc::XmlRpcValue servo_descriptions;
        if (!pnh.getParam("servo", servo_descriptions))
        {
            ROS_ERROR("No speed and acc specified");
        }
        std::string ft_servo;
        pnh.param<std::string>("ft_servo", ft_servo, "/dev/ttyUSB0");
        ROS_INFO_STREAM("serial:" << ft_servo);
        if (!ps_.begin(ft_servo.c_str()))
        {
            ROS_ERROR_STREAM("Cannot open ft servo at" << ft_servo);
        }
        ps_.ping();
        ps_.get_speed_and_acc(servo_descriptions);
    }
}