#include "my_hand_eye/pos.hpp"
#include "ros/ros.h"
#include <cmath>

namespace my_hand_eye
{
    Pos::Pos(s16 *Position, u16 *Speed, u8 *ACC, SMS_STS *sm_st_ptr, SCSCL *sc_ptr, bool cat, bool look)
    {
        this->Position = Position;
        this->Speed = Speed;
        this->ACC = ACC;
        this->cat = cat;
        this->look = look;
        this->sm_st_ptr = sm_st_ptr;
        this->sc_ptr = sc_ptr;
        x = ARM_DEFAULT_X;
        y = ARM_DEFAULT_Y;
        z = ARM_DEFAULT_Z;
    }

    bool Pos::begin(char *argv)
    {
        if (!(sm_st_ptr->begin(115200, argv)) || !(sc_ptr->begin(115200, argv)))
        {
            ROS_ERROR("Failed to init motor!");
            return 0;
        }
        else
            return 1;
    }

    bool Pos::calculate_pos()
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
            ROS_INFO_STREAM("position (from 1 to 5):" << Position[1] << " " << Position[2] << " " << Position[3] << " " << Position[4] << " " << Position[5]);
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
        bool valid = calculate_pos();
        if(valid)
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
        if(ID == 1 || ID == 5)
            Position[ID] = (s16)sc_ptr->ReadPos(ID);
        else if (ID == 2 ||ID == 3 ||ID == 4)
            Position[ID] = (s16)sm_st_ptr->ReadPos(ID);
        else
        {
            ROS_ERROR("ID error!");
            return false;
        }
		if(Position[ID]!=-1){
			usleep(10*1000);
		}else{
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
            if((++ID) > 5)break;
        }
        return valid;
    }

    bool Pos::refresh_xyz()
    {
        bool valid = read_all_position();
        if (valid)
        {
            double deg1 = (Position[1] - ARM_JOINT1_POS_WHEN_DEG0)/(ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0)*180.0;
            double deg2 = (Position[2] - ARM_JOINT234_POS_WHEN_DEG0)/(ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0)*180.0;
            double deg3 = (Position[3] - ARM_JOINT234_POS_WHEN_DEG0)/(ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0)*180.0;
            double deg4 = (Position[4] - ARM_JOINT234_POS_WHEN_DEG0)/(ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0)*180.0;
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
        ax.x = x;ax.y = y;ax.z = z;
        if(valid)
        {
            valid = ax.test_ok(deg1, deg2, deg3, deg4, look);
        }
        return valid;
    }

    cv::Mat Pos::R_end_to_base()
    {
        double deg1 = (Position[1] - ARM_JOINT1_POS_WHEN_DEG0)/(ARM_JOINT1_POS_WHEN_DEG180 - ARM_JOINT1_POS_WHEN_DEG0)*180;
        double deg2 = (Position[2] - ARM_JOINT234_POS_WHEN_DEG0)/(ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0)*180;
        double deg3 = (Position[3] - ARM_JOINT234_POS_WHEN_DEG0)/(ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0)*180;
        double deg4 = (Position[4] - ARM_JOINT234_POS_WHEN_DEG0)/(ARM_JOINT234_POS_WHEN_DEG180 - ARM_JOINT234_POS_WHEN_DEG0)*180;
        Angle j1 = Angle(deg1);j1._j_degree_convert(1);
        Angle j2 = Angle(deg2);j2._j_degree_convert(2);
        Angle j3 = Angle(deg3);j3._j_degree_convert(3);
        Angle j4 = Angle(deg4);j4._j_degree_convert(4);
        Angle alpha = j2 + j3 + j4;
        double cj1 = j1.cos();
        double sj1 = j1.sin();
        double calpha = alpha.cos();
        double salpha = alpha.sin();
        cv::Mat R = (cv::Mat_<double>(3, 3) << cj1*calpha, -sj1, cj1*salpha, sj1*calpha, cj1, sj1*salpha, -salpha, 0.0, calpha);
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
}