#include "my_hand_eye/cargo_table.h"

namespace my_hand_eye
{
    CargoTable::CargoTable(SMS_STS *sm_st_ptr)
        : ID(6), where_(2), where_last_(2),
          sm_st_ptr_(sm_st_ptr), rst_(false), clockwise_(true),
          what_color_({0}), where_cargo_({-1, -1, -1, -1}) {}

    void CargoTable::set_speed_and_acc(XmlRpc::XmlRpcValue &servo_description)
    {
        // Assert the servo description is a struct
        ROS_ASSERT(servo_description.getType() ==
                   XmlRpc::XmlRpcValue::TypeStruct);
        // Assert type of field "speed" is a int
        ROS_ASSERT(servo_description["speed"].getType() ==
                   XmlRpc::XmlRpcValue::TypeInt);
        // Assert type of field "acc" is a int
        ROS_ASSERT(servo_description["acc"].getType() ==
                   XmlRpc::XmlRpcValue::TypeInt);
        Speed = (int)servo_description["speed"];
        ACC = (int)servo_description["acc"];
        ROS_INFO_STREAM("Loaded servo desciptions id: 6, speed: " << Speed << ", acc: " << (int)servo_description["acc"]);
    }

    void CargoTable::reset()
    {
        where_last_ = where_;
        where_ = round(nearest(0));
        sm_st_ptr_->WritePosEx(ID, where_ * ARM_CARGO_TABLE_POS_WHEN_DEG120, Speed, ACC);
        rst_ = true;
    }

    int CargoTable::mod3()
    {
        int mod = where_ % 3;
        if (mod < 0)
            mod += 3;
        return mod;
    }

    double CargoTable::nearest(double where)
    {
        int sub = where - mod3();
        double add = abs(sub) > 1.5 ? (sub > 0 ? sub - 3 : sub + 3) : sub;
        return where_ + add;
    }

    void CargoTable::midpoint()
    {
        where_last_ = where_;
        where_ = 0;
        sm_st_ptr_->WritePosEx(ID, 0.5 * ARM_CARGO_TABLE_POS_WHEN_DEG120, Speed, ACC);
    }

    void CargoTable::put_next(const Color color)
    {
        where_last_ = where_;
        if (rst_)
        {
            rst_ = false;
            where_ = round(nearest(0));
        }
        else if (what_color_.at(mod3()) == 0) // 当前位置没放块
        {
            clockwise_ = !clockwise_;
            return;
        } 
        else if (clockwise_)
            where_++;
        else
            where_--;
        sm_st_ptr_->WritePosEx(ID, where_ * ARM_CARGO_TABLE_POS_WHEN_DEG120, Speed, ACC);
        try
        {
            what_color_.at(mod3()) = color;
            where_cargo_.at(color) = mod3();
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Exception: %s", e.what());
        }
    }

    bool CargoTable::arrived(int tolerance)
    {
        int Position_now = sm_st_ptr_->ReadPos(6);
        if (Position_now != -1)
        {
            usleep(10 * 1000);
        }
        else
        {
            ROS_ERROR("Failed to read positon!");
            sleep(1);
            return false;
        }
        return abs(where_ * ARM_CARGO_TABLE_POS_WHEN_DEG120 -
                   nearest(Position_now * 1.0 / ARM_CARGO_TABLE_POS_WHEN_DEG120) *
                       ARM_CARGO_TABLE_POS_WHEN_DEG120) <=
               tolerance;
    }

    bool CargoTable::is_moving()
    {
        int move_now = sm_st_ptr_->ReadMove(ID);
        if (move_now != -1)
            usleep(10 * 1000);
        else
        {
            ROS_ERROR("Failed to read move!");
            sleep(1);
            return true;
        }
        return move_now;
    }

    double CargoTable::calculate_time()
    {
        // 时间（单位s）=[(位置-目标)/速度]+(速度/(加速度*100)) or 0.1
        int Position = where_ * ARM_CARGO_TABLE_POS_WHEN_DEG120;
        int Position_now = where_last_ * ARM_CARGO_TABLE_POS_WHEN_DEG120;
        double time = abs((Position - Position_now) * 0.025 / (Speed + 0.01)) + Speed / (100.0 * ACC + 0.01);
        time = time > 15.0 ? 15.0 : time;
        // ROS_INFO_STREAM("ID:" << unsigned(ID) << " time:" << time);
        return time;
    }

    void CargoTable::get_next()
    {
        where_last_ = where_;
        if (clockwise_)
            where_++;
        else
            where_--;
        sm_st_ptr_->WritePosEx(ID, where_ * ARM_CARGO_TABLE_POS_WHEN_DEG120, Speed, ACC);
        try
        {
            where_cargo_.at(what_color_.at(mod3())) = -1;
            what_color_.at(mod3()) = 0;
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Exception: %s", e.what());
        }
    }

    void CargoTable::get_color(const Color color)
    {
        where_last_ = where_;
        try
        {
            where_ = round(nearest(where_cargo_.at(color)));
            where_cargo_.at(color) = -1;
            what_color_.at(mod3()) = 0;
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Exception: %s", e.what());
        }
        sm_st_ptr_->WritePosEx(ID, where_ * ARM_CARGO_TABLE_POS_WHEN_DEG120, Speed, ACC);
    }
} // namespace my_hand_eye
