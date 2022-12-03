#include <cmath>
#include "ros/ros.h"
#include "my_hand_eye/nijie.hpp"

namespace nijie
{
    const double pi = 3.1415926;

    Angle::Angle(double du)
    {
        this->du = du;
        state = normal;
    }

    Angle::Angle(double v1, double v2)
    {
        double rad = std::atan2(v1, v2);
        du = degree(rad);
        state = normal;
    }

    double Angle::_get_degree()
    {
        return du;
    };

    double Angle::rad()
    {
        return du / 180 * pi;
    }

    double Angle::cos()
    {
        return std::cos(rad());
    }

    double Angle::sin()
    {
        return std::sin(rad());
    }

    void Angle::_j_degree_convert(int joint)
    {
        if (joint == 2 || joint == 3 || joint == 4)
        {
            du = 90 - du;
        }
        else if (joint != 1)
        {
            ROS_ERROR("Set joint to 1,2,3,4!");
        }
    }

    double degree(double rad)
    {
        return rad / pi * 180;
    }

    bool Angle::_valid_degree(int joint)
    {
        if (0 <= du && du <= 180)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool Angle::_valid_j(int joint)
    {
        if (state == error)
            return false;
        _j_degree_convert(joint);
        bool res = _valid_degree(joint);
        _j_degree_convert(joint);
        return res;
    }

    void Angle::_set_state(int state)
    {
        this->state = state;
    }

    Angle Angle::operator+(const Angle &a) const
    {
        return Angle(du + a.du);
    }

    Angle Angle::operator-(const Angle &a) const
    {
        return Angle(du - a.du);
    }

    double Axis::height()
    {
        return z;
    }

    double Axis::length()
    {
        return std::sqrt((y + ARM_P) * (y + ARM_P) + x * x);
    }

    double Axis::L(double alpha)
    {
        return length() - ARM_A4 * Angle(alpha).sin();
    }

    double Axis::H(double alpha)
    {
        return height() - ARM_A4 * Angle(alpha).cos() - ARM_A1;
    }

    bool Axis::_out_of_range()
    {
        if (height() > ARM_MAX_HIGH)
        {
            ROS_WARN_STREAM("height " << height() << " is out of range " << ARM_MAX_HIGH << ".");
            return true;
        }
        if (length() > ARM_MAX_LEN)
        {
            ROS_WARN_STREAM("length " << length() << " is out of range " << ARM_MAX_LEN << "." << std::endl);
            return true;
        }
        return false;
    }

    Angle Axis::_calculate_j1()
    {
        if (length() < 1e-2)
        {
            return Angle(0);
        }
        else
            return Angle(y + ARM_P, x);
    };

    Angle Axis::_calculate_j3(double alpha)
    {
        double L = this->H(alpha);
        double H = this->L(alpha);
        double cos3 = (L * L + H * H - ARM_A2 * ARM_A2 - ARM_A3 * ARM_A3) / (2 * ARM_A2 * ARM_A3);
        if (cos3 * cos3 > 1)
        {
            // std::cout << "cos3:" << cos3 << " L:" << L << " H:" << H << std::endl;
            // std::cout << "calculate j3 error" << std::endl;
            Angle j3 = Angle(0);
            j3._set_state(j3.error);
            return j3;
        }
        double sin3 = std::sqrt(1 - cos3 * cos3);
        return Angle(sin3, cos3);
    }

    Angle Axis::_calculate_j2(double alpha)
    {
        Angle j3 = _calculate_j3(alpha);
        double L = this->L(alpha);
        double H = this->H(alpha);
        double K1 = ARM_A2 + ARM_A3 * j3.cos();
        double K2 = ARM_A3 * j3.sin();
        return (Angle(L, H) - Angle(K2, K1));
    }

    Angle Axis::_calculate_j4(double alpha)
    {
        return Angle(Angle(alpha) - _calculate_j2(alpha) - _calculate_j3(alpha));
    }

    bool Axis::_j123_length_and_height_is_valid(double alpha)
    {
        Angle j1 = _calculate_j1();
        Angle j2 = _calculate_j2(alpha);
        Angle j3 = _calculate_j3(alpha);
        Angle j4 = _calculate_j4(alpha);
        return j1._valid_j(1) && j2._valid_j(2) && j3._valid_j(3) && j4._valid_j(4) && !(_out_of_range());
    }

    bool Axis::_modify_alpha(double &alpha, bool look)
    {
        alpha = look?90:180;
        int MIN_ALPHA = look?0:90; // j2+j3+j4 最小值，最后一个joint不后仰
        bool valid = false;
        while (alpha >= MIN_ALPHA && !valid)
        {
            valid = _j123_length_and_height_is_valid(alpha);
            if (!valid)
                alpha -= 1;
        }
        return valid;
    }

    bool Axis::backward_kinematics(double &deg1, double &deg2, double &deg3, double &deg4, bool look)
    {
        if (z < 0)
        {
            std::cout << "z cannot less than 0." << std::endl;
            return false;
        }
        if (y < 0)
        {
            std::cout << "y cannot less than 0." << std::endl;
            return false;
        }
        double alpha = 0;
        bool valid = _modify_alpha(alpha, look);
        if (valid)
        {
            Angle j1 = _calculate_j1();
            Angle j2 = _calculate_j2(alpha);
            Angle j3 = _calculate_j3(alpha);
            Angle j4 = _calculate_j4(alpha);
            j1._j_degree_convert(1);
            j2._j_degree_convert(2);
            j3._j_degree_convert(3);
            j4._j_degree_convert(4);
            deg1 = j1._get_degree();
            deg2 = j2._get_degree();
            deg3 = j3._get_degree();
            deg4 = j4._get_degree();
        }
        ROS_INFO_STREAM("valid:" << valid << " deg1:" << deg1 << " deg2:" << deg2 << " deg3:" << deg3 << " deg4:" << deg4);
        return valid;
    }

    bool forward_kinematics(double &deg1, double &deg2, double &deg3, double &deg4, double &x, double &y, double &z)
    {
        bool valid = false;
        Angle j1 = Angle(deg1);
        Angle j2 = Angle(deg2);
        Angle j3 = Angle(deg3);
        Angle j4 = Angle(deg4);
        if (!(j1._valid_degree(1) && j2._valid_degree(2) && j3._valid_degree(3) && j4._valid_degree(4)))
        {
            return valid;
        }
        j1._j_degree_convert(1);
        j2._j_degree_convert(2);
        j3._j_degree_convert(3);
        j4._j_degree_convert(4);
        double length = ARM_A2 * j2.sin() + ARM_A3 * (j2 + j3).sin() + ARM_A4 * (j2 + j3 + j4).sin();
        double height = ARM_A1 + ARM_A2 * j2.cos() + ARM_A3 * (j2 + j3).cos() + ARM_A4 * (j2 + j3 + j4).cos();
        double alpha = (j2 + j3 + j4)._get_degree();
        z = height;
        x = length * j1.cos();
        y = length * j1.sin() - ARM_P;
        if (0 <= y && z >= 0)
            valid = true;
        ROS_INFO_STREAM("valid:" << valid << " x:" << x << " y:" << y << " z:" << z << " length:" << length << " height:" << height << " alpha:" << alpha);
        return valid;
    }

    bool Axis::test_ok(double &deg1,double &deg2,double &deg3,double &deg4,bool look)
    {
        double tx, ty, tz;
        tx = ty = tz = 0;
        bool valid = backward_kinematics(deg1, deg2, deg3, deg4, look);
        if (valid)
        {
            valid = forward_kinematics(deg1, deg2, deg3, deg4, tx, ty, tz);
            if (!valid || std::abs(tx - x) > 0.5 || std::abs(ty - y) > 0.5 || std::abs(tz - z) > 0.5)
            {
                ROS_WARN("Cannot move to this position");
                return false;
            }
            else
            {
                return true;
            }
        }
        else
        {    
            ROS_WARN("Cannot move to this position");
            return false;
        }
    }
} // namespace nijie
