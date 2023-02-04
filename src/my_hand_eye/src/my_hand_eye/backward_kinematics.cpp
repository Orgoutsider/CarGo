#include "my_hand_eye/backward_kinematics.hpp"

#ifndef M_PI
# define M_PI 3.141592653589793238462643383279502884196
#endif

namespace my_hand_eye
{
    Angle::Angle(double du)
    {
        this->deg = du;
        state = normal;
    }

    Angle::Angle(double v1, double v2)
    {
        double rad = std::atan2(v1, v2);
        deg = degree(rad);
        state = normal;
    }

    double Angle::_get_degree()
    {
        return deg;
    };

    double Angle::rad()
    {
        return deg / 180 * M_PI;
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
            deg = 90 - deg;
        }
        else if (joint != 1)
        {
            ROS_ERROR("Set joint to 1,2,3,4!");
        }
    }

    double degree(double rad)
    {
        return rad / M_PI * 180;
    }

    bool Angle::_valid_degree(int joint)
    {

        return 0 <= deg && deg <= 180;
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
        return Angle(deg + a.deg);
    }

    Angle Angle::operator-(const Angle &a) const
    {
        return Angle(deg - a.deg);
    }

    Angle Angle::operator-() const
    {
        return Angle(-deg);
    }

    Angle Angle::operator=(const Angle &t)
    {
        // 如果是对象本身, 则直接返回
        if (this == &t) {
            return *this;
        }

        // 复制等号右边对象的成员值到等号左边对象的成员
        deg = t.deg;
        state = t.state;
        return *this;
    }

    bool Angle::operator>(const Angle &t)
    {
        return deg > t.deg;
    }

    bool Angle::operator<(const Angle &t)
    {
        return deg < t.deg;
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
            ROS_WARN_STREAM("length " << length() << " is out of range " << ARM_MAX_LEN << "." );
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

    bool Axis::_modify_xy()
    {
        static bool flag = false;
        if (flag)
        {
            y = -(y + 2 * ARM_P);
            x = -x;
            flag = false;
        }
        else if (expand_y && (y + ARM_P < 0))
        {
            y = -(y + 2 * ARM_P);
            x = -x;
            flag = true;
        }
        return (expand_y && (y + ARM_P < 0)) || flag;
    }

    bool Axis::_modify_alpha(double &alpha, bool look)
    {
        alpha = 90;
        bool valid = false;
        while (alpha >= 0 && alpha <= 135 && !valid)
        {
            valid = _j123_length_and_height_is_valid(alpha);
            if (!valid)
                alpha = look ? alpha - 1 : alpha + 1;
        }
        return valid;
    }

    bool Axis::backward_kinematics(double &deg1, double &deg2, double &deg3, double &deg4, bool look)
    {
        bool flag = _modify_xy();
        if (z < 0)
        {
            ROS_ERROR("z cannot less than 0.");
            return false;
        }
        if (!expand_y && y < 0)
        {
            ROS_ERROR("y cannot less than 0.");
            return false;
        }
        double alpha = 0;
        bool valid = _modify_alpha(alpha, look);
        if (valid)
        {
            Angle j1 = _calculate_j1();
            Angle j2 = flag ? (-_calculate_j2(alpha)) : _calculate_j2(alpha);
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
        else
            ROS_WARN("alpha invalid!");
        _modify_xy();
        // ROS_ERROR_STREAM("valid:" << valid << " deg1:" << deg1 << " deg2:" << deg2 << " deg3:" << deg3 << " deg4:" << deg4);
        return valid;
    }

    bool Axis::first_step(double &deg1)
    {
        if (z < 0)
        {
            ROS_ERROR("z cannot less than 0.");
            return false;
        }
        if (!expand_y && y < 0)
        {
            ROS_ERROR("y cannot less than 0.");
            return false;
        }
        Angle j1 = _calculate_j1();
        j1._j_degree_convert(1);
        deg1 = j1._get_degree();
        return true;
    }

    bool forward_kinematics(double &deg1, double &deg2, double &deg3, double &deg4,
                            double &x, double &y, double &z, bool expand_y)
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
        bool flag = j2._get_degree() < 0;
        j2 = flag? -j2 : j2;
        double length = ARM_A2 * j2.sin() + ARM_A3 * (j2 + j3).sin() +
                        ARM_A4 * (j2 + j3 + j4).sin();
        double height = ARM_A1 + ARM_A2 * j2.cos() + ARM_A3 * (j2 + j3).cos() +
                        ARM_A4 * (j2 + j3 + j4).cos();
        double alpha = (j2 + j3 + j4)._get_degree();
        // ROS_ERROR("j2:%lf j3:%lf j4:%lf", j2._get_degree(), j3._get_degree(), j4._get_degree());
        z = height;
        x = length * j1.cos();
        y = length * j1.sin() - ARM_P;
        y = flag ? -y - 2 * ARM_P : y;
        if ((0 <= y || expand_y) && z >= 0)
            valid = true;
        // ROS_ERROR_STREAM("valid:" << valid << " x:" << x << " y:" << y << " z:" << z << " length:" << length << " height:" << height << " alpha:" << alpha);
        return valid;
    }

    bool Axis::test_ok(double &deg1, double &deg2, double &deg3, double &deg4, bool look)
    {
        double tx, ty, tz;
        tx = ty = tz = 0;
        bool valid = backward_kinematics(deg1, deg2, deg3, deg4, look);
        if (valid)
        {
            valid = forward_kinematics(deg1, deg2, deg3, deg4, tx, ty, tz, expand_y);
            if (!valid)
            {
                ROS_WARN("Result invalid!");
                return false;
            }
            else if (std::abs(tx - x) > 0.5 || std::abs(ty - y) > 0.5 || std::abs(tz - z) > 0.5)
            {
                ROS_ERROR("Forward kinematics error! tx:%lf ty:%lf tz:%lf x:%lf y:%lf z:%lf",
                          tx, ty, tz, x, y, z);
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
} // namespace my_hand_eye
