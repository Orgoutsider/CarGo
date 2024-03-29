#include "my_hand_eye/backward_kinematics.h"

namespace my_hand_eye
{
    Angle::Angle(double deg) : deg(deg)
    {
        state = normal;
    }

    Angle Angle::atan2(double v1, double v2)
    {
        return Angle(degree(std::atan2(v1, v2)));
    }

    double Angle::get_degree()
    {
        return deg;
    };

    double Angle::rad()
    {
        return deg / 180.0 * M_PI;
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

    double Angle::degree(double rad)
    {
        return rad / M_PI * 180;
    }

    bool Angle::_valid_degree(int joint)
    {
        bool valid;
        if (joint == 1)
        {
            valid = 0 <= deg && deg < 360;
        }
        else if (joint == 3)
            valid = -38 <= deg && deg <= 225;
        else
            valid = -28 <= deg && deg <= 208;
        return valid;
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
        if (this == &t)
        {
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

    Angle Angle::now2goal(const Action &enlarge)
    {
        return atan2(tan(rad()) * enlarge.y / enlarge.x, 1);
    }

    Angle Angle::goal2now(const Action &enlarge)
    {
        return atan2(tan(rad()) * enlarge.x / enlarge.y, 1);
    }

    Action::Action() : x(0), y(0), z(0) {}

    Action::Action(double x, double y, double z) : x(x), y(y), z(z) {}

    Action Action::front2left()
    {
        return Action(-y - ARM_P, -ARM_P + x, z);
    }

    Action Action::arm2footprint()
    {
        return Action(0.01 * y, -0.01 * x, z);
    }

    Action Action::footprint2arm()
    {
        return Action(-100 * y, 100 * x, z);
    }

    Action Action::now2goal(double err_x, double err_y, double err_theta, const Action &enlarge)
    {
        // m转化成cm
        err_y = err_y * enlarge.y * 100;
        err_x = err_x * enlarge.x * 100;
        // err_theta = atan(tan(err_theta) * enlarge.y / enlarge.x);
        double theta = atan((y + ARM_P) / (-x)) + err_theta;
        // ROS_INFO_STREAM("theta: " << theta);
        double len = length();
        Action err(-x - (len + err_y) * cos(theta) + err_x * sin(theta),
                   -y - ARM_P + (len + err_y) * sin(theta) + err_x * cos(theta), 0);
        ARM_INFO_XYZ(err);
        return err;
    }

    Action Action::operator+=(const Action &t)
    {
        x += t.x, y += t.y, z += t.z;
        return *this;
    }

    Action Action::operator-=(const Action &t)
    {
        x -= t.x, y -= t.y, z -= t.z;
        return *this;
    }

    Action Action::operator*=(const double &t)
    {
        x *= t, y *= t, z *= t;
        return *this;
    }

    Action Action::operator/=(const double &t)
    {
        x /= t, y /= t, z /= t;
        return *this;
    }

    // double Action::normxy(const Action &a1, const Action &a2)
    // {
    //     return sqrt((a1.x - a2.x) * (a1.x - a2.x) + (a1.y - a2.y) * (a1.y - a2.y));
    // }

    double Action::height()
    {
        return z;
    }

    double Action::length()
    {
        return sqrt(x * x + (y + ARM_P) * (y + ARM_P));
    }

    bool Action::near(const Action &t) const
    {
        return abs(x - t.x) < 0.5 && abs(y - t.y) < 0.5 && abs(z - t.z) < 0.5;
    }

    Axis::Axis() : expand_y(false) {}

    double Axis::L(double alpha)
    {
        return length() - ARM_A5 * Angle(alpha).cos() - ARM_A4 * Angle(alpha).sin() - ARM_A0;
    }

    double Axis::H(double alpha)
    {
        return height() + ARM_A5 * Angle(alpha).sin() - ARM_A4 * Angle(alpha).cos() - ARM_A1;
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
            ROS_WARN_STREAM("length " << length() << " is out of range " << ARM_MAX_LEN << ".");
            return true;
        }
        return false;
    }

    Angle Axis::_calculate_j1()
    {
        Angle j1 = Angle::atan2(y + ARM_P, x);
        if (j1.get_degree() < 0)
            j1 = j1 + Angle(360);
        return j1;
    };

    Angle Axis::_calculate_j3(double alpha)
    {
        double L = this->H(alpha);
        double H = this->L(alpha);
        // ROS_INFO_STREAM("L:" << L);
        // ROS_INFO_STREAM("H:" << H);
        double cos3 = (L * L + H * H - ARM_A2 * ARM_A2 - ARM_A3 * ARM_A3) / (2 * ARM_A2 * ARM_A3);
        if (cos3 * cos3 > 1)
        {
            Angle j3 = Angle(0);
            j3._set_state(j3.error);
            return j3;
        }
        double sin3 = std::sqrt(1 - cos3 * cos3);
        return Angle::atan2(sin3, cos3);
    }

    Angle Axis::_calculate_j2(double alpha)
    {
        Angle j3 = _calculate_j3(alpha);
        double L = this->L(alpha);
        double H = this->H(alpha);
        double K1 = ARM_A2 + ARM_A3 * j3.cos();
        double K2 = ARM_A3 * j3.sin();
        return (Angle::atan2(L, H) - Angle::atan2(K2, K1));
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
        // ROS_INFO_STREAM("j3:" << j3.get_degree() << "j2:" << j2.get_degree());
        return j1._valid_j(1) && j2._valid_j(2) && j3._valid_j(3) && j4._valid_j(4) && !(_out_of_range());
    }

    bool Axis::_modify_xy()
    {
        static bool flag = false;
        static double last_x = 0;
        static double last_y = 0;
        if (flag)
        {
            x = last_x;
            y = last_y;
            flag = false;
        }
        else if (expand_y && (length() < ARM_A0))
        {
            last_x = x;
            last_y = y;
            double l = length();
            y = 2 * (-ARM_P - ARM_A0 * (y + ARM_P) / l) - y;
            x = -2 * ARM_A0 * x / l - x;
            flag = true;
        }
        return (expand_y && (length() < ARM_A0)) || flag;
    }

    bool Axis::_modify_alpha(double &alpha, bool look)
    {
        // const int ALPHA_MAX = 110; // alpha上限，超过此值无法抓取
        const int ALPHA_MAX = 115; // alpha上限，超过此值无法抓取
        bool valid = false;
        while (alpha >= 0 && alpha <= ALPHA_MAX && !valid)
        {
            valid = _j123_length_and_height_is_valid(alpha);
            if (!valid)
                alpha = look ? alpha - 1 : alpha + 1;
        }
        if (!valid && !look)
        {
            // 当invalid输出alpha用
            do
            {
                alpha++;
            } while (alpha <= 180 && !_j123_length_and_height_is_valid(alpha));
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
        double alpha = 90;
        bool valid = _modify_alpha(alpha, look);
        // ROS_INFO_STREAM("alpha: " << alpha);
        if (valid)
        {
            Angle j1 = _calculate_j1();
            Angle j2 = flag ? (-_calculate_j2(alpha)) : _calculate_j2(alpha);
            Angle j3 = flag ? (-_calculate_j3(alpha)) : _calculate_j3(alpha);
            Angle j4 = flag ? (-_calculate_j4(alpha)) : _calculate_j4(alpha);
            j1._j_degree_convert(1);
            j2._j_degree_convert(2);
            j3._j_degree_convert(3);
            j4._j_degree_convert(4);
            deg1 = j1.get_degree();
            deg2 = j2.get_degree();
            deg3 = j3.get_degree();
            deg4 = j4.get_degree();
        }
        else
            ROS_WARN("Invalid alpha: %lf", alpha);
        _modify_xy();
        // ROS_ERROR_STREAM("valid:" << valid << " deg1:" << deg1 << " deg2:" << deg2 << " deg3:" << deg3 << " deg4:" << deg4);
        return valid;
    }

    bool Axis::forward_kinematics(double &deg1, double &deg2, double &deg3, double &deg4,
                                  double &x, double &y, double &z, bool expand_y)
    {
        bool valid = false;
        Angle j1 = Angle(deg1);
        Angle j2 = Angle(deg2);
        Angle j3 = Angle(deg3);
        Angle j4 = Angle(deg4);
        if (!(j1._valid_degree(1) && j2._valid_degree(2) && j3._valid_degree(3) && j4._valid_degree(4)))
        {
            if (!j1._valid_degree(1))
                ROS_WARN_STREAM("joint 1 has invalid deg " << j1.get_degree());
            if (!j2._valid_degree(2))
                ROS_WARN_STREAM("joint 2 has invalid deg " << j2.get_degree());
            if (!j3._valid_degree(3))
                ROS_WARN_STREAM("joint 3 has invalid deg " << j3.get_degree());
            if (!j4._valid_degree(4))
                ROS_WARN_STREAM("joint 4 has invalid deg " << j4.get_degree());
            return valid;
        }
        j1._j_degree_convert(1);
        j2._j_degree_convert(2);
        j3._j_degree_convert(3);
        j4._j_degree_convert(4);
        double length = ARM_A0 + ARM_A2 * j2.sin() + ARM_A3 * (j2 + j3).sin() +
                        ARM_A4 * (j2 + j3 + j4).sin() + ARM_A5 * (j2 + j3 + j4).cos();
        double height = ARM_A1 + ARM_A2 * j2.cos() + ARM_A3 * (j2 + j3).cos() +
                        ARM_A4 * (j2 + j3 + j4).cos() - ARM_A5 * (j2 + j3 + j4).sin();
        double alpha = (j2 + j3 + j4).get_degree();
        // ROS_ERROR("j2:%lf j3:%lf j4:%lf", j2.get_degree(), j3.get_degree(), j4.get_degree());
        z = height;
        x = length * j1.cos();
        y = length * j1.sin() - ARM_P;
        valid = (0 <= y || expand_y);
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
                ROS_WARN("forward_kinematics: Result invalid!");
                return false;
            }
            else if (!near(Action(tx, ty, tz)))
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
