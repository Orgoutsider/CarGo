#include "motion_controller/field_guide.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace motion_controller
{
  FieldGuide::FieldGuide()
      : route_({route_QR_code_board, route_raw_material_area, route_border,
                route_roughing_area, route_border, route_semi_finishing_area,
                route_border, route_raw_material_area, route_border,
                route_roughing_area, route_border, route_semi_finishing_area, route_border,
                route_parking_area}),
        dr_route_(route_rest),
        doing_(false), where_(0),
        x_(0), y_(0), theta_(0), loop_(0), // length_field_(2.26)
        length_car_(0.29), width_car_(0.27), width_road_(0.45), length_field_(2.29), width_field_(2.03),
        y_QR_code_board_(0.8), x_QR_code_board_(0.02),
        y_raw_material_area_(1.665), angle_raw_material_area_(0.715584993), radius_raw_material_area_(0.15),
        x_roughing_area_(1.22), length_from_semi_finishing_area_(1.045),
        y_semi_finishing_area_(length_field_ - length_from_semi_finishing_area_),
        length_from_ellipse_(0.028),
        width_from_semi_finishing_area_(0.295), width_from_roughing_area_(0.29),
        length_from_parking_area_(0.396), x_road_up_(0.08), x_parking_area_(0.58),
        clockwise_(false)
  {
  }

  int FieldGuide::where_is_car(bool debug, bool startup, int offset) const
  {
    if (!debug)
      return route_[(where_ + offset >= route_.size())
                        ? (route_.size() - 1)
                        : ((where_ + offset < 0) ? 0 : (where_ + offset))];
    else if (startup)
      return dr_route_;
    else
      return route_rest;
  }

  void FieldGuide::doing(boost::recursive_mutex &mtx)
  {
    boost::lock_guard<boost::recursive_mutex> lk(mtx);
    doing_ = true;
  }

  void FieldGuide::finished(boost::recursive_mutex &mtx)
  {
    boost::lock_guard<boost::recursive_mutex> lk(mtx);
    if (where_ + 1 < route_.size())
    {
      if (loop_ == 0 && !clockwise_ && where_is_car(false) == route_semi_finishing_area)
        clockwise_ = true;
      else if (loop_ == 1 && clockwise_ && where_is_car(false) == route_raw_material_area)
        clockwise_ = false;
      if (loop_ == 0 && where_is_car(false) == route_semi_finishing_area)
        loop_++;
      where_++;
      doing_ = false;
    }
  }

  bool FieldGuide::is_doing() const
  {
    return doing_;
  }

  bool FieldGuide::arrived(bool debug, bool startup) const
  {
    if (doing_)
      return false;
    switch (where_is_car(debug, startup))
    {
    case route_QR_code_board:
      return length_route(debug, startup) < 0.1 && -x_ < x_road_up_ + width_road_;

    case route_raw_material_area:
      if (loop_ == 0)
        return length_route(debug, startup) < 0.1 &&
               -x_ < x_road_up_ + width_road_;
      else if (loop_ == 1)
        return -length_route(debug, startup) < 0.1 &&
               y_ > length_field_ - width_road_;
      else
        ROS_ERROR("Invalid loop!");
      return false;

    case route_roughing_area:
      return length_route(debug, startup) < 0.1 && y_ > length_field_ - width_road_;

    case route_semi_finishing_area:
      return length_route(debug, startup) < 0.1 && -x_ > x_road_up_ + width_field_ - width_road_;

    case route_parking_area:
      return length_route(debug, startup) < 0.1 && y_ < width_road_;

    case route_border:
      return length_route(debug, startup) * (clockwise_ ? -1 : 1) < 0.1;

    default:
      ROS_ERROR_ONCE("where_is_car returns invalid value!");
      return false;
    }
  }

  double FieldGuide::length_from_road(bool debug, bool startup, int offset) const
  {
    // 需要考虑车所处的方位，考虑车子xy正向
    switch (where_is_car(debug, startup, offset))
    {
    case route_QR_code_board:
      return -(x_road_up_ + width_road_ - x_QR_code_board_ - length_car_ / 2 - (-x_));

    case route_raw_material_area:
      return (!doing_ && loop_ == 1)
                 ? length_field_ - width_road_ / 2 - y_
                 : -(x_road_up_ + width_road_ / 2 - (-x_));

    case route_roughing_area:
      return length_field_ - width_road_ / 2 - y_;

    case route_semi_finishing_area:
      return x_road_up_ + width_field_ - width_road_ / 2 - (-x_);

    case route_parking_area:
      return width_road_ / 2 - y_;

    default:
      ROS_WARN("Attempted to use 'length_from_road' in route %d.", where_is_car(false));
      return 0;
    }
  }

  double FieldGuide::angle_from_road(bool debug, bool startup, int offset) const
  {
    double theta;
    // 需要考虑车所处的方位，考虑车子theta正向
    switch (where_is_car(debug, startup, offset))
    {
    case route_QR_code_board:
      return -theta_;
      break;

    case route_raw_material_area:
      return -theta_;

    case route_roughing_area:
      return -theta_;

    case route_semi_finishing_area:
      return -(theta_ - M_PI / 2);

    case route_parking_area:
      return -theta_;

    case route_border:
      theta = angle_correction(theta_);
      // 所得结果正好是旋转角度但要限制在-pi到pi之间
      theta = (theta > M_PI) ? theta - 2 * M_PI : (theta <= -M_PI ? theta + 2 * M_PI : theta);
      return theta;

    default:
      ROS_WARN("Attempted to use 'angle_from_road' in route %d.", where_is_car(false));
      return 0;
    }
  }

  bool FieldGuide::position_in_corner(double dist, double yaw,
                                      double &x, double &y, double &theta, bool outside) const
  {
    ROS_INFO("Before setting: x: %lf y:%lf theta:%lf", x_, y_, theta_);
    int n = 0, sign = outside ? 1 : -1;
    yaw = angle_correction(yaw);
    theta = theta_;
    if (-x_ < x_road_up_ + width_road_) // 上
      n += 1;
    else if (-x_ > x_road_up_ + width_field_ - width_road_) // 下
      n += 2;
    if (y_ < width_road_) // 右
      n += 5;
    else if (y_ > length_field_ - width_road_) // 左
      n += 10;
    switch (n)
    {
    case 1:
      x = -x_road_up_ - width_road_ / 2 - sign * dist * cos(yaw);
      y = y_;
      break;
    case 2:
      x = -x_road_up_ - width_field_ + width_road_ / 2 + sign * dist * cos(yaw);
      y = y_;
      break;
    case 5:
      x = x_;
      y = width_road_ / 2 + sign * dist * cos(yaw);
      break;
    case 10:
      x = x_;
      y = length_field_ - width_road_ / 2 - sign * dist * cos(yaw);
      break;
    case 6:
      if (clockwise_)
      {
        x = x_;
        y = width_road_ / 2 + sign * dist * cos(yaw);
      }
      else
      {
        x = -x_road_up_ - width_road_ / 2 - sign * dist * cos(yaw);
        y = y_;
      }
      break;
    case 11:
      if (!clockwise_)
      {
        x = x_;
        y = length_field_ - width_road_ / 2 - sign * dist * cos(yaw);
      }
      else
      {
        x = -x_road_up_ - width_road_ / 2 - sign * dist * cos(yaw);
        y = y_;
      }
      break;
    case 7:
      if (!clockwise_)
      {
        x = x_;
        y = width_road_ / 2 + sign * dist * cos(yaw);
      }
      else
      {
        x = -x_road_up_ - width_field_ + width_road_ / 2 + sign * dist * cos(yaw);
        y = y_;
      }
      break;
    case 12:
      if (clockwise_)
      {
        x = x_;
        y = length_field_ - width_road_ / 2 - sign * dist * cos(yaw);
      }
      else
      {
        x = -x_road_up_ - width_field_ + width_road_ / 2 + sign * dist * cos(yaw);
        y = y_;
      }
      break;
    default:
      ROS_ERROR("position_in_corner: Car's postion is abnormal, n: %d", n);
      return false;
    }
    ROS_INFO("After setting: x: %lf y:%lf theta:%lf", x, y, theta);
    return true;
  }

  // bool FieldGuide::can_turn() const
  // {
  //   return (loop_ == 1 && y_ > x_road_up_ + width_road_ && where_is_car(false) == route_raw_material_area);
  // }

  double FieldGuide::length_route(bool debug, bool startup, int offset) const
  {
    switch (where_is_car(debug, startup, offset))
    {
    case route_QR_code_board:
      return y_QR_code_board_ - y_;

    case route_raw_material_area:
      if (loop_ == 0)
        return y_raw_material_area_ - y_ -
               (radius_raw_material_area_ + width_road_ / 2 +
                length_from_road(debug, startup, offset)) *
                   tan(angle_raw_material_area_);
      else if (loop_ == 1)
        return x_road_up_ + width_road_ / 2 - (-x_);
      else
        ROS_ERROR("Invalid loop!");
      return false;

    case route_roughing_area:
      return x_roughing_area_ - (-x_);

    case route_semi_finishing_area:
      return -(y_semi_finishing_area_ - y_);

    case route_parking_area:
      return -(x_parking_area_ - (-x_));

    case route_border:
      switch (where_is_car(debug, startup, offset + 1))
      {
      case route_roughing_area:
        return length_field_ - width_road_ / 2 - y_;

      case route_semi_finishing_area:
        return x_road_up_ + width_field_ - width_road_ / 2 - (-x_);

      case route_raw_material_area:
        return -(length_field_ - width_road_ / 2 - y_);

      case route_parking_area:
        return -(width_road_ / 2 - y_);

      default:
        ROS_WARN("Attempted to use 'length_route' in route %d.",
                 where_is_car(false, false, offset + 1));
        return 0.5;
      }

    default:
      ROS_ERROR_ONCE("where_is_car returns invalid value!");
      return 0.5;
    }
  }

  // double FieldGuide::length_border() const
  // {
  //   double len0 = 0.5, len1 = 0.5;
  //   if (-x_ < x_road_up_ + width_road_) // 上
  //     len0 = -x_ - x_road_up_ - width_road_ / 2;
  //   else if (-x_ > x_road_up_ + width_field_ - width_road_) // 下
  //     len0 = -(-x_ - x_road_up_ - width_field_ + width_road_ / 2);
  //   if (y_ < width_road_) // 右
  //     len1 = y_ - width_road_ / 2;
  //   else if (y_ > length_field_ - width_road_) // 左
  //     len1 = -(y_ - length_field_ + width_road_ / 2);
  //   return abs(len0) > abs(len1) ? len0 : len1;
  // }

  double FieldGuide::angle_corner() const
  {
    if (where_is_car(false, false, -1) == route_roughing_area)
      return M_PI / 2 - theta_;
    else if (where_is_car(false, false, -1) == route_semi_finishing_area)
      return -theta_;
    else
    {
      ROS_WARN("Attempted to use 'angle_corner' in route %d.", where_is_car(false, false, -1));
      return 0;
    }
  }

  double FieldGuide::angle_correction(double theta) const
  {
    if (theta_ > M_PI * 3 / 4 || theta_ <= -M_PI * 3 / 4)
      theta = M_PI - theta;
    else if (theta_ > M_PI / 4)
      theta = M_PI / 2 - theta;
    else if (theta_ > -M_PI / 4)
      theta = -theta;
    else
      theta = -M_PI / 2 - theta;
    return theta;
  }
} // namespace motion_controller
