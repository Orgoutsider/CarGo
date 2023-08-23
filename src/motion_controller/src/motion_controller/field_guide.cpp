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
        x_(0), y_(0), theta_(0), loop_(0),
        length_car_(0.28), width_car_(0.271), width_road_(0.45), length_field_(2),
        y_QR_code_board_(0.8), x_QR_code_board_(0.02),
        y_raw_material_area_(1.6), angle_raw_material_area_(0.715584993), radius_raw_material_area_(0.15),
        y_roughing_area_(1.15), x_semi_finishing_area_(1.2),
        length_parking_area_(0.3), x_road_up_(0.08), y_parking_area_(0.7) {}

  int FieldGuide::where_is_car(bool debug, bool startup) const
  {
    if (!debug)
      return route_[where_];
    else if (startup)
      return dr_route_;
    else
      return route_rest;
  }

  void FieldGuide::doing()
  {
    doing_ = true;
  }

  void FieldGuide::finished()
  {
    if (where_ + 1 < route_.size())
    {
      if (loop_ == 0 && where_is_car(false) == route_semi_finishing_area)
        loop_++;
      where_++;
      doing_ = false;
    }
    else
      ROS_WARN("route_ is out of range!");
  }

  bool FieldGuide::is_doing() const
  {
    return doing_;
  }

  bool FieldGuide::arrived(bool debug, bool startup) const
  {
    // ROS_INFO_STREAM(doing_ << abs(y_ - y_raw_material_area_ +
    //                               (radius_raw_material_area_ + width_road_ / 2) * tan(angle_raw_material_area_))
    //                        << x_ + x_road_up_ + width_road_ - length_car_ / 2);
    if (doing_)
      return false;
    switch (where_is_car(debug, startup))
    {
    case route_QR_code_board:
      return y_ > y_QR_code_board_ - 0.1 && x_ > -(x_road_up_ + width_road_ - length_car_ / 2);

    case route_raw_material_area:
      return abs(y_ - y_raw_material_area_ +
                 (radius_raw_material_area_ + width_road_ / 2) * tan(angle_raw_material_area_)) < 0.1 &&
             x_ > -(x_road_up_ + width_road_ - length_car_ / 2);

    case route_roughing_area:
      return y_ > y_roughing_area_ - 0.1 && x_ > length_field_ - width_road_ + length_car_ / 2;

    case route_semi_finishing_area:
      return x_ < x_semi_finishing_area_ + 0.1 && y_ > x_road_up_ + length_field_ - width_road_ + length_car_ / 2;

    case route_parking_area:
      return y_ < y_parking_area_ + 0.1 && x_ < width_road_ - length_car_ / 2;

    default:
      ROS_ERROR_ONCE("where_is_car returns invalid value!");
      return false;
    }
  }

  double FieldGuide::length_from_road() const
  {
    // 需要考虑车所处的方位，考虑车子xy正向
    switch (where_is_car(false))
    {
    case route_QR_code_board:
      return -(x_road_up_ + width_road_ / 2 - (-x_));

    case route_raw_material_area:
      return -(x_road_up_ + width_road_ / 2 - (-x_));

    case route_roughing_area:
      return length_field_ - width_road_ / 2 - y_;

    case route_semi_finishing_area:
      return x_road_up_ + length_field_ - width_road_ / 2 - (-x_);

    case route_parking_area:
      return width_road_ / 2 - y_;

    default:
      ROS_WARN("Attempted to use 'length_from_road' in route %d.", where_is_car(false));
      return 0;
    }
  }

  double FieldGuide::angle_from_road() const
  {
    // 需要考虑车所处的方位，考虑车子theta正向
    switch (where_is_car(false))
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

    default:
      ROS_WARN("Attempted to use 'angle_from_road' in route %d.", where_is_car(false));
      return 0;
    }
  }

  bool FieldGuide::can_turn() const
  {
    return (loop_ == 1 && y_ > x_road_up_ + width_road_ && where_is_car(false) == route_raw_material_area);
  }

  double FieldGuide::length_route() const
  {
    switch (where_is_car(false))
    {
    case route_QR_code_board:
      return abs(y_QR_code_board_ - x_);

    case route_raw_material_area:
      return abs(y_raw_material_area_ - x_);

    case route_roughing_area:
      return abs(y_roughing_area_ - y_);

    case route_semi_finishing_area:
      return abs(x_semi_finishing_area_ - x_);

    case route_parking_area:
      return abs(y_parking_area_ - y_);

    default:
      ROS_ERROR("where_is_car returns invalid value!");
      return 0.5;
    }
  }

  // double FieldGuide::length_corner() const
  // {
  //   int n = 0;
  //   if (y_ < x_road_up_ + width_road_ - length_car_ / 2) // 上
  //     n += 1;
  //   else if (y_ > x_road_up_ + length_field_ - width_road_ + length_car_ / 2) // 下
  //     n += 2;
  //   if (x_ < width_road_ - length_car_ / 2) // 右
  //     n += 5;
  //   else if (x_ > length_field_ - width_road_ + length_car_ / 2) // 左
  //     n += 10;
  //   switch (n)
  //   {
  //   case 6:
  //     if (left_)
  //       return y_ - (x_road_up_ + width_road_ / 2);
  //     else
  //       return x_ - width_road_ / 2;

  //   case 11:
  //     if (left_)
  //       return (length_field_ - width_road_ / 2) - x_;
  //     else
  //       return y_ - (x_road_up_ + width_road_ / 2);

  //   case 12:
  //     if (left_)
  //       return (x_road_up_ + length_field_ - width_road_ / 2) - y_;
  //     else
  //       return (length_field_ - width_road_ / 2) - x_;

  //   case 7:
  //     if (left_)
  //       return x_ - width_road_ / 2;
  //     else
  //       return (x_road_up_ + length_field_ - width_road_ / 2) - y_;

  //   default:
  //     ROS_WARN("Car is not in the corner. Do not use length_corner.");
  //     return 0;
  //   }
  // }

  // double FieldGuide::angle_corner() const
  // {
  //   int n = 0;
  //   if (y_ < x_road_up_ + width_road_) // 上
  //     n += 1;
  //   else if (y_ > x_road_up_ + length_field_ - width_road_) // 下
  //     n += 2;
  //   if (x_ < width_road_) // 右
  //     n += 5;
  //   else if (x_ > length_field_ - width_road_) // 左
  //     n += 10;
  //   switch (n)
  //   {
  //   case 6:
  //     if (left_)
  //       return -theta_;
  //     else
  //       return -(theta_ - M_PI / 2);

  //   case 11:
  //     if (left_)
  //       return M_PI / 2 - theta_;
  //     else
  //       return -(M_PI + theta_);

  //   case 12:
  //     if (left_)
  //       return M_PI - theta_;
  //     else
  //       return -(M_PI / 2 + theta_);

  //   case 7:
  //     if (left_)
  //       return -theta_ - M_PI / 2;
  //     else
  //       return -theta_;

  //   default:
  //     ROS_WARN("Car is not in the corner. Do not use angle_corner.");
  //     return 0;
  //   }
  // }

  // double FieldGuide::angle_U_turn() const
  // {
  //   if ((where_is_car(false) == route_semi_finishing_area && loop_ == 0) ||
  //       (where_is_car(false) == route_raw_material_area && loop_ == 1))
  //   {
  //     return -theta_;
  //   }
  //   ROS_WARN("Cannot use angle_U_turn here!");
  //   return 0;
  // }
} // namespace motion_controller
