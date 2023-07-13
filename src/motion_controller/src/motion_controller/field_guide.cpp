#include "motion_controller/field_guide.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace motion_controller
{
  FieldGuide::FieldGuide()
      : route_({route_QR_code_board, route_raw_material_area,
                route_roughing_area, route_semi_finishing_area, route_raw_material_area,
                route_roughing_area, route_semi_finishing_area, route_parking_area}),
        doing_(false), where_(0), left_(true),
        x_(0), y_(0), theta_(0), loop_(0),
        length_car_(0.296), width_road_(0.45), length_field_(2),
        x_QR_code_board_(0.8), x_raw_material_area_(1.59),
        y_roughing_area_(1.15), x_semi_finishing_area_(1.2),
        length_parking_area_(0.3), y_road_up_up_(0.078), y_parking_area_(0.7) {}

  Route FieldGuide::where_is_car()
  {
    return route_[where_];
  }

  void FieldGuide::doing()
  {
    doing_ = true;
  }

  void FieldGuide::finish()
  {
    if (where_ + 1 < route_.size())
    {
      if (loop_ == 0 && where_is_car() == route_semi_finishing_area)
        loop_++;
      where_++;
      doing_ = false;
    }
    else
      ROS_WARN("route_ is out of range!");
  }

  bool FieldGuide::arrive()
  {
    if (doing_)
      return false;
    switch (where_is_car())
    {
    case route_QR_code_board:
      return x_ > x_QR_code_board_ - 0.1 && y_ < y_road_up_up_ + width_road_ - length_car_ / 2;
    
    case route_raw_material_area:
      return abs(x_ - x_raw_material_area_) < 0.1 && y_ < y_road_up_up_ + width_road_ - length_car_ / 2;
    
    case route_roughing_area:
      return y_ > y_roughing_area_ - 0.1 && x_ > length_field_ - width_road_ + length_car_ / 2;

    case route_semi_finishing_area:
      return x_ < x_semi_finishing_area_ + 0.1 && y_ > y_road_up_up_ + length_field_ - width_road_ + length_car_ / 2;

    case route_parking_area:
      return y_ < y_parking_area_ + 0.1 && x_ < width_road_ - length_car_ / 2;
    
    default:
      ROS_ERROR("where_is_car returns invalid value!");
      return false;
    }
  }

  bool FieldGuide::can_turn()
  {
    return (loop_ == 1 && y_ > y_road_up_up_ + width_road_ && where_is_car() == route_raw_material_area);
  }

  double FieldGuide::length_route()
  {
    switch (where_is_car())
    {
    case route_QR_code_board:
      return abs(x_QR_code_board_ - x_);

    case route_raw_material_area:
      return abs(x_raw_material_area_ - x_);

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

  double FieldGuide::length_corner()
  {
    int n = 0;
    if (y_ < y_road_up_up_ + width_road_ - length_car_ / 2) // 上
      n += 1;
    else if (y_ > y_road_up_up_ + length_field_ - width_road_ + length_car_ / 2) // 下
      n += 2;
    if (x_ < width_road_ - length_car_ / 2) // 右
      n += 5;
    else if (x_ > length_field_ - width_road_ + length_car_ / 2) // 左
      n += 10;
    switch (n)
    {
    case 6:
      if (left_)
        return y_ - (y_road_up_up_ + width_road_ / 2);
      else
        return x_ - width_road_ / 2;
    
    case 11:
      if (left_)
        return (length_field_ - width_road_ / 2) - x_;
      else
        return y_ - (y_road_up_up_ + width_road_ / 2);

    case 12:
      if (left_)
        return (y_road_up_up_ + length_field_ - width_road_ / 2) - y_;
      else
        return (length_field_ - width_road_ / 2) - x_;

    case 7:
      if (left_)
        return x_ - width_road_ / 2;
      else 
        return (y_road_up_up_ + length_field_ - width_road_ / 2) - y_;

    default:
      ROS_WARN("Car is not in the corner. Do not use length_corner.");
      return 0;
    }
  }

  double FieldGuide::angle_corner()
  {
    int n = 0;
    if (y_ < y_road_up_up_ + width_road_) // 上
      n += 1;
    else if (y_ > y_road_up_up_ + length_field_ - width_road_) // 下
      n += 2;
    if (x_ < width_road_) // 右
      n += 5;
    else if (x_ > length_field_ - width_road_) // 左
      n += 10;
    switch (n)
    {
    case 6:
      if (left_)
        return -theta_;
      else
        return -(theta_ - M_PI / 2);
    
    case 11:
      if (left_)
        return M_PI / 2 - theta_;
      else
        return -(M_PI + theta_);

    case 12:
      if (left_)
        return M_PI - theta_;
      else
        return -(M_PI / 2 + theta_);

    case 7:
      if (left_)
        return -theta_ - M_PI / 2;
      else 
        return -theta_;

    default:
      ROS_WARN("Car is not in the corner. Do not use angle_corner.");
      return 0;
    }
  }

  double FieldGuide::angle_U_turn()
  {
    if ((where_is_car() == route_semi_finishing_area && loop_ == 0) || (where_is_car() == route_raw_material_area && loop_ == 1))
    {
      return -theta_;
    }
    ROS_WARN("Cannot use angle_U_turn here!");
    return 0;
  }
} // namespace motion_controller
