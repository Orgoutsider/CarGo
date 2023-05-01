#include "motion_controller/field_guide.h"

namespace motion_controller
{
  FieldGuide::FieldGuide()
      : route_({route_QR_code_board, route_raw_material_area,
                route_roughing_area, route_semi_finishing_area, route_raw_material_area,
                route_roughing_area, route_semi_finishing_area, route_parking_area}),
        doing_(false), where_(0),
        x_(0), y_(0), loop_(0),
        length_car_(0.296), width_road_(0.45),
        x_QR_code_board_(), x_raw_material_area_(1.59),
        y_roughing_area_(1.05), x_semi_finishing_area_(),
        length_parking_area_(0.3), y_road_up_up_(0.078), y_parking_area_() {}

  int FieldGuide::where_is_car()
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
      return x_ > x_QR_code_board_ - 0.1;
    
    case route_raw_material_area:
      return abs(x_ - x_raw_material_area_) < 0.1;
    
    case route_roughing_area:
      return y_ > y_roughing_area_ - 0.1;

    case route_semi_finishing_area:
      return x_ < x_semi_finishing_area_ + 0.1;

    case route_parking_area:
      return y_ < y_parking_area_ + 0.1;
    
    default:
      ROS_ERROR("where_is_car returns invalid value!");
      return false;
    }
  }

  bool FieldGuide::can_turn()
  {
    return (loop_ == 1 && y_ > y_road_up_up_ + width_road_ && where_is_car() == route_parking_area);
  }
} // namespace motion_controller
