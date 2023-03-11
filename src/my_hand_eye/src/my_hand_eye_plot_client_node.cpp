#include "my_hand_eye/arm_controller.h"

my_hand_eye::ArmController controller;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "my_hand_eye_plot_client_node");
    ros::NodeHandle nh;
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::NodeHandle pnh("~");
    controller.init(nh, pnh, true);
    // double h = 22.5;
    double h = 20.5;
    for (h = 0.5; h < ARM_MAX_HIGH - 0.2; h += 0.2)
    {
        if (h < ARM_MAX_HIGH && h > 0)
            controller.find_points_with_height(h, false, true);
    }
    controller.find_points_with_height(h, true, true);
    // controller.find_points_with_height(controller.z_turntable, true, true);
    return 0;
}