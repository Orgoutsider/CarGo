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
    controller.init(nh, pnh);
    // double h = 22.5;
    double h = 32.23;
    // for (h = 0; h < ARM_MAX_HIGH-10; h += 0.2)
    for (h = 0+3.5; h < 7+3.5; h += 0.2)
    {
        if (h < ARM_MAX_HIGH && h > 0)
            controller.find_points_with_height(h, false, true);
    }
    controller.find_points_with_height(h, true, true);
    // controller.find_points_with_height(controller.z_turntable, true, true);
    return 0;
}