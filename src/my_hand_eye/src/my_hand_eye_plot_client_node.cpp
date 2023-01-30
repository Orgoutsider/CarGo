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
    double h;
    for (h = controller.z_turntable + 5; h > controller.z_turntable - 5; h -= 0.2)
    {
        if (h < ARM_MAX_HIGH && h > 0)
            controller.find_points_with_height(h, false);
    }
    controller.find_points_with_height(h, true);
    return 0;
}