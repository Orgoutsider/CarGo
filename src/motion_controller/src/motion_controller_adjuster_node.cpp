#include "motion_controller/vision_adjuster.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motion_controller_adjuster_node");
    motion_controller::VisionAdjuster adjuster;

    ros::spin();
    return 0;
}
