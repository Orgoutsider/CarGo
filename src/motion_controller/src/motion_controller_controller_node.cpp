#include "motion_controller/motion_controller.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motion_controller_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    motion_controller::MotionController controller(nh, pnh);
    
    ros::spin();
    return 0;
}
