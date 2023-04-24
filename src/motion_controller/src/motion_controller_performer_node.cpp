#include "motion_controller/motion_performer.h"

int main(int argc, char *argv[]){
    ros::init(argc, argv, "motion_controller_performer_node");
    motion_controller::MotionPerformer performer;
    ros::MultiThreadedSpinner spiner(3);
    spiner.spin();
    return 0;
}
