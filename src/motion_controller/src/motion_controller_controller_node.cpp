#include "motion_controller/motion_controller.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "motion_controller_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    motion_controller::MotionController controller(nh, pnh);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    // ros::CallbackQueue callback_queue_t;
    // nh_t.setCallbackQueue(&callback_queue_t);
    // boost::thread spinner_thread_t([&callback_queue_t]()
    //                                {
    //  ros::SingleThreadedSpinner spinner_t;
    //  spinner_t.spin(&callback_queue_t); });
    // ros::spin();
    // spinner_thread_t.join();
    return 0;   
}
