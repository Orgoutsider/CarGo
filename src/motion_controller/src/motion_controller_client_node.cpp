#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <motion_controller/MoveAction.h>

typedef actionlib::SimpleActionClient<motion_controller::MoveAction> Client;

// 处理最终结果
void done_cb(const actionlib::SimpleClientGoalState &state, const motion_controller::MoveResultConstPtr &result)
{
    if (state.state_ == state.SUCCEEDED)
    {
        ROS_INFO("Succeeded to move");
    }
    else
    {
        ROS_INFO("Failed to move");
    }
}
// 服务已经激活
void active_cb()
{
    ROS_INFO("Service is active....");
}
// 处理连续反馈
void feedback_cb(const motion_controller::MoveFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("is_paning:" << (bool)feedback->is_paning << " theta:"
                                 << feedback->pose_now.theta << " x:"
                                 << feedback->pose_now.x << " y:" << feedback->pose_now.y);
}

int main(int argc, char *argv[])
{
    // 传参 theta x y
    if (argc < 5)
        return 1;
    ros::init(argc, argv, "motion_controller_client_node");
    ros::NodeHandle nh;
    Client client(nh, "Move", true);
    client.waitForServer();
    motion_controller::MoveGoal goal;
    goal.pose.theta = atof(argv[1]);
    goal.pose.x = atof(argv[2]);
    goal.pose.y = atof(argv[3]);
    goal.precision = atoi(argv[4]);
    client.sendGoal(goal, &done_cb, &active_cb, &feedback_cb);
    ros::spin();
    return 0;
}
