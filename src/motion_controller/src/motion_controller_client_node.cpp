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
}

int main(int argc, char *argv[])
{
    // 传参 theta x y
    if (argc != 4)
        return 1;
    ros::init(argc, argv, "motion_controller_client_node");
    ros::NodeHandle nh;
    Client client(nh, "Move", true);
    client.waitForServer();
    motion_controller::MoveGoal goal;
    goal.pose.theta = atoi(argv[1]);
    goal.pose.x = atoi(argv[2]);
    goal.pose.y = atoi(argv[3]);
    client.sendGoal(goal);
    ros::spin();
    return 0;
}
