#ifndef _MOTION_CONTROLLER_H_
#define _MOTION_CONTROLLER_H_

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <my_hand_eye/ArmAction.h>
#include <motion_controller/MoveAction.h>
#include <motion_controller/Distance.h>
#include <motion_controller/Go.h>

#include "motion_controller/line_follower.h"
#include "motion_controller/field_guide.h"

namespace motion_controller
{
    typedef actionlib::SimpleActionClient<motion_controller::MoveAction> MoveClient;

    typedef actionlib::SimpleActionClient<my_hand_eye::ArmAction> ArmClient;

    class MotionController : public FieldGuide
    {
    private:
        double delta_x_; // 设置位置时，真实坐标与tf坐标转换关系的偏移量
        double delta_y_;
        double delta_theta_;
        bool finish_turning_; // 转弯之后关闭弯道视觉订阅
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;
        // ros::Publisher vision_publisher;            // 视觉信息发布者
        ros::Timer timer_;                           // 全局定位定时器
        LineFollower follower_;                      // 走直线
        MoveClient ac_move_;                         // 移动服务客户端
        ArmClient ac_arm_;                           // 机械臂客户端
        ros::ServiceServer go_client_;               // 一键式客户端
        geometry_msgs::Pose2D move_pose_, arm_pose_; // 目标姿态
        ros::Time stamp_;                            // 一直更新的时间戳，用于激活
        ros::Time move_time_, arm_time_;             // 接收时间
        ros::Time move_stamp_, arm_stamp_;           // 感知时间
        bool move_active_, arm_active_;              // 传感器是否活动
        bool move_initialized_, arm_initialized_;    // 传感器已经初始化
        double timeout_;                             // 最大超时
        // 转弯
        bool _turn();
        // 掉头，需要改变之后的转弯方向
        void _U_turn();
        // 通过全局定位信息转弯
        bool _turn_by_position();
        void _timer_callback(const ros::TimerEvent &event);
        void _arm_done_callback(const actionlib::SimpleClientGoalState &state,
                                const my_hand_eye::ArmResultConstPtr &result);
        void _arm_active_callback();
        void _arm_feedback_callback(const my_hand_eye::ArmFeedbackConstPtr &feedback);
        void _move_done_callback(const actionlib::SimpleClientGoalState &state,
                                 const motion_controller::MoveResultConstPtr &result);
        void _move_active_callback();
        void _move_feedback_callback(const motion_controller::MoveFeedbackConstPtr &feedback);

    public:
        MotionController(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        // 设置当前位置
        bool set_position(double x, double y, double theta);
        // 获取当前位置
        bool get_position();
        // 开/关循线回调
        bool go(Go::Request &req, Go::Response &resp);
        // 当move与视觉协同时调用的循环
        bool spin();
    };

} // namespace motion_controller

#endif // !_MOTION_CONTROLLER_H_
