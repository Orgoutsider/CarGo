#ifndef _MOTION_PERFORMER_H_
#define _MOTION_PERFORMER_H_

#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <motion_controller/TwistMightEnd.h>

namespace motion_controller
{
    enum
    {
        level_line,
        level_vision,
        level_service
    };

    class MotionPerformer
    {
    private:
        int level_; // 当前接受的消息优先级，service最高，vision次之
        boost::mutex mtx_;
        ros::Publisher publisher_;
        ros::Subscriber subscriber_line_;
        ros::Subscriber subscriber_vision_;
        ros::Subscriber subscriber_service_;
        // 不同优先级消息的回调函数
        void _line_callback(const geometry_msgs::TwistConstPtr &msg);
        void _vision_callback(const TwistMightEndConstPtr &msg);
        void _service_callback(const TwistMightEndConstPtr &msg);

    public:
        MotionPerformer();
    };

} // namespace motion_controller

#endif // !_MOTION_PERFORMER_H_