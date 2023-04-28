/*
    需求:
        编写两个节点实现服务通信，客户端节点需要提交两个整数到服务器
        服务器需要解析客户端提交的数据，相加后，将结果响应回客户端，
        客户端再解析

    服务器实现:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 ROS 句柄
        4.创建 客户端 对象
        5.请求服务，接收响应

*/
// 1.包含头文件
// #include ""

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <vision_msgs/BoundingBox2DArray.h>

#include <yolov5_ros/cargoSrv.h>
cv::RNG rngs = { 12345 };
class ImageListener
{
public:
    ros::ServiceClient client;
    cv_bridge::CvImagePtr cv_image;
    void imageCallback(const sensor_msgs::ImageConstPtr& image_rect)
    {
        try
		{
			cv_image = cv_bridge::toCvCopy(image_rect, "bgr8");
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		if (cv_image->image.empty())
		{
			ROS_ERROR("No data!");
			return;
		}
        // 5.组织请求数据
        yolov5_ros::cargoSrv cargo;
        // cv::cvtColor(cv_image->image, cv_image->image, cv::COLOR_RGB2BGR);
        cargo.request.image = *image_rect;
        // 6.发送请求,返回 bool 值，标记是否成功
        bool flag = client.call(cargo);
        // 7.处理响应
        if (flag)
        {
            ROS_INFO("请求正常处理");
            // if (cargo.response.results.boxes.size())
            // {
            //     for (int color = 1; color <= 3; color++)
            //     {
            //         if (!cargo.response.results.boxes[color].center.x)
            //             continue;
            //         cv::RotatedRect rect(cv::Point2f(cargo.response.results.boxes[color].center.x, cargo.response.results.boxes[color].center.y), 
            //                             cv::Size2f(cargo.response.results.boxes[color].size_x, cargo.response.results.boxes[color].size_y), 
            //                             cargo.response.results.boxes[color].center.theta); 
            //         cv::Point2f vtx[4];//矩形顶点容器
            //         // cv::Mat dst = cv::Mat::zeros(cv_image->image.size(), CV_8UC3);//创建空白图像
            //         rect.points(vtx);//确定旋转矩阵的四个顶点
            //         cv::Scalar colors = cv::Scalar(rngs.uniform(0, 255), rngs.uniform(0, 255), rngs.uniform(0, 255)); //创建随机颜色，便于区分
            //         for (int j = 0; j < 4;j++)
            //         {
            //             cv::line(cv_image->image, vtx[j], vtx[(j + 1) % 4], colors, 2);//随机颜色绘制矩形
            //         }

            //     }

            // }
		    // cv::imshow("COLOR_Result", cv_image->image);
            // cv::waitKey(100);
        }
    }
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    // 2.初始化 ROS 节点
    ros::init(argc, argv, "my_hand_eye_client_node");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ImageListener imgl;
    // 4.创建 客户端 对象
    imgl.client = nh.serviceClient<yolov5_ros::cargoSrv>("cargoSrv");
    // 等待服务启动成功
    // 方式1
    ros::service::waitForService("cargoSrv");
    // 方式2
    // client.waitForExistence();
    image_transport::ImageTransport it(nh);
    std::string transport_hint;
    pnh.param<std::string>("transport_hint", transport_hint, "raw");
    image_transport::Subscriber camera_image_subscriber =
        it.subscribe("/eye/image_rect_color", 1, &ImageListener::imageCallback, &imgl, image_transport::TransportHints(transport_hint));
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
