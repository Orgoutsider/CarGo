#include "apriltag_ros/common_functions.h"
#include "my_hand_eye/pos.hpp"
#include <apriltag_ros/AnalyzeSingleImage.h>
#include <opencv2/core/eigen.hpp>
#include <fstream>
std::ofstream File1;
std::ofstream File2;
std::ofstream Res;
s16 Position[6];
u16 Speed[6] = {500, 500, 500, 500, 500, 500};
u8 ACC[6] = {10, 10, 10, 10, 10, 10};
SMS_STS sm_st;
SCSCL sc;
nijie::Pos ps(Position, Speed, ACC, &sm_st, &sc);

int num = 0;
const int maxnum = 20;
char text[100] = {};
//指定运动位置
double targetx[30] = {-0.563389, -1.45867, -2.04845, -1.41844, 0.535275, -5.75446, 1.80323, 7.59661e-07, -3.72793, 4.38057, -0.605159, 1.51909, -5.65667, 3.30037, -0.526636, -0.567919, 8.35015e-07, 8.26643e-07, 5.5163, 3.24953-2.21159, 2.25481, -6.94017, 1.64182, 8.03178e-07, -6.86798, -0.482365, -1.42676, -0.96989, 6.19705};
double targety[30] = {24.7766, 20.3331, 21.7942, 19.5654, 23.1659, 22.1041, 18.2874, 20.851, 19.0256, 28.1768, 27.1696, 21.486, 21.601, 23.901, 22.671, 25.0361, 23.6632, 23.3508, 18.4521, 23.4172, 24.1272, 24.7452, 22.5612, 23.8279, 22.475, 22.2485, 20.1347, 19.7243, 20.274, 21.6548};
double targetz[30] = {13.6579, 6.99897, 3.6582, 12.4393, 10.5183, 9.03096, 2.43279, 10.2917, 6.22193, 11.1699, 7.97784, 12.963, 7.78461, 13.3025, 12.2238, 14.553, 8.39071, 14.6011, 4.99748, 12.2275, 10.781, 10.5426, 12.3971, 11.124, 14.2681, 7.75108, 5.37771, 4.59983, 2.15212, 10.975};
std::vector<cv::Mat> Rend2base;
std::vector<cv::Mat> tend2base;
std::vector<cv::Mat> Rboard2camera;
std::vector<cv::Mat> tboard2camera;
cv::Mat SrcImg;

bool getRosParameter(ros::NodeHandle &pnh, std::string name, double &param)
{
  // Write parameter "name" from ROS Parameter Server into param
  // Return true if successful, false otherwise
  if (pnh.hasParam(name.c_str()))
  {
    pnh.getParam(name.c_str(), param);
    ROS_INFO_STREAM("Set camera " << name.c_str() << " = " << param);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Could not find " << name.c_str() << " parameter!");
    return false;
  }
}

// class ImageListener
// {
// public:
//   int num, ok;
//   cv_bridge::CvImagePtr cv_image_;
//   ImageListener() : num(0), ok(0){};
//   void imageCallback(const sensor_msgs::ImageConstPtr &image_rect)
//   {
//     ROS_INFO("do imageCallback...");
//     if (num > maxnum)
//       return;
//     try
//     {
//       cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
//     }
//     catch (cv_bridge::Exception &e)
//     {
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//       return;
//     }
//     if (!(cv_image_->image.data))
//     {
//       ROS_ERROR("No data!");
//       return;
//     }
//     else if (!ok)
//       ok = 1;
//   }
// };

// ImageListener imgl;

void MouseEvent(int event, int x, int y, int flags, void *data)
{
  if (event == cv::EVENT_LBUTTONDBLCLK)
  {
    num++;
    nijie::ArmPose aps = ps.end_to_base_now();
    if (aps.empty)
    {
      ROS_ERROR("Arm's pose is empty!");
      
    }
    cv::Mat Me2b = aps.R;
    cv::Mat Re2b(3, 1, CV_64F);
    cv::Mat te2b = aps.t;
    cv::Rodrigues(Me2b, Re2b);
    Rend2base.push_back(Me2b);
    //将单位从cm转为mm
    tend2base.push_back(te2b * 10.0);
    File2 << "R" << num << ":\n"
          << Me2b << std::endl;
    File2 << "t" << num << ":\n"
          << (te2b * 10.0) << std::endl;
    // imshow("cut", srcImg);
    snprintf(text, sizeof(text), "/home/fu/apriltag_ws/src/my_hand_eye/img/ApriltagRaw_%d.png", num);
    /*
      这里需要重新设置一下双引号里面图片的保存路径，ZZY_%d是图片名称
      %d是标识符，值即为num
      .jpg是文件拓展名，一般为jpg格式就行
    */
    cv::imwrite(text, SrcImg);
    ROS_INFO("第%d张图片保存成功\n", num);
    if (num < maxnum)
      ps.go_to(targetx[num], targety[num], targetz[num], true, true);
  }
}

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  if (argc < 2)
  {
    ROS_ERROR("argc error!");
    return 1;
  }
  ROS_INFO_STREAM("serial:" << argv[1]);
  if (!ps.begin(argv[1]))
    return 1;
  ps.reset();
  ps.go_to(targetx[num], targety[num], targetz[num], true, true);
  cv::VideoCapture USBCamera(202);
  if (!USBCamera.isOpened())
    return 1;
  Rend2base.reserve(20);
  tend2base.reserve(20);
  Rboard2camera.reserve(20);
  tboard2camera.reserve(20);
  File1.open("/home/fu/apriltag_ws/src/my_hand_eye/img/board2camera.txt", std::ios::binary | std::ios::out | std::ios::ate);
  File2.open("/home/fu/apriltag_ws/src/my_hand_eye/img/end2base.txt", std::ios::binary | std::ios::out | std::ios::ate);
  Res.open("/home/fu/apriltag_ws/src/my_hand_eye/img/camera2end.txt", std::ios::binary | std::ios::out | std::ios::ate);
  // 设备编号根据实际而定，一般电脑摄像头默认是0，USB 1、2、3等都有可能，可以修改尝试
  //USBCamera = cv::VideoCapture(202);
 
  //需要在这里重新设置读取的视频宽高，opencv默认640*480
  USBCamera.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
  USBCamera.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
  ROS_INFO_STREAM("\n视频的宽为：" << USBCamera.get(cv::CAP_PROP_FRAME_WIDTH) << "\n视频的高为：" << USBCamera.get(cv::CAP_PROP_FRAME_HEIGHT));

  ros::init(argc, argv, "apriltag_ros_single_image_client");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  // image_transport::ImageTransport it(nh);
  // std::string transport_hint;
  // pnh.param<std::string>("transport_hint", transport_hint, "raw");

  // image_transport::Subscriber camera_image_subscriber =
      // it.subscribe("image_rect", 1, &ImageListener::imageCallback, &imgl, image_transport::TransportHints(transport_hint));
  // ros::Rate loop_rate(10);
  cv::namedWindow("Video");
  cv::setMouseCallback("Video", MouseEvent, 0);
  while (num < maxnum)
  {
    USBCamera >> SrcImg;
    if (SrcImg.empty())
      ROS_ERROR("Read error!");
    if((char)cv::waitKey(1)==27)
      return 1;
    cv::imshow("Video", SrcImg);
    // ros::spinOnce();
    // if (imgl.ok)
    //   cv::imshow("Video", imgl.cv_image_->image);
    // loop_rate.sleep();
  }

  // ps.reset();
  ps.end();

  ros::ServiceClient client =
      nh.serviceClient<apriltag_ros::AnalyzeSingleImage>(
          "single_image_tag_detection");

  // Get the request parameters
  apriltag_ros::AnalyzeSingleImage service;

  // Replicate sensors_msgs/CameraInfo message (must be up-to-date with the
  // analyzed image!)
  service.request.camera_info.distortion_model = "plumb_bob";
  double fx, fy, cx, cy;
  if (!getRosParameter(pnh, "fx", fx))
    return 1;
  if (!getRosParameter(pnh, "fy", fy))
    return 1;
  if (!getRosParameter(pnh, "cx", cx))
    return 1;
  if (!getRosParameter(pnh, "cy", cy))
    return 1;
  // Intrinsic camera matrix for the raw (distorted) images
  service.request.camera_info.K[0] = fx;
  service.request.camera_info.K[2] = cx;
  service.request.camera_info.K[4] = fy;
  service.request.camera_info.K[5] = cy;
  service.request.camera_info.K[8] = 1.0;
  service.request.camera_info.P[0] = fx;
  service.request.camera_info.P[2] = cx;
  service.request.camera_info.P[5] = fy;
  service.request.camera_info.P[6] = cy;
  service.request.camera_info.P[10] = 1.0;

  for (int i = 1; i <= maxnum; i++)
  {
    snprintf(text, sizeof(text), "/home/fu/apriltag_ws/src/my_hand_eye/img/ApriltagRaw_%d.png", i); //
    service.request.full_path_where_to_get_image =
        apriltag_ros::getAprilTagOption<std::string>(
            pnh, "image_load_path", text);
    if (service.request.full_path_where_to_get_image.empty())
    {
      File1.close();
      File2.close();
      Res.close();
      return 1;
    }
    snprintf(text, sizeof(text), "/home/fu/apriltag_ws/src/my_hand_eye/img/Apriltag_%d.png", i); //
    service.request.full_path_where_to_save_image =
        apriltag_ros::getAprilTagOption<std::string>(
            pnh, "image_save_path", text);
    if (service.request.full_path_where_to_save_image.empty())
    {
      File1.close();
      File2.close();
      Res.close();
      return 1;
    }
    // Call the service (detect tags in the image specified by the
    // image_load_path)
    if (client.call(service))
    {
      // use parameter run_quielty=false in order to have the service
      // print out the tag position and orientation
      if (service.response.tag_detections.detections.size() != 1)
      {
        ROS_WARN_STREAM("The number of tags is not equal to 1!");
      }
      else
      {
        apriltag_ros::AprilTagDetection detections = service.response.tag_detections.detections[0];
        geometry_msgs::Pose pose = detections.pose.pose.pose;
        Eigen::Matrix3d rotation_R;
        Eigen::Quaterniond rotation_q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        rotation_R = rotation_q.toRotationMatrix();
        cv::Mat Mb2c(3, 1, CV_64F);
        cv::Mat Rb2c(3, 1, CV_64F);
        cv::Mat tb2c = (cv::Mat_<double>(3, 1) << pose.position.x, pose.position.y, pose.position.z);
        cv::eigen2cv(rotation_R, Mb2c);
        cv::Rodrigues(Mb2c, Rb2c);
        Rboard2camera.push_back(Mb2c);
        tboard2camera.push_back(tb2c * 1000.0);
        File1 << "R" << i << ":\n"
              << Mb2c << std::endl;
        File1 << "t" << i << ":\n"
              << (tb2c * 1000.0) << std::endl;
        ROS_INFO_STREAM("Succeeded to get image" << i);
        // ROS_INFO_STREAM("rotation_R" << i << ":\n" << rotation_R);
      }
    }
    else
    {
      ROS_ERROR("Failed to call service single_image_tag_detection");
      File1.close();
      File2.close();
      Res.close();
      return 1;
    }
  }
//   tend2base[1] =(cv::Mat_<double>(3,1)<<-1.549177905626925,27.73965957852912,6.564064428944966);
// 	tend2base[2] =(cv::Mat_<double>(3,1)<<-1.510573315293879,
//  27.04840379046395,
//  11.90062941039443);
// 	tend2base[0] =(cv::Mat_<double>(3,1)<<0.3280637061129121,
//  30.52095393265363,
//  9.824476837122955);
//  Rend2base[0] =(cv::Mat_<double>(3,3)<<0.0008367145496574766, -0.999685526224246, -0.02506289222166352,
// 									-0.03335551544114999, -0.02507685502116106, 0.9991289010592691,
// 									-0.9994431997359418, 0, -0.03336600817572286);
// 	Rend2base[1] =(cv::Mat_<double>(3,3)<<-0.001732229538130811, -0.9984441923587182, -0.05573324076324875,
//  									0.03101739156302764, -0.05576015374034603, 0.9979622872010139,
//  									-0.9995173439222819, 0, 0.03106572385358099);
// 	Rend2base[2] =(cv::Mat_<double>(3,3)<<-0.02401984394665902, -0.9984441923587182, -0.05032138553264579,
//  									0.4301005661064302, -0.05576015374034603, 0.9010573279707464,
//  									-0.9024613842883841, 0, 0.430770762550447);
  cv::Mat Rcamera2end(3, 3, CV_64FC1);
  cv::Mat tcamera2end(3, 1, CV_64FC1);
  // ROS_INFO_STREAM(Rboard2camera.size() << '\n' << Rend2base.size());
  cv::calibrateHandEye(Rend2base, tend2base, Rboard2camera, tboard2camera, Rcamera2end, tcamera2end);
  ROS_INFO("Succeeded to calibrateHandEye");
  Res << "Rcamera2end:\n" << Rcamera2end << std::endl;
  Res << "tcamera2end:\n" << tcamera2end << std::endl;
  File1.close();
  File2.close();
  Res.close();
  return 0; // happy ending
}
