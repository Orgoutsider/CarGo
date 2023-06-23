#include "my_hand_eye/pose.h"

s16 Position[6];
u16 Speed[6] = {0, 0, 20, 60, 45, 45};
u8 ACC[6] = {10, 10, 10, 10, 10, 10};
SMS_STS sm_st;
SCSCL sc;
my_hand_eye::Pos ps(&sm_st, &sc);


//指定运动位置
double targetx[30] = {-0.563389, -1.45867, -2.04845, -1.41844, 0.535275, -5.75446, 1.80323, 7.59661e-07, -3.72793, 4.38057, -0.605159, 1.51909, -5.65667, 3.30037, -0.526636, -0.567919, 8.35015e-07, 8.26643e-07, 5.5163, 3.24953-2.21159, 2.25481, -6.94017, 1.64182, 8.03178e-07, -6.86798, -0.482365, -1.42676, -0.96989, 6.19705};
double targety[30] = {24.7766, 20.3331, 21.7942, 19.5654, 23.1659, 22.1041, 18.2874, 20.851, 19.0256, 28.1768, 27.1696, 21.486, 21.601, 23.901, 22.671, 25.0361, 23.6632, 23.3508, 18.4521, 23.4172, 24.1272, 24.7452, 22.5612, 23.8279, 22.475, 22.2485, 20.1347, 19.7243, 20.274, 21.6548};
double targetz[30] = {13.6579, 6.99897, 3.6582, 12.4393, 10.5183, 9.03096, 2.43279, 10.2917, 6.22193, 11.1699, 7.97784, 12.963, 7.78461, 13.3025, 12.2238, 14.553, 8.39071, 14.6011, 4.99748, 12.2275, 10.781, 10.5426, 12.3971, 11.124, 14.2681, 7.75108, 5.37771, 4.59983, 2.15212, 10.975};
std::vector<cv::Mat> Rend2base;
std::vector<cv::Mat> tend2base;
std::vector<cv::Mat> Rboard2camera;
std::vector<cv::Mat> tboard2camera;
int num = 0;int maxnum = 30;

void MouseEvent(int event, int x, int y, int flags, void *data)
{
  if (event == cv::EVENT_LBUTTONDBLCLK)
  {
    num++;
    if(ps.refresh_xyz())
      ARM_INFO_XYZ(ps);
    else
      ROS_WARN("Invalid!");
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
  ros::init(argc, argv, "my_hand_eye_move");
  ros::NodeHandle nh;
  cv::namedWindow("Video");
  cv::setMouseCallback("Video", MouseEvent, 0);
  cv::Mat Img(640, 480, CV_8UC4, cv::Scalar(0, 0, 0, 255));
  while (num < maxnum&&ros::ok()){
    cv::imshow("Video", Img);
    cv::waitKey(5);

  };
  ps.reset();
  ps.end();
  return 0; // happy ending
}
