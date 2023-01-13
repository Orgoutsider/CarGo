#include <opencv.hpp>

//void ColorFinding(cv::VideoCapture* camera, std::vector<cv::Point>* figure_info, unsigned RGB);

void QRcode(cv::VideoCapture* camera, std::string* info);

//void line_detect(cv::VideoCapture* camera);

void TargetTracking(cv::VideoCapture* camera, unsigned RGB);
