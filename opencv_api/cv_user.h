#include <opencv.hpp>
#ifndef _COLORFINDING_H_
#define _COLORFINDING_H_

void ColorFinding(cv::VideoCapture* camera, std::vector<cv::Point>* figure_info, std::vector<cv::RotatedRect>* rects, unsigned RGB);
#endif 

#ifndef _QRCODE_H_
#define _QRCODE_H_

void QRcode(cv::VideoCapture* camera, std::string* info);
#endif
