#include<iostream>
#include<opencv.hpp>

using namespace cv;
using namespace std;

void HistogramLookup(Mat Img)
{
	Mat HSVImg;
	cvtColor(Img, HSVImg, COLOR_BGR2HSV);

	vector<Mat> HSV_planes;
	split(HSVImg, HSV_planes);

	//Mat H_planes(HSV_planes[0], CV_32FC1);

	const int channels[1] = { 0 };
	const int histSize[1] = { 180 };
	float pranges[2] = { 0,179 };
	const float* ranges[1] = { pranges };

	MatND hist;
	calcHist(&HSV_planes[0], 1, channels, Mat(), hist, 1, histSize, ranges);
	normalize(hist, hist, 1.0, 0, NORM_L1, -1, Mat());
	double count = .0;
	for (size_t i = 0; i < 180; i++)
	{
		float data = hist.at<float>(i);
		count += data;
		cout << i << ":  " << data << "\n";
	}
	cout << count << endl;
}