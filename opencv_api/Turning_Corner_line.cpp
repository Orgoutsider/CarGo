#include<opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;
Scalar BK_low__ = Scalar(0, 0, 0);
Scalar BK_up__ = Scalar(180, 255, 100);
const int judge_x = 50, judge_y = 200, judge_turning=10;//动态调整
const int x_low_ = 190, x_up_ = 290, y_low_ = 50, y_up_ = 270;//动态调整
//RNG rng__(12345);
void Turning_Corner_line(VideoCapture* camera)
{
	Mat srcF, srcImg;

	while (true)
	{
		*camera >> srcImg;
		if (srcImg.empty())
			break;
		if ((char)waitKey(1) == 27)
		{
			break;
		}
		resize(srcImg, srcF, srcImg.size() / 4);
		srcF = srcF(Range(y_low_, y_up_), Range(x_low_, x_up_));//动态调整
		cvtColor(srcF, srcF, COLOR_BGR2HSV);
		inRange(srcF, BK_low__, BK_up__, srcF);
#if(0)
		Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
		morphologyEx(srcF, srcF, MORPH_OPEN, element);
#endif
		Canny(srcF, srcF, 50, 100, 3);
		vector<Vec2f> lines;
		HoughLines(srcF, lines, 1, CV_PI / 180, 50, 0, 0);//动态调整
		imshow("Turning_line", srcF);
		Mat res=Mat::zeros(srcF.size(), CV_8UC3);
		int miny, maxy, nowy, derta;
		if (lines.size())
		{
			for (int i = 0; i < lines.size(); i++)
			{
				double rho = lines[i][0], theta = lines[i][1];
				double a = cos(theta), b = sin(theta);
				//cout << i << "rho: " << rho << endl;
				//cout << i << "theta: " << theta << endl;
				//Point pt1, pt2;
				//double x0 = a * rho, y0 = b * rho;
				//pt1.x = cvRound(x0 + 1000 * (-b));
				//pt1.y = cvRound(y0 + 1000 * (a));
				//pt2.x = cvRound(x0 - 1000 * (-b));
				//pt2.y = cvRound(y0 - 1000 * (a));
				//Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
				//line(res, pt1, pt2, color, 1, LINE_AA);
				nowy = cvRound((rho - a * judge_x) / b);
				if (i == 0)
				{
					miny = maxy = nowy;
				}
				miny = (nowy < miny) ? nowy : miny;
				maxy = (nowy > maxy) ? nowy : maxy;
			}
			derta = abs((miny + maxy) / 2 - judge_y);
			//std::cout << "derta:  " << derta << endl;
			if (derta <= judge_turning)
				std::cout << "停止并转弯" << endl;
		}
	}
}