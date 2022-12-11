#include<opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;
Scalar BK_low = Scalar(0, 0, 0);
Scalar BK_up = Scalar(180, 255, 46);
const double judge = 0;
//RNG rng(12345);
void line_detect(VideoCapture* camera)
{
	Mat srcF, srcImg;
	
	while (true)
	{
		//flag++;
		*camera >> srcImg;
		imshow("src", srcImg);
		if (srcImg.empty())
			break;
		if ((char)waitKey(1) == 27)
		{
			break;
			//destroyAllWindows();//关闭所有图形窗口
		}
		resize(srcImg, srcF, srcImg.size() / 4);
		srcF = srcF(Range(100, 270), Range(0, 480));//动态调整

		cvtColor(srcF, srcF, COLOR_BGR2HSV);
		inRange(srcF, BK_low, BK_up, srcF);
		Canny(srcF, srcF, 50, 100, 3);
		vector<Vec2f> lines;
		HoughLines(srcF, lines, 1, CV_PI / 180, 50, 0, 0);//动态调整
		imshow("line1", srcF);
		Mat res(srcF.size(), CV_8UC3, Scalar::all(0));//int flag1 = 0;
		int minx, maxx, nowx, last_maxx, last_minx, bar = 400;
		//int err_t = 0;
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
				//line(res, Point(0, 0), Point(479, 0), Scalar(0, 0, 255), 3, LINE_AA);
				nowx = cvRound((rho - b * judge) / a);
				if (i == 0)
				{
					minx = maxx = nowx;
					
				}

				minx = (nowx < minx) ? nowx : minx;
				maxx = (nowx > maxx) ? nowx : maxx;
				if (i != 0)
				{
					minx = ((minx - last_minx) * (minx - last_minx) > bar) ? last_minx : minx;
					maxx = ((maxx - last_maxx) * (maxx - last_maxx) > bar) ? last_maxx : maxx;
				}
				last_minx = minx;
				last_maxx = maxx;

			}
			cout << minx << ' ' << maxx << endl;
			int err = (minx + maxx) / 2 - srcF.cols / 2;
			cout << err << endl;

		}
	}
}