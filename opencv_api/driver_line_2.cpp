#include<opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;
Scalar BK_low = Scalar(0, 0, 0);
Scalar BK_up = Scalar(180, 255, 46);
const double judge = 100;
void line_detect(VideoCapture* camera)
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
			destroyAllWindows();//关闭所有图形窗口
		}
		resize(srcImg, srcF, srcImg.size() / 4);
		srcF = srcF(Range(60, 250), Range(0, 480));

		cvtColor(srcF, srcF, COLOR_BGR2HSV);
		inRange(srcF, BK_low, BK_up, srcF);
		//srcF.convertTo(srcF, CV_8UC1);
		Canny(srcF, srcF, 50, 100, 3);
		vector<Vec2f> lines;
		HoughLines(srcF, lines, 1, CV_PI / 180, 50, 0, 0);
		imshow("line1", srcF);
		Mat res(srcF.size(),CV_8UC3);
		int miny, maxy, judge = 150, nowy;
		for (int i = 0; i < lines.size(); i++)
		{
			double rho = lines[i][0], theta = lines[i][1];
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a * rho, y0 = b * rho;
			pt1.x = cvRound(x0 + 1000 * (-b));
			pt1.y = cvRound(y0 + 1000 * (a));
			pt2.x = cvRound(x0 - 1000 * (-b));
			pt2.y = cvRound(y0 - 1000 * (a));
			line(res, pt1, pt2, Scalar(0, 0, 255), 3, LINE_AA);
			nowy = cvRound((rho));// - a * judge)/b
			if (i == 0)
			{
				miny = maxy = nowy;
			}
			miny = (nowy < miny) ? nowy : miny;
			maxy = (nowy > maxy) ? nowy : maxy;
		}
		cout << miny << ' ' << maxy << endl;
		int err = (miny + maxy) / 2;// -srcF.rows / 2;
		cout << err << endl;
		imshow("line2", res);
	}

}