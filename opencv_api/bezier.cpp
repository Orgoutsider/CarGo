#include<opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;

//贝塞尔曲线参数取值的步长
#define BEZIER_STEP_LENTH 0.001

RNG rng(123456);//随机颜色

Mat drawing_board = Mat(Size(800, 500), CV_8UC3, Scalar::all(0));

//贝塞尔曲线绘制，P0-P2为控制点，bezier_t为可以从给定的贝塞尔曲线参数处开始绘制
//注意：控制点顺序为P2-P0-P1，改变控制点的顺序则绘制函数也需要改变
void draw_bezier(Point2d P0, Point2d P1, Point2d P2, double bezier_t)
{
	//绘制控制点
	Scalar ConP_color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	line(drawing_board, P0, P0, ConP_color, 4);
	line(drawing_board, P1, P1, ConP_color, 4);
	line(drawing_board, P2, P2, ConP_color, 4);

	//绘制贝塞尔曲线
	Scalar P_color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	for (double i = bezier_t; i <= 1; i+= BEZIER_STEP_LENTH)
	{
		Point2d P_1_ = (1 - i) * P2 + i * P0;
		Point2d P_2_ = (1 - i) * P0 + i * P1;
		Point2d P_3_ = (1 - i) * P_1_ + i * P_2_;
		line(drawing_board, P_3_, P_3_, P_color, 1);
	}
}

//贝塞尔曲线绘制，P0-P2为控制点
//注意：控制点顺序为P2-P0-P1，改变控制点的顺序则绘制函数也需要改变
void draw_bezier(Point2d P0, Point2d P1, Point2d P2)
{
	//绘制控制点
	Scalar ConP_color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	line(drawing_board, P0, P0, ConP_color, 4);
	line(drawing_board, P1, P1, ConP_color, 4);
	line(drawing_board, P2, P2, ConP_color, 4);

	//绘制贝塞尔曲线
	Scalar P_color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	for (double i = 0; i <= 1; i += BEZIER_STEP_LENTH)
	{
		Point2d P_1_ = (1 - i) * P2 + i * P0;
		Point2d P_2_ = (1 - i) * P0 + i * P1;
		Point2d P_3_ = (1 - i) * P_1_ + i * P_2_;
		line(drawing_board, P_3_, P_3_, P_color, 1);
	}
}

//给定点寻找控制点，给定点为Pb，P0-P1为已知控制点，P2为未知控制点，bezier_t为通过该函数计算得到的Pb点对应的参数
//注意：控制点顺序为P2-P0-P1，改变控制点则寻找控制点函数也需要改变
Point2d point_find(Point2d Pb, Point2d P0, Point2d P1, double& bezier_t)
{
	double k = (Pb.x - P0.x) / (P1.x - P0.x);
	bezier_t = sqrt(k);
	double y = (Pb.y + (pow(bezier_t, 2.0) - 2 * bezier_t) * P0.y) / pow((1 - bezier_t), 2.0);
	double x = P0.x;
	return Point2d(x, y);
}

//求出给定贝塞尔参数对应点处的导数，P0-P2为控制点
//注意：控制点顺序为P2-P0-P1，改变控制点则该函数也需要改变
double bezier_derivative(Point2d P0, Point2d P1, Point2d P2, double bezier_t)
{
	double x = 2 * bezier_t * (P1.x - P0.x);
	double y = -2 * (1 - bezier_t) * P2.y + (2 - 2 * bezier_t) * P0.y;
	return y / x;
}

//求出给定贝塞尔点处的导数，P0-P2为控制点
//注意：控制点顺序为P2-P0-P1，改变控制点则该函数也需要改变
double bezier_derivative(Point2d P0, Point2d P1, Point2d P2, Point2d Pb)
{
	double k = (Pb.x - P0.x) / (P1.x - P0.x);
	double bezier_t = sqrt(k);
	double x = 2 * bezier_t * (P1.x - P0.x);
	double y = -2 * (1 - bezier_t) * P2.y + (2 - 2 * bezier_t) * P0.y;
	return y / x;
}

//绘制对应贝塞尔点处的切线
void draw_tangent(double Pb_Derivative, Point2d Pb)
{
	for (double i = 0; i <= 100; i++)
	{
		double y = Pb_Derivative * (i - Pb.x) + Pb.y;
		line(drawing_board, Point2d(i, y), Point2d(i, y), Scalar(200, 100, 50), 2);
	}
}


void bezier(vector<Point2d> control_points)
{
	line(drawing_board, Point(0, 249), Point(799, 249), Scalar(255, 255, 255));
	line(drawing_board, Point(0, 0), Point(0, 499), Scalar(255, 255, 255));

	Point2d P0 = control_points[0];
	Point2d P1 = control_points[1];
	Point2d P2 = control_points[2];

	double t = 0.0;
	draw_bezier(P0, P1, P2, t);

	Point2d Pb = Point2d(100, 15);
	line(drawing_board, Pb, Pb, Scalar(0, 255, 0), 8);

	double t_New = 0;
	Point2d P2_New = point_find(Pb, P0, P1, t_New);

	cout << "P2_New.x  :" << P2_New.x << endl;
	cout << "P2_New.y  :" << P2_New.y << endl;
	cout << "t_New  :" << t_New << endl;
	draw_bezier(P0, P1, P2_New, 0);

	double bezier_der = bezier_derivative(P0, P1, P2_New, t_New);
	cout << "bezier_derivative  :" << bezier_der << endl;
	draw_tangent(bezier_der, Pb);

	imshow("Drawing_board", drawing_board);

}