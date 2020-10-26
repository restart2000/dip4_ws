#include <stdlib.h>
#include <cv.h>
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <math.h>

#define LINEAR_X 0
#define WINDOW_AUTOSIZE "ColourRange"

using namespace cv; 
using namespace std;

int k=0;
Mat frame;
Mat ColourRange(Mat input, int k);
Mat get_area_img(Mat input, Point2f P[4]);

void RGB_to_HSV(int R, int G, int B, float& H, float& S, float& V)  //RGB转化为HSV，传入每个像素的RGB值，计算相应的HSV值，存入指针
{
	int max_RG = MAX(R, G);
	int max = MAX(max_RG, B);
	int min_RG = MIN(R, G);
	int min = MIN(min_RG, B);
	V = (float)max; //V
	float b_Nor = B/(float)255;
	float g_Nor = G/(float)255;
	float r_Nor = R/(float)255;
	if(max != 0) S = (V - (float)min)/V; //S
	else 
	{
		S = 0;
		H = 0;
		return;
	}
	if(max == R)
	{
		if(min == G) H = (float)((5 + b_Nor)/6);
		else H = (float)((1 - g_Nor)/6);
	}
	else if(max == G)
	{
		if(min == B) H = (float)((1 + r_Nor)/6);
		else H = (float)((1 - g_Nor)/6);
	}
	else if(max == B)
	{
		if(min == R) H = (float)((3 + g_Nor)/6);
		else H = (float)((5 - r_Nor)/6);
	}
	H *= 180;
	S *= 255;
}

Mat RGB_to_HSV_MatTransform(Mat input)  //传入RGB图像，将RGB图像转成HSV图像，返回HSV图像
{
	int row = input.rows, col = input.cols;
	Mat Gaussian = Mat(row, col ,CV_8UC3);
	GaussianBlur(input, Gaussian, Size(5, 5), 3, 3);
	Mat output = Mat(row, col ,CV_8UC3);
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			float b = Gaussian.at<Vec3b>(i, j)[0];
			float g = Gaussian.at<Vec3b>(i, j)[1];
			float r = Gaussian.at<Vec3b>(i, j)[2];
			float h, s, v;

			RGB_to_HSV(r, g, b, h, s, v);
			output.at<Vec3b>(i, j)[0] = h;
			output.at<Vec3b>(i, j)[1] = s;
			output.at<Vec3b>(i, j)[2] = v;
		}
	}
	return output;
}

void text(int,void*)//回调函数
{
	cout<<"数字k的值为:"<<k<<endl;
	Mat Range_img = ColourRange(frame, k);
	imshow(WINDOW_AUTOSIZE, Range_img);
}

Mat ColourRange(Mat input, int k)  //颜色分割，针对不同颜色进行分割，返回二值图像(目标颜色为白色，其他部分为黑色）
{
	int row = input.rows, col = input.cols;
	Mat HSV = Mat(row, col, CV_8UC3);
	Mat output = Mat(row, col, CV_8UC1);
	//cvtColor(input, HSV, CV_RGB2HSV);
	HSV = RGB_to_HSV_MatTransform(input);
	if(k == 0) //黑
	{
		for(int i = 0; i < row; i++)
			for(int j = 0; j < col; j++)
				if(HSV.at<Vec3b>(i, j)[0] >= 0 && HSV.at<Vec3b>(i, j)[0] <= 180
				&& HSV.at<Vec3b>(i, j)[1] >= 0 && HSV.at<Vec3b>(i, j)[1] <= 255
				&& HSV.at<Vec3b>(i, j)[2] >= 0 && HSV.at<Vec3b>(i, j)[2] <= 46)
				{
					output.at<uchar>(i, j) = 255;
				}
				else output.at<uchar>(i, j) = 0;
	}
	if(k == 1) //灰
	{
		for(int i = 0; i < row; i++)
			for(int j = 0; j < col; j++)
				if(HSV.at<Vec3b>(i, j)[0] >= 0 && HSV.at<Vec3b>(i, j)[0] <= 180
				&& HSV.at<Vec3b>(i, j)[1] >= 0 && HSV.at<Vec3b>(i, j)[1] <= 43
				&& HSV.at<Vec3b>(i, j)[2] >= 46 && HSV.at<Vec3b>(i, j)[2] <= 220)
				{
					output.at<uchar>(i, j) = 255;
				}
				else output.at<uchar>(i, j) = 0;
	}
	if(k == 2) //白
	{
		for(int i = 0; i < row; i++)
			for(int j = 0; j < col; j++)
				if(HSV.at<Vec3b>(i, j)[0] >= 0 && HSV.at<Vec3b>(i, j)[0] <= 180
				&& HSV.at<Vec3b>(i, j)[1] >= 0 && HSV.at<Vec3b>(i, j)[1] <= 30
				&& HSV.at<Vec3b>(i, j)[2] >= 221 && HSV.at<Vec3b>(i, j)[2] <= 255)
				{
					output.at<uchar>(i, j) = 255;
				}
				else output.at<uchar>(i, j) = 0;
	}
	if(k == 3) //红
	{
		for(int i = 0; i < row; i++)
			for(int j = 0; j < col; j++)
				if((HSV.at<Vec3b>(i, j)[0] >= 0 && HSV.at<Vec3b>(i, j)[0] <= 10
				&& HSV.at<Vec3b>(i, j)[1] >= 43 && HSV.at<Vec3b>(i, j)[1] <= 255
				&& HSV.at<Vec3b>(i, j)[2] >= 46 && HSV.at<Vec3b>(i, j)[2] <= 255))
				{
					output.at<uchar>(i, j) = 255;
				}
				else if(HSV.at<Vec3b>(i, j)[0] >= 156 && HSV.at<Vec3b>(i, j)[0] <= 180
				&& HSV.at<Vec3b>(i, j)[1] >= 43 && HSV.at<Vec3b>(i, j)[1] <= 255
				&& HSV.at<Vec3b>(i, j)[2] >= 46 && HSV.at<Vec3b>(i, j)[2] <= 255)
				{
					output.at<uchar>(i, j) = 255;
				}
				else output.at<uchar>(i, j) = 0;
	}
	if(k == 4) //橙
	{
		for(int i = 0; i < row; i++)
			for(int j = 0; j < col; j++)
				if(HSV.at<Vec3b>(i, j)[0] >= 11 && HSV.at<Vec3b>(i, j)[0] <= 25
				&& HSV.at<Vec3b>(i, j)[1] >= 43 && HSV.at<Vec3b>(i, j)[1] <= 255
				&& HSV.at<Vec3b>(i, j)[2] >= 46 && HSV.at<Vec3b>(i, j)[2] <= 255)
				{
					output.at<uchar>(i, j) = 255;
				}
				else output.at<uchar>(i, j) = 0;
	}
	if(k == 5) //黄
	{
		for(int i = 0; i < row; i++)
			for(int j = 0; j < col; j++)
				if(HSV.at<Vec3b>(i, j)[0] >= 26 && HSV.at<Vec3b>(i, j)[0] <= 34
				&& HSV.at<Vec3b>(i, j)[1] >= 43 && HSV.at<Vec3b>(i, j)[1] <= 255
				&& HSV.at<Vec3b>(i, j)[2] >= 46 && HSV.at<Vec3b>(i, j)[2] <= 255)
				{
					output.at<uchar>(i, j) = 255;
				}
				else output.at<uchar>(i, j) = 0;
	}
	if(k == 6) //绿
	{
		for(int i = 0; i < row; i++)
			for(int j = 0; j < col; j++)
				if(HSV.at<Vec3b>(i, j)[0] >= 35 && HSV.at<Vec3b>(i, j)[0] <= 77
				&& HSV.at<Vec3b>(i, j)[1] >= 43 && HSV.at<Vec3b>(i, j)[1] <= 255
				&& HSV.at<Vec3b>(i, j)[2] >= 46 && HSV.at<Vec3b>(i, j)[2] <= 255)
				{
					output.at<uchar>(i, j) = 255;
				}
				else output.at<uchar>(i, j) = 0;
	}
	if(k == 7) //青
	{
		for(int i = 0; i < row; i++)
			for(int j = 0; j < col; j++)
				if(HSV.at<Vec3b>(i, j)[0] >= 78 && HSV.at<Vec3b>(i, j)[0] <= 99
				&& HSV.at<Vec3b>(i, j)[1] >= 43 && HSV.at<Vec3b>(i, j)[1] <= 255
				&& HSV.at<Vec3b>(i, j)[2] >= 46 && HSV.at<Vec3b>(i, j)[2] <= 255)
				{
					output.at<uchar>(i, j) = 255;
				}
				else output.at<uchar>(i, j) = 0;
	}
	if(k == 8) //蓝
	{
		for(int i = 0; i < row; i++)
			for(int j = 0; j < col; j++)
				if(HSV.at<Vec3b>(i, j)[0] >= 100 && HSV.at<Vec3b>(i, j)[0] <= 124
				&& HSV.at<Vec3b>(i, j)[1] >= 43 && HSV.at<Vec3b>(i, j)[1] <= 255
				&& HSV.at<Vec3b>(i, j)[2] >= 46 && HSV.at<Vec3b>(i, j)[2] <= 255)
				{
					output.at<uchar>(i, j) = 255;
				}
				else output.at<uchar>(i, j) = 0;
	}
	if(k == 9) //紫
	{
		for(int i = 0; i < row; i++)
			for(int j = 0; j < col; j++)
				if(HSV.at<Vec3b>(i, j)[0] >= 125 && HSV.at<Vec3b>(i, j)[0] <= 155
				&& HSV.at<Vec3b>(i, j)[1] >= 43 && HSV.at<Vec3b>(i, j)[1] <= 255
				&& HSV.at<Vec3b>(i, j)[2] >= 46 && HSV.at<Vec3b>(i, j)[2] <= 255)
				{
					output.at<uchar>(i, j) = 255;
				}
				else output.at<uchar>(i, j) = 0;
	}
	return output;
}

float dist(Point2f x1,Point2f x2)//两点距离
{
	float dist = sqrt((x1.x - x2.x)*(x1.x - x2.x)+(x1.y - x2.y)*(x1.y - x2.y));
	return dist;
}

Mat Contours(Mat input) //框选多边形区域，返回截取的该区域图像
{
	int row = input.rows, col = input.cols;
	Mat grey = Mat(row, col ,CV_8UC1);
	Mat Gaussian = Mat(row, col ,CV_8UC1);
	Mat binary = Mat(row, col ,CV_8UC1);
	Mat area_img;
	cvtColor(input,grey,CV_BGR2GRAY);
	GaussianBlur(grey, Gaussian, Size(5, 5), 3, 1);
	for(int i = 0; i < row; i++)
	{
		for(int j = 0; j < col; j++)
		{
			if(Gaussian.at<uchar>(i, j) <= 100) binary.at<uchar>(i, j) = 0;
			else binary.at<uchar>(i, j) = 255;
		}
	}
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
	findContours(binary, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE, Point());//RETR_EXTERNAL
	Mat canvas = Mat::zeros(input.size(),CV_8UC1);
	//cout<<contours.size()<<endl;
	int flag = 1;
	for(int k=0; k<contours.size(); k++)
	{
		drawContours(canvas,contours,k,Scalar(255),1,8,hierarchy);
		RotatedRect rect = minAreaRect(contours[k]);
		Point2f P[4];
		rect.points(P);
		if(dist(P[0],P[1]) >100 && dist(P[0],P[1]) <130 && dist(P[2],P[1]) >160 && dist(P[2],P[1]) <220 && flag == 1)//限定矩形大小 P[0]左下 P[1]左上 P[2]右上 p[3]右下
		{
			for(int j=0;j<=3;j++)
			{
				line(input,P[j],P[(j+1)%4],Scalar(0,0,255),1);
				line(canvas,P[j],P[(j+1)%4],Scalar(80),2);
			}
			area_img = get_area_img(input, P);
			flag = 0;
		}
	}
	imshow("轮廓（边缘图）",canvas);
	imshow("轮廓（原图）",input);
	return area_img;
}

Mat get_area_img(Mat input, Point2f P[4]) //Contours()中，得到矩形区域的四个顶点的坐标，将该区域提取出来，得到新的图像
{
	int y0 = cvRound(P[1].x);
	int x0 = cvRound(P[1].y);
	int width = cvRound(P[3].x - P[1].x);
	int height = cvRound(P[3].y - P[1].y);
	Mat area_img = Mat(height, width, CV_8UC3);
	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			area_img.at<Vec3b>(i, j)[0] = input.at<Vec3b>(x0 + i, y0 + j)[0];
			area_img.at<Vec3b>(i, j)[1] = input.at<Vec3b>(x0 + i, y0 + j)[1];
			area_img.at<Vec3b>(i, j)[2] = input.at<Vec3b>(x0 + i, y0 + j)[2];
		}
	}
	return area_img;
}

int colour_sum(Mat input) //计算图像中某种颜色的像素总数
{
	int row = input.rows, col = input.cols;
	int sum = 0;
	for(int i = 0; i < row; i++)
	{
		for(int j = 0; j < col; j++)
		{
			if(input.at<uchar>(i, j) != 0) sum++;
		}
	}
	return sum;
}

Mat colour_culculator(Mat input) //画出图像的颜色统计图
{
	Mat culculator = Mat::zeros(500, 500, CV_8UC3);
	Mat black = ColourRange(input, 0);
	int black_sum = colour_sum(black);
	Mat gray = ColourRange(input, 1);
	int gray_sum = colour_sum(gray);
	Mat white = ColourRange(input, 2);
	int white_sum = colour_sum(white);
	Mat red = ColourRange(input, 3);
	int red_sum = colour_sum(red);
	Mat orange = ColourRange(input, 4);
	int orange_sum = colour_sum(orange);
	Mat yellow = ColourRange(input, 5);
	int yellow_sum = colour_sum(yellow);
	Mat green = ColourRange(input, 6);
	int green_sum = colour_sum(green);
	Mat cyan = ColourRange(input, 7);
	int cyan_sum = colour_sum(cyan);
	Mat blue = ColourRange(input, 8);
	int blue_sum = colour_sum(blue);
	Mat purple = ColourRange(input, 9);
	int purple_sum = colour_sum(purple);
	int SUM = (double)(black_sum + gray_sum + white_sum + red_sum + orange_sum + yellow_sum + green_sum + cyan_sum + blue_sum + purple_sum);
	
	double black_rate = black_sum/(double)SUM;
	double gray_rate = gray_sum/(double)SUM;
	double white_rate = white_sum/(double)SUM;
	double red_rate = red_sum/(double)SUM;
	double orange_rate = orange_sum/(double)SUM;
	double yellow_rate = yellow_sum/(double)SUM;
	double green_rate = green_sum/(double)SUM;
	double cyan_rate = cyan_sum/(double)SUM;
	double blue_rate = blue_sum/(double)SUM;
	double purple_rate = purple_sum/(double)SUM;

	int col = culculator.cols, row = culculator.rows;
	int wid = col/11;//每个颜色条的宽度
	int gap = wid/10;

	cout<<black_rate<<endl<<gray_rate<<endl<<white_rate<<endl<<red_rate<<endl<<orange_rate<<endl<<yellow_rate<<endl<<green_rate<<endl<<cyan_rate<<endl<<blue_rate<<endl;
	cout<<purple_rate<<endl;
	
	for(int i = row-1; i > round(row-row*black_rate); i--)//黑
	{
		for(int j = 0; j < wid; j++)
		{
			culculator.at<Vec3b>(i, j)[0]=0, culculator.at<Vec3b>(i, j)[1]=0, culculator.at<Vec3b>(i, j)[2]=0;
		}
	}
	for(int i = row-1; i > round(row-row*gray_rate); i--)//灰
	{
		for(int j = wid+gap; j < wid*2+gap; j++)
		{
			culculator.at<Vec3b>(i, j)[0]=192, culculator.at<Vec3b>(i, j)[1]=192, culculator.at<Vec3b>(i, j)[2]=192;
		}
	}
	for(int i = row-1; i > round(row-row*white_rate); i--)//白
	{
		for(int j = (wid+gap)*2; j < (wid+gap)*2+wid; j++)
		{
			culculator.at<Vec3b>(i, j)[0]=255, culculator.at<Vec3b>(i, j)[1]=255, culculator.at<Vec3b>(i, j)[2]=255;
		}
	}
	for(int i = row-1; i > round(row-row*red_rate); i--)//红
	{
		for(int j = (wid+gap)*3; j < (wid+gap)*3+wid; j++)
		{
			culculator.at<Vec3b>(i, j)[0]=0, culculator.at<Vec3b>(i, j)[1]=0, culculator.at<Vec3b>(i, j)[2]=255;
		}
	}
	for(int i = row-1; i > round(row-row*orange_rate); i--)//橙色
	{
		for(int j = (wid+gap)*4; j < (wid+gap)*4+wid; j++)
		{
			culculator.at<Vec3b>(i, j)[0]=0, culculator.at<Vec3b>(i, j)[1]=165, culculator.at<Vec3b>(i, j)[2]=255;
		}
	}
	for(int i = row-1; i > round(row-row*yellow_rate); i--)//黄
	{
		for(int j = (wid+gap)*5; j < (wid+gap)*5+wid; j++)
		{
			culculator.at<Vec3b>(i, j)[0]=0, culculator.at<Vec3b>(i, j)[1]=255, culculator.at<Vec3b>(i, j)[2]=255;
		}
	}
	for(int i = row-1; i > round(row-row*green_rate); i--)//绿
	{
		for(int j = (wid+gap)*6; j < (wid+gap)*6+wid; j++)
		{
			culculator.at<Vec3b>(i, j)[0]=0, culculator.at<Vec3b>(i, j)[1]=255, culculator.at<Vec3b>(i, j)[2]=0;
		}
	}
	for(int i = row-1; i > round(row-row*cyan_rate); i--)//青色
	{
		for(int j = (wid+gap)*7; j < (wid+gap)*7+wid; j++)
		{
			culculator.at<Vec3b>(i, j)[0]=255, culculator.at<Vec3b>(i, j)[1]=255, culculator.at<Vec3b>(i, j)[2]=0;
		}
	}
	for(int i = row-1; i > round(row-row*blue_rate); i--)//蓝
	{
		for(int j = (wid+gap)*8; j < (wid+gap)*8+wid; j++)
		{
			culculator.at<Vec3b>(i, j)[0]=255, culculator.at<Vec3b>(i, j)[1]=0, culculator.at<Vec3b>(i, j)[2]=0;
		}
	}
	for(int i = row-1; i > round(row-row*purple_rate); i--)//紫色
	{
		for(int j = (wid+gap)*9; j < (wid+gap)*9+wid; j++)
		{
			culculator.at<Vec3b>(i, j)[0]=255, culculator.at<Vec3b>(i, j)[1]=0, culculator.at<Vec3b>(i, j)[2]=255;
		}
	}
	return culculator;
}

int main(int argc, char **argv)
{
	frame = imread("TargetColour2.jpg");//当前帧图片
	Mat frln = frame.clone();
	if (!frame.data) return 0;
	imshow("原图", frame);
	Mat HSV_img = RGB_to_HSV_MatTransform(frame);
	imshow("HSV图像", HSV_img);
	namedWindow(WINDOW_AUTOSIZE, 1);
	createTrackbar("数字：",WINDOW_AUTOSIZE, &k, 9, text);
	text(k,0);
	Mat area_img = Contours(frln);
	imshow("截取的区域图像", area_img);
	Mat area_img_HSV = RGB_to_HSV_MatTransform(area_img);
	imshow("区域图像的HSV图像", area_img_HSV);
	Mat culculator = colour_culculator(area_img);
	imshow("颜色分类统计", culculator);

	waitKey(0);
	return 0;
}
