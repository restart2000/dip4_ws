#include <stdlib.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include"ros/ros.h"
#define READIMAGE_ONLY
#ifndef READIMAGE_ONLY
#include <geometry_msgs/Twist.h>
#endif
#define max_3( a, b, c) (a) > (b)? (((a) > (c)? (a) : (c)) ): (((b) > (c)? (b):(c)))
#define min_3( a, b, c) (a) < (b)? (((a) < (c)? (a) : (c)) ): (((b) < (c)? (b):(c)))

int pos_H_min=0;
int pos_H_max=0;
int pos_S_min=0;
int pos_S_max=0;
int pos_V_min=0;
int pos_V_max=0;

int HBmax = 126, HBmin = 88, HGmax = 80, HGmin = 48, HYmax = 34, HYmin = 20, HRmax = 180, HRmin = 163;


using namespace cv;
using namespace std;

Mat RGB2HSV(Mat input)
{
    Mat frame=input.clone();
    Mat frame_gaussian;
    Mat output;
    int sigma=1;
    GaussianBlur(frame,frame_gaussian,Size(0,0),sigma,0,BORDER_DEFAULT);
    output=frame_gaussian.clone();
    
    for(int i=0;i<frame.rows;i++)
    {
        for(int j=0;j<frame.cols;j++)
        {
            double delta,Cmax,Cmin,R,G,B;
            B=(frame_gaussian.at<Vec3b>(i,j)[0]);
            G=(frame_gaussian.at<Vec3b>(i,j)[1]);
            R=(frame_gaussian.at<Vec3b>(i,j)[2]);
            B/=255; G/=255; R/=255;
            Cmax=max_3(B,G,R);
            Cmin=min_3(B,G,R);
            delta=Cmax-Cmin;
    
            double H,S,V;
            if(delta==0)    H=0;
            else if(Cmax==R)    H=60*((G-B)/delta)/2;
            else if(Cmax==G)    H=60*((B-R)/delta+2)/2;
            else H=60*((R-G)/delta+4)/2;
            if(H<0)  H+=180;      //计算中H可能会H<0，这时候加上180°（360°/2）


            if(Cmax==0) S=0;
            else S=(delta/Cmax)*255;

            V=Cmax*255;

            output.at<Vec3b>(i,j)[0]=H;
            output.at<Vec3b>(i,j)[1]=S;
            output.at<Vec3b>(i,j)[2]=V;
        }
    }

    return output;
}

Mat Threshold_segmentation(Mat input)
{
    Mat output(input.rows,input.cols,CV_8UC1);
    //cout<<pos_H_min<<endl;
    for(int i=0;i<input.rows;i++)
    {
        for(int j=0;j<input.cols;j++)
        {
            int H,S,V;
            H=input.at<Vec3b>(i,j)[0];
            S=input.at<Vec3b>(i,j)[1];
            V=input.at<Vec3b>(i,j)[2];
            //if(H>180) cout<<H<<endl;
            if(H>pos_H_min&&H<pos_H_max  &&S>pos_S_min&&S<pos_S_max    &&V>pos_V_min&&V<pos_V_max)
            {
                output.at<uchar>(i,j)=255;
            }
            else 
                output.at<uchar>(i,j)=0;
        }
    }
    return output;
}

void COlor_detection(Mat input)
{
    
}

int main(int argc, char **argv)
{
	ROS_WARN("*****START*****");
	ros::init(argc, argv, "trafficLaneTrack"); //初始化ROS节点
	ros::NodeHandle n;

	//Before the use of camera, you can test ur program with images first: imread()
	VideoCapture capture;
	capture.open(0); //打开zed相机，如果要打开笔记本上的摄像头，需要改为0
	waitKey(100);
	if (!capture.isOpened())
	{
		printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
		return 0;
	}

#ifndef READIMAGE_ONLY
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5); //定义dashgo机器人的速度发布器
#endif

	while (ros::ok())
	{
		// capture.read(src_frame);
		// if (src_frame.empty())
		// {
		// 	break;
		// }
        // Mat image_1=imread("./dip4_1.jpg");
        // if(image_1.empty())
        // {
        //     cout<<"can not load the image"<<endl;
        // }
        // Mat output_1;
        // output_1=RGB2HSV(image_1);
        // imshow("Hsv",output_1);

        Mat image_2=imread("./dip4_2.jpg");
        if(image_2.empty())
        {
            cout<<"can not load the image"<<endl;
        }
        Mat image_2_hsv=RGB2HSV(image_2);
        imshow("00",image_2_hsv);
        Mat output_2(image_2.rows,image_2.cols,CV_8U);
        namedWindow("Color_Segmentation");
        createTrackbar("H_min", "Color_Segmentation", &pos_H_min, 255 ,NULL); //创建滑动条
        createTrackbar("H_max", "Color_Segmentation", &pos_H_max, 255 ,NULL); //创建滑动条
        createTrackbar("S_min", "Color_Segmentation", &pos_S_min, 255 ,NULL); //创建滑动条
        createTrackbar("S_max", "Color_Segmentation", &pos_S_max, 255 ,NULL); //创建滑动条
        createTrackbar("V_min", "Color_Segmentation", &pos_V_min, 255 ,NULL); //创建滑动条
        createTrackbar("V_max", "Color_Segmentation", &pos_V_max, 255 ,NULL); //创建滑动条
        output_2=Threshold_segmentation(image_2_hsv);
        imshow("Color_Segmentation",output_2);


        vector<vector<Point> > contours;
        //vector容器里面放了一个vector容器，子容器里放点
		vector<Vec4i> h; 
        //1、Vec4i指的是四个整形数。
        //2、typedef Vec<int, 4> Vec4i;

        Mat thresholdimgB, thresholdimgG, thresholdimgY, thresholdimgR;
		inRange(image_2_hsv, Scalar(HGmin, 43, 46), Scalar(HGmax, 255, 255), thresholdimgG);
		inRange(image_2_hsv, Scalar(HYmin, 43, 46), Scalar(HYmax, 255, 255), thresholdimgY);
		inRange(image_2_hsv, Scalar(HBmin, 43, 46), Scalar(HBmax, 255, 255), thresholdimgB);
		inRange(image_2_hsv, Scalar(HRmin, 43, 46), Scalar(HRmax, 255, 255), thresholdimgR);
        // void inRange(	InputArray src,
        // 		InputArray lowerb,  
        // 		InputArray upperb,   
        // 		OutputArray dst);
        // 第一个参数：输入图像
        // 第二个参数：H、S、V的最小值，示例：Scalar(low_H, low_S, low_V)
        // 第三个参数：H、S、V的最大值，示例：Scalar(low_H, low_S, low_V)
        // 第四个参数：输出图像,要和输入图像有相同的尺寸且为CV_8U类

        findContours(thresholdimgY, contours, h, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
        //vector<Vec4i>  temp;
      //  temp=minAreaRect(h);

        // findContours( InputOutputArray image, OutputArrayOfArrays contours,  
        //                       OutputArray hierarchy, int mode,  
        //                       int method, Point offset=Point()); 
        //https://blog.csdn.net/laobai1015/article/details/76400725
		if (h.size() == 0) continue;
		
		for ( int idx = 0; idx >= 0; idx = h[idx][0] ) {
			Scalar color( 255, 255, 255 );
			drawContours( image_2, contours, idx, color, CV_FILLED, 8, h );
		}
		imshow("Green", thresholdimgG);
		imshow("Yellow",thresholdimgY);
		imshow("Blue", thresholdimgB);
		imshow("Red", thresholdimgR);
        imshow("image",image_2);

		ros::spinOnce();
		waitKey(5);
	}
	return 0;
}



