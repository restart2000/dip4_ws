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

int low_Threshold=0;
int high_Threshold=0;


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


void Color_detection(Mat input)
{
        Mat image=input.clone();
        Mat image_gaussian;
        Mat gray;
        Mat binary;
        int sigma=1;
        GaussianBlur(image,image_gaussian,Size(0,0),sigma,0,BORDER_DEFAULT); //进行高斯滤波
        cvtColor(image_gaussian,gray,CV_BGR2GRAY); //将高斯模糊图转化为灰度图
        //imshow("gray",gray);
        namedWindow("binary", WINDOW_AUTOSIZE); //创建一个窗口：binary
        createTrackbar("low_Threshold", "binary", &low_Threshold, 255, NULL); //创建滑动条
        createTrackbar("high_Threshold", "binary", &high_Threshold, 255, NULL); //创建滑动条
        Canny(gray,binary,low_Threshold,high_Threshold); 
        imshow("binary",binary);

        vector<vector<Point> > contours;
        //vector容器里面放了一个vector容器，子容器里放点
		vector<Vec4i> hierarchy; 
        //1、Vec4i指的是四个整形数。
        //2、typedef Vec<int, 4> Vec4i;
        findContours(binary,contours,hierarchy,RETR_TREE,CV_CHAIN_APPROX_NONE,Point(0, 0));
        // findContours( InputOutputArray image, OutputArrayOfArrays contours,  
        //                       OutputArray h, int mode,  
        //                       int method, Point offset=Point()); 
        //https://blog.csdn.net/laobai1015/article/details/76400725
        Mat edge = Mat::zeros(input.rows,input.cols, CV_8UC3);
        double TargetArea = 0;
        findContours(binary,contours,hierarchy,RETR_TREE,CV_CHAIN_APPROX_NONE,Point(0, 0));
        for(int i = 0; i < contours.size(); i++)
        {
            Rect rect = boundingRect(contours[i]);
            if(rect.area() > 40000)
            {
                rectangle(edge,rect,Scalar(255),2);
                Mat ROI=input(rect);
                imshow("Interesting",ROI);
                Mat HSV_Interesting=ROI.clone();
                HSV_Interesting=RGB2HSV(ROI);
                imshow("HSV_Interesting",HSV_Interesting);
                int color[7]={0};
                for(int i=0;i<HSV_Interesting.rows;i++)
                {
                    for(int j=0;j<HSV_Interesting.cols;j++)
                    {
                        int h=input.at<Vec3b>(i,j)[0];
                            int s=input.at<Vec3b>(i,j)[1];
                                int v=input.at<Vec3b>(i,j)[2];
                        if(h>210 && h<255 && s>43 && s<255 && v>46 && v<255)  color[0]+=1;
                        else if(h>11 && h<25 && s>43 && s<255 && v>46 && v<255)  color[1]+=1;
                        else if(h>26 && h<34 && s>43 && s<255 && v>46 && v<255)  color[2]+=1;
                        else if(h>35 && h<77 && s>43 && s<255 && v>46 && v<255)  color[3]+=1;
                        else if(h>78 && h<99 && s>43 && s<255 && v>46 && v<255)  color[4]+=1;
                        else if(h>100 && h<124 && s>43 && s<255 && v>46 && v<255)  color[5]+=1;
                        else if(h>125 && h<155 && s>43 && s<255 && v>46 && v<255)  color[6]+=1;	

                    }
                }
                cout<<"红色像素个数："<<color[0]<<endl;
                cout<<"橙色像素个数："<<color[1]<<endl;
                cout<<"黄色像素个数："<<color[2]<<endl;
                cout<<"绿色像素个数："<<color[3]<<endl;
                cout<<"青色像素个数："<<color[4]<<endl;
                cout<<"蓝色像素个数："<<color[5]<<endl;
                cout<<"紫色像素个数："<<color[6]<<endl;

            }
        imshow("Contours",edge);
     }

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

        // Mat image_2=imread("./dip4_2.jpg");
        // if(image_2.empty())
        // {
        //     cout<<"can not load the image"<<endl;
        // }
        // Mat image_2_hsv=RGB2HSV(image_2);
        // imshow("00",image_2_hsv);
        // Mat output_2(image_2.rows,image_2.cols,CV_8U);
        // namedWindow("Color_Segmentation");
        // createTrackbar("H_min", "Color_Segmentation", &pos_H_min, 255 ,NULL); //创建滑动条
        // createTrackbar("H_max", "Color_Segmentation", &pos_H_max, 255 ,NULL); //创建滑动条
        // createTrackbar("S_min", "Color_Segmentation", &pos_S_min, 255 ,NULL); //创建滑动条
        // createTrackbar("S_max", "Color_Segmentation", &pos_S_max, 255 ,NULL); //创建滑动条
        // createTrackbar("V_min", "Color_Segmentation", &pos_V_min, 255 ,NULL); //创建滑动条
        // createTrackbar("V_max", "Color_Segmentation", &pos_V_max, 255 ,NULL); //创建滑动条
        // output_2=Threshold_segmentation(image_2_hsv);
        // imshow("Color_Segmentation",output_2);

        Mat image_3=imread("./dip4_3.jpg");
        Color_detection(image_3);

		ros::spinOnce();
		waitKey(5);
	}
	return 0;
}



