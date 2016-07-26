#include "stdafx.h"
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>  
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <Windows.h>

/*******************

********************/

using namespace cv;
using namespace std;

HANDLE Handle_Comm;               //声明串口

int Serial_Init()
{
	Handle_Comm = CreateFile(
		_T("COM4:"),                  //COM4
		GENERIC_READ | GENERIC_WRITE, //读写权限
		0,                            //独占方式
		NULL,
		OPEN_EXISTING,                //直接打开
		0,                            //同步方式
		NULL
	);
	if (Handle_Comm == (HANDLE)-1)
	{
		return 1;
	}
	else
	{
		//同步I/O方式打开串口
		SetupComm(Handle_Comm, 1024, 1024);  //输入输出缓冲区1024
		COMMTIMEOUTS TimeOuts;

		//设定读超时时间
		TimeOuts.ReadIntervalTimeout = 1000;
		TimeOuts.ReadTotalTimeoutMultiplier = 500;
		TimeOuts.ReadTotalTimeoutConstant = 5000;

		//设定写超时时间
		TimeOuts.WriteTotalTimeoutMultiplier = 500;
		TimeOuts.WriteTotalTimeoutConstant = 2000;

		SetCommTimeouts(Handle_Comm, &TimeOuts);  //设置超时

		DCB Handle_Dcb;
		GetCommState(Handle_Comm, &Handle_Dcb);  //获取串口配置信息
												 //修改串口参数
		Handle_Dcb.BaudRate = 115200;  //波特率
		Handle_Dcb.ByteSize = 8;  //数据位8位
		Handle_Dcb.Parity = NOPARITY;  //无奇偶检验位
		Handle_Dcb.StopBits = TWOSTOPBITS;  //两个停止位

											//保存串口信息
		SetCommState(Handle_Comm, &Handle_Dcb);
		//清空缓存
		PurgeComm(Handle_Comm, PURGE_TXCLEAR | PURGE_RXCLEAR);
		return 0;
	}
}//串口初始化

void Serial_Write(char lpOut[1])//串口写
{
	DWORD dwBytesWrite = 1;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	ClearCommError(Handle_Comm, &dwErrorFlags, &ComStat);
	bWriteStat = WriteFile(Handle_Comm, lpOut, dwBytesWrite, &dwBytesWrite, NULL);
	if (!bWriteStat)
	{
		printf("写串口失败!!/n");
	}
	else
	{
		printf("写串口成功!!/n");
	}
	PurgeComm(Handle_Comm,
		PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
}

void Serial_Close()//关闭串口
{
	CloseHandle(Handle_Comm);
}

int main()
{
	printf_s("         .,                                                             \n            A#.                                                            \n            hS                                                             \n                                                                           \n            .,          .:,...,.                           sGXX&&S,        \n            .,            ,,   .,.                       1H@@@@A1.         \n            .,         .....::.  .,.                  .5B@@@@Xs.           \n            .,     .,,...    .::.  .,.              ,SM@@@@@@M#@#B&91:     \n            .,   .,.   .,,,,...:ii.  .,.          :3M@@@BA&XX&&HM@@@@GS,   \n            ,:  .,...,::.  ,;;:  ,::.  .,.      ;9#@@@A1.       .;h5: #Mi  \n         ,,..:,;:.,:   .SH#@@#H3, ,:,.  .,.  iG@@@@Xr     ,,,:.,,   1@@#;  \n        ,,   i. : .,  ;B@#M@@@@@M; ,,.,.  .i8@@@#9;       .,::,::,. .A@@X  \n        :.   i. : .,  8@@BX959A#@G  :  .,sHH,SB5.  sMMMMMMMMMMMMMA   5@@M. \n       ,:.   i. : .,  1@@@@@MA&H#1  : .,.;8&sGM3,  s########MMMM#H.  G@@B  \n       ,,.   i, : .,   sA@@@@@@Hs  ,:,,    ,3#@@@8:  .............  h@@@3  \n       ,,,   :.:;.,:..   :shhs:  .:;.  .,.   .5B@@@8:             ,9@@@&.  \n       : .,,.,  ,,..,::,,.    .,::,  .,. .18,  .hB@@@8i.       :s8#@@@8.   \n       :         .,.   ..,,,,,,.. .,,.  ,&@M ,:   hA@@@@BA&X&HM@@@@MGi     \n       ;.....      ..,..............     :3&,,      ;1SG&BM##MBAGSr.      \n        :......,.        ......             ..               .              \n       .......::...                                                        \n             ..  ...,,..           .,,...........,.                        \n                      ..,,........:.i.            :                       \n                                   ..:,...........,,  \n");
	printf_s("\n*************************沈航黑鹰战队战车识别系统**************************\n");
	printf_s("\n******************请输入敌方颜色（红色为“0”，蓝色为“1”）********************\n");
	string type;
	cin >> type;
	String template_M, template_L, template_XL;
	printf_s("加载模板…………");
	if (type == "0")//加载模板
	{
		template_M = "Armor_Template\\Red_M.JPG";
		template_L = "Armor_Template\\Red_L.JPG";
		template_XL = "Armor_Template\\Red_XL.JPG";
	}
	else
	{
		template_M = "Armor_Template\\Blue_M.JPG";
		template_L = "Armor_Template\\Blue_L.JPG";
		template_XL = "Armor_Template\\Blue_XL.JPG";
	}
	printf_s("完成！\n");
	printf_s("初始化串口…………");
	int Seraial_open = Serial_Init();
	if (Seraial_open == 0)
	{
		printf_s("完成！");
	}
	else
	{
		printf_s("失败！");
		char fail;
		cin >> fail;
	}
	Serial_Write("0");
	waitKey(500);
	system("cls");

	VideoCapture cap(0);
	if (!cap.isOpened())
	{
		return -1;
	}

	Mat frame;//源图像
	Mat gauss;//高斯模糊图像
	Mat bina;//二值图像
	Mat dst;//膨胀图像
	Mat thes;//阈值图像
	vector<vector<Point>> contours; //连通域

	
	Mat temp_rslt;//模板匹配结果
	double distance;//最大连通域与识别点的距离
	double minVal; //匹配值最小值
	double maxVal;//匹配值最大值
	Point minLoc; //匹配值最小点
	Point maxLoc;//匹配值最大点
	Point matchLoc;//匹配位置点
	
	bool stop = false;

	while (!stop)
	{
		cap >> frame;//原始图像采集

		//********************寻找最大连通域*******************//
		imshow("原始图像", frame);
		GaussianBlur(frame, gauss, Size(3, 3), 1.5);
		threshold(gauss, thes, 250, 255, 1);//阈值限制
		cvtColor(thes, bina, CV_BGR2GRAY);//二值化
		bitwise_not(bina, thes);//二值反色
		dilate(thes, dst, Mat(10, 40, CV_8U), Point(-1, -1), 2);//膨胀
		imshow("阈值膨胀图像", dst);
		findContours(dst, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //寻找连通域
		double maxArea = 0;//最大连通域面积
		vector<cv::Point> maxContour;//最大连通域
		for (size_t i = 0; i < contours.size(); i++)//寻找最大连通域
		{
			double area = contourArea(contours[i]);
			if (area > maxArea)
			{
				maxArea = area;
				maxContour = contours[i];
			}
		}
		Rect maxRect = boundingRect(maxContour);//最大连通域矩形

		//********************模板匹配*******************//
		Mat templa;
		if (maxArea <= 7000)
		{
			templa = imread(template_M);
		}
		else if (maxArea > 7000 && maxArea < 13000)
		{
			templa = imread(template_L);
		}
		else
		{
			templa = imread(template_XL);
		}
		matchTemplate(frame, templa, temp_rslt, CV_TM_SQDIFF);//模板匹配
		normalize(temp_rslt, temp_rslt, 0, 1, NORM_MINMAX, -1, Mat());//归一化
		minMaxLoc(temp_rslt, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
		matchLoc = minLoc;
		
		//*******************模板与连通域比较***********************//
		distance = sqrt(fabs(((maxRect.x) - (minLoc.x))*((maxRect.x) - (minLoc.x)) + ((maxRect.y) - (minLoc.y))*((maxRect.y) - (minLoc.y))));
		if (distance<150.0)
		{		
			rectangle(frame, matchLoc, Point(matchLoc.x + templa.cols, matchLoc.y + templa.rows), Scalar::all(2), 2, 8, 0);
			if (matchLoc.x > 220 && matchLoc.x < 420 && matchLoc.y>150 && matchLoc.y < 340)
			{
				putText(frame, "TARGET LOCKED", Point(130, 240), 1, 3, Scalar(0, 0, 255, 255), 3.0, false);
				Serial_Write("2");//成功发现目标，输出信号1
			}
			else
			{
				putText(frame, "TARGET FOUND", Point(140, 240), 1, 3, Scalar(255, 255, 120, 0), 3.0, false);
				Serial_Write("1");//其他情况，输出0
			}
		}
		else
		{
			putText(frame, "TARGET LOST", Point(160, 240), 1, 3, Scalar(255, 0, 0, 0), 3.0, false);
			Serial_Write("0");//其他情况，输出0
		}
		//********************显示参数********************//
		char temp[8];
		_itoa_s(matchLoc.x, temp, 10);
		String match_x = temp;
		_itoa_s(matchLoc.y, temp, 10);
		String match_y = temp;
		String Match_out = "Match Point (" + match_x + "," + match_y + ")";
		putText(frame, Match_out, Point(50, 50), 1, 1, Scalar(255, 255, 255, 255), 1.0, false);

		_itoa_s(maxRect.x, temp, 10);
		String contour_x = temp;
		_itoa_s(maxRect.y, temp, 10);
		String contour_y = temp;
		String contour_out = "Contour Point (" + contour_x + "," + contour_y + ")";
		putText(frame, contour_out, Point(50, 70), 1, 1, Scalar(255, 255, 255, 255), 1.0, false);

		_itoa_s(distance, temp, 10);
		String distance_ = temp;
		String distance_out = "Distance " + distance_;
		putText(frame, distance_out, Point(50, 90), 1, 1, Scalar(255, 255, 255, 255), 1.0, false);

		_itoa_s(maxArea, temp, 10);
		String cont_max_sq = temp;
		String cont_max_sq_out = "Max Contour Square " + cont_max_sq;
		putText(frame, cont_max_sq_out, Point(50, 110), 1, 1, Scalar(255,255,255,255), 1.0, false);

		//***************显示结果***********************//
		imshow("输出窗口", frame);

		//*********************延时等待****************//
		waitKey(10);
	}
	Serial_Close();//关闭串口
	return 0;
}

