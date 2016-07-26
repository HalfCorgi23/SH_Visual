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

HANDLE Handle_Comm;               //��������

int Serial_Init()
{
	Handle_Comm = CreateFile(
		_T("COM4:"),                  //COM4
		GENERIC_READ | GENERIC_WRITE, //��дȨ��
		0,                            //��ռ��ʽ
		NULL,
		OPEN_EXISTING,                //ֱ�Ӵ�
		0,                            //ͬ����ʽ
		NULL
	);
	if (Handle_Comm == (HANDLE)-1)
	{
		return 1;
	}
	else
	{
		//ͬ��I/O��ʽ�򿪴���
		SetupComm(Handle_Comm, 1024, 1024);  //�������������1024
		COMMTIMEOUTS TimeOuts;

		//�趨����ʱʱ��
		TimeOuts.ReadIntervalTimeout = 1000;
		TimeOuts.ReadTotalTimeoutMultiplier = 500;
		TimeOuts.ReadTotalTimeoutConstant = 5000;

		//�趨д��ʱʱ��
		TimeOuts.WriteTotalTimeoutMultiplier = 500;
		TimeOuts.WriteTotalTimeoutConstant = 2000;

		SetCommTimeouts(Handle_Comm, &TimeOuts);  //���ó�ʱ

		DCB Handle_Dcb;
		GetCommState(Handle_Comm, &Handle_Dcb);  //��ȡ����������Ϣ
												 //�޸Ĵ��ڲ���
		Handle_Dcb.BaudRate = 115200;  //������
		Handle_Dcb.ByteSize = 8;  //����λ8λ
		Handle_Dcb.Parity = NOPARITY;  //����ż����λ
		Handle_Dcb.StopBits = TWOSTOPBITS;  //����ֹͣλ

											//���洮����Ϣ
		SetCommState(Handle_Comm, &Handle_Dcb);
		//��ջ���
		PurgeComm(Handle_Comm, PURGE_TXCLEAR | PURGE_RXCLEAR);
		return 0;
	}
}//���ڳ�ʼ��

void Serial_Write(char lpOut[1])//����д
{
	DWORD dwBytesWrite = 1;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	ClearCommError(Handle_Comm, &dwErrorFlags, &ComStat);
	bWriteStat = WriteFile(Handle_Comm, lpOut, dwBytesWrite, &dwBytesWrite, NULL);
	if (!bWriteStat)
	{
		printf("д����ʧ��!!/n");
	}
	else
	{
		printf("д���ڳɹ�!!/n");
	}
	PurgeComm(Handle_Comm,
		PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
}

void Serial_Close()//�رմ���
{
	CloseHandle(Handle_Comm);
}

int main()
{
	printf_s("         .,                                                             \n            A#.                                                            \n            hS                                                             \n                                                                           \n            .,          .:,...,.                           sGXX&&S,        \n            .,            ,,   .,.                       1H@@@@A1.         \n            .,         .....::.  .,.                  .5B@@@@Xs.           \n            .,     .,,...    .::.  .,.              ,SM@@@@@@M#@#B&91:     \n            .,   .,.   .,,,,...:ii.  .,.          :3M@@@BA&XX&&HM@@@@GS,   \n            ,:  .,...,::.  ,;;:  ,::.  .,.      ;9#@@@A1.       .;h5: #Mi  \n         ,,..:,;:.,:   .SH#@@#H3, ,:,.  .,.  iG@@@@Xr     ,,,:.,,   1@@#;  \n        ,,   i. : .,  ;B@#M@@@@@M; ,,.,.  .i8@@@#9;       .,::,::,. .A@@X  \n        :.   i. : .,  8@@BX959A#@G  :  .,sHH,SB5.  sMMMMMMMMMMMMMA   5@@M. \n       ,:.   i. : .,  1@@@@@MA&H#1  : .,.;8&sGM3,  s########MMMM#H.  G@@B  \n       ,,.   i, : .,   sA@@@@@@Hs  ,:,,    ,3#@@@8:  .............  h@@@3  \n       ,,,   :.:;.,:..   :shhs:  .:;.  .,.   .5B@@@8:             ,9@@@&.  \n       : .,,.,  ,,..,::,,.    .,::,  .,. .18,  .hB@@@8i.       :s8#@@@8.   \n       :         .,.   ..,,,,,,.. .,,.  ,&@M ,:   hA@@@@BA&X&HM@@@@MGi     \n       ;.....      ..,..............     :3&,,      ;1SG&BM##MBAGSr.      \n        :......,.        ......             ..               .              \n       .......::...                                                        \n             ..  ...,,..           .,,...........,.                        \n                      ..,,........:.i.            :                       \n                                   ..:,...........,,  \n");
	printf_s("\n*************************�򺽺�ӥս��ս��ʶ��ϵͳ**************************\n");
	printf_s("\n******************������з���ɫ����ɫΪ��0������ɫΪ��1����********************\n");
	string type;
	cin >> type;
	String template_M, template_L, template_XL;
	printf_s("����ģ�塭������");
	if (type == "0")//����ģ��
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
	printf_s("��ɣ�\n");
	printf_s("��ʼ�����ڡ�������");
	int Seraial_open = Serial_Init();
	if (Seraial_open == 0)
	{
		printf_s("��ɣ�");
	}
	else
	{
		printf_s("ʧ�ܣ�");
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

	Mat frame;//Դͼ��
	Mat gauss;//��˹ģ��ͼ��
	Mat bina;//��ֵͼ��
	Mat dst;//����ͼ��
	Mat thes;//��ֵͼ��
	vector<vector<Point>> contours; //��ͨ��

	
	Mat temp_rslt;//ģ��ƥ����
	double distance;//�����ͨ����ʶ���ľ���
	double minVal; //ƥ��ֵ��Сֵ
	double maxVal;//ƥ��ֵ���ֵ
	Point minLoc; //ƥ��ֵ��С��
	Point maxLoc;//ƥ��ֵ����
	Point matchLoc;//ƥ��λ�õ�
	
	bool stop = false;

	while (!stop)
	{
		cap >> frame;//ԭʼͼ��ɼ�

		//********************Ѱ�������ͨ��*******************//
		imshow("ԭʼͼ��", frame);
		GaussianBlur(frame, gauss, Size(3, 3), 1.5);
		threshold(gauss, thes, 250, 255, 1);//��ֵ����
		cvtColor(thes, bina, CV_BGR2GRAY);//��ֵ��
		bitwise_not(bina, thes);//��ֵ��ɫ
		dilate(thes, dst, Mat(10, 40, CV_8U), Point(-1, -1), 2);//����
		imshow("��ֵ����ͼ��", dst);
		findContours(dst, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //Ѱ����ͨ��
		double maxArea = 0;//�����ͨ�����
		vector<cv::Point> maxContour;//�����ͨ��
		for (size_t i = 0; i < contours.size(); i++)//Ѱ�������ͨ��
		{
			double area = contourArea(contours[i]);
			if (area > maxArea)
			{
				maxArea = area;
				maxContour = contours[i];
			}
		}
		Rect maxRect = boundingRect(maxContour);//�����ͨ�����

		//********************ģ��ƥ��*******************//
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
		matchTemplate(frame, templa, temp_rslt, CV_TM_SQDIFF);//ģ��ƥ��
		normalize(temp_rslt, temp_rslt, 0, 1, NORM_MINMAX, -1, Mat());//��һ��
		minMaxLoc(temp_rslt, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
		matchLoc = minLoc;
		
		//*******************ģ������ͨ��Ƚ�***********************//
		distance = sqrt(fabs(((maxRect.x) - (minLoc.x))*((maxRect.x) - (minLoc.x)) + ((maxRect.y) - (minLoc.y))*((maxRect.y) - (minLoc.y))));
		if (distance<150.0)
		{		
			rectangle(frame, matchLoc, Point(matchLoc.x + templa.cols, matchLoc.y + templa.rows), Scalar::all(2), 2, 8, 0);
			if (matchLoc.x > 220 && matchLoc.x < 420 && matchLoc.y>150 && matchLoc.y < 340)
			{
				putText(frame, "TARGET LOCKED", Point(130, 240), 1, 3, Scalar(0, 0, 255, 255), 3.0, false);
				Serial_Write("2");//�ɹ�����Ŀ�꣬����ź�1
			}
			else
			{
				putText(frame, "TARGET FOUND", Point(140, 240), 1, 3, Scalar(255, 255, 120, 0), 3.0, false);
				Serial_Write("1");//������������0
			}
		}
		else
		{
			putText(frame, "TARGET LOST", Point(160, 240), 1, 3, Scalar(255, 0, 0, 0), 3.0, false);
			Serial_Write("0");//������������0
		}
		//********************��ʾ����********************//
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

		//***************��ʾ���***********************//
		imshow("�������", frame);

		//*********************��ʱ�ȴ�****************//
		waitKey(10);
	}
	Serial_Close();//�رմ���
	return 0;
}

