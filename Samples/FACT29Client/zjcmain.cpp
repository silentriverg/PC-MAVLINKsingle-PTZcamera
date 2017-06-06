#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream> 
#include "SerialPort.h"  
#include <iomanip>
#include "SerialPort.h"
#include "processMavlink.h"
#include "controlPosition.h"
#include <conio.h>




using namespace std;


//串口部分――预定义
CMavLinkProcessor myMavLinkProcessor1; 
Mode myMode;
CPositionController myPositionController(&myMavLinkProcessor1, &myMode);

//-------------------------end--------------------------------------------------------

int main()
{
	//-----------------------begain------------------------------------------------------
	//串口部分――初始化
	int comNu1 = 3, baud = 57600 ,disp_flag = 1;
	/*
	cout << "输入串口号1: ";
	cin >> comNu1;
	cout << "输入串口号2: ";
	cin >> comNu2;
	cout << "输入串口号3: ";
	cin >> comNu3;
	*/

	
	
	int KeyCommand;


	/*
	cout<<"是否显示输出（1是,0否）:";
	cin>>disp_flag;

	cout<<"每几帧计算一帧：";//4
	cin>>tc_Control;
	cout<<"每几帧发一帧：";//三架飞机的数据发送是8，单架飞机可为5
	cin>>tc_Port;
	cout<<"串口号:";
	cin>>comNu1;
	cout<<"波特率:";
	cin>>baud;
	*/
	
	if (!myMavLinkProcessor1.InitPort(comNu1,baud))  
	{  
		cout << "initPort fail !" <<endl;  //以串口实际发送的数据为准（一般初始化失败也能正确发送数据）
	}  
	else 
	{  
		cout << "initPort success !" << endl;  
	}  

	

	if (myMavLinkProcessor1.OpenListenThread())
	{
		cout << "open listen thread success !" <<endl;
	}
	else
	{
		cout << "open listen thread fail !" <<endl;
	}



	
	if (!myPositionController.OpenControllerThread())  
	{  
		cout << "open Controller thread fail !" <<endl;  
	}  
	else 
	{  
		cout << "open Controller thread success !" << endl;  
	} 
	
	
	
	
	
	
	while(KeyCommand =_getch())
	{
		//myMavLinkProcessor1.parseCharStream (Attitude);
		//cout << "Lat: " << Attitude[0] <<  " Lon: " << Attitude[1] << " Alt: " << Attitude[2] << endl;
		
		switch(KeyCommand)
		{

		case'i':
			myMode = MODE_INIT;//所有四旋翼均为初始化模式
			cout<<"所有四旋翼均为初始化模式"<<endl;
			break;
		case'f':
			myMode = MODE_POINT;//所有四旋翼均为编队模式
			cout<<"所有四旋翼均为编队模式"<<endl;
			break;
		case't':
			myMode = MODE_TAKEOFF;//所有四旋翼均为起飞模式
			cout<<"所有四旋翼均为起飞模式"<<endl;
			break;
		case'c':
			myMode = MODE_CIRCLE;
			cout<<"所有四旋翼均为转圈模式"<<endl;
			break;
		case'r':
			myMode = MODE_RETURN;
			cout<<"所有四旋翼均为返航模式"<<endl;
			break;

		//---------------------------------------------------
		case'a':
			myMode = MODE_UAV_LEFT;
			cout<<"无人机开始左移"<<endl;
			break;
		case'd':
			myMode = MODE_UAV_RIGHT;
			cout<<"无人机开始右移"<< endl;
			break;
		case'w':
			myMode = MODE_UAV_FORWARD;
			cout<<"无人机开始前移"<<endl;
			break;
		case's':
			myMode = MODE_UAV_BACKWARD;
			cout<<"无人机开始后移"<<endl;
			break;
		case'q':
			myMode = MODE_UAV_UP;
			cout<<"无人机开始上升"<<endl;
			break;
		case'e':
			myMode = MODE_UAV_DOWN;
			cout<<"无人机开始下降"<<endl;
			break;
		case'x':
			myMode = MODE_UAV_HOLD;
			cout<<"无人机开始悬停"<<endl;
			break;
		//---------------------------------------------------
		case'h':
			myMode = MODE_PTZ_LEFT;
			cout<<"云台左转"<<endl;
			break;
		case'k':
			myMode = MODE_PTZ_RIGHT;
			cout<<"云台右转"<<endl;
			break;
		case'u':
			myMode = MODE_PTZ_UP;
			cout<<"云台抬头"<<endl;
			break;
		case'n':
			myMode = MODE_PTZ_DOWN;
			cout<<"云台低头"<<endl;
			break;
		case'j':
			myMode = MODE_PTZ_HOLD;
			cout<<"云台静止"<<endl;
			break;
		//---------------------------------------------
		default:
			cout<<"无此指令或关闭大写"<<endl;
			break;
		}
	}

	getchar();

}




