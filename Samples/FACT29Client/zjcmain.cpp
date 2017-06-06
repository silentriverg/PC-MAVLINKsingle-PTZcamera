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


//���ڲ��֨D�DԤ����
CMavLinkProcessor myMavLinkProcessor1; 
Mode myMode;
CPositionController myPositionController(&myMavLinkProcessor1, &myMode);

//-------------------------end--------------------------------------------------------

int main()
{
	//-----------------------begain------------------------------------------------------
	//���ڲ��֨D�D��ʼ��
	int comNu1 = 3, baud = 57600 ,disp_flag = 1;
	/*
	cout << "���봮�ں�1: ";
	cin >> comNu1;
	cout << "���봮�ں�2: ";
	cin >> comNu2;
	cout << "���봮�ں�3: ";
	cin >> comNu3;
	*/

	
	
	int KeyCommand;


	/*
	cout<<"�Ƿ���ʾ�����1��,0��:";
	cin>>disp_flag;

	cout<<"ÿ��֡����һ֡��";//4
	cin>>tc_Control;
	cout<<"ÿ��֡��һ֡��";//���ܷɻ������ݷ�����8�����ܷɻ���Ϊ5
	cin>>tc_Port;
	cout<<"���ں�:";
	cin>>comNu1;
	cout<<"������:";
	cin>>baud;
	*/
	
	if (!myMavLinkProcessor1.InitPort(comNu1,baud))  
	{  
		cout << "initPort fail !" <<endl;  //�Դ���ʵ�ʷ��͵�����Ϊ׼��һ���ʼ��ʧ��Ҳ����ȷ�������ݣ�
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
			myMode = MODE_INIT;//�����������Ϊ��ʼ��ģʽ
			cout<<"�����������Ϊ��ʼ��ģʽ"<<endl;
			break;
		case'f':
			myMode = MODE_POINT;//�����������Ϊ���ģʽ
			cout<<"�����������Ϊ���ģʽ"<<endl;
			break;
		case't':
			myMode = MODE_TAKEOFF;//�����������Ϊ���ģʽ
			cout<<"�����������Ϊ���ģʽ"<<endl;
			break;
		case'c':
			myMode = MODE_CIRCLE;
			cout<<"�����������ΪתȦģʽ"<<endl;
			break;
		case'r':
			myMode = MODE_RETURN;
			cout<<"�����������Ϊ����ģʽ"<<endl;
			break;

		//---------------------------------------------------
		case'a':
			myMode = MODE_UAV_LEFT;
			cout<<"���˻���ʼ����"<<endl;
			break;
		case'd':
			myMode = MODE_UAV_RIGHT;
			cout<<"���˻���ʼ����"<< endl;
			break;
		case'w':
			myMode = MODE_UAV_FORWARD;
			cout<<"���˻���ʼǰ��"<<endl;
			break;
		case's':
			myMode = MODE_UAV_BACKWARD;
			cout<<"���˻���ʼ����"<<endl;
			break;
		case'q':
			myMode = MODE_UAV_UP;
			cout<<"���˻���ʼ����"<<endl;
			break;
		case'e':
			myMode = MODE_UAV_DOWN;
			cout<<"���˻���ʼ�½�"<<endl;
			break;
		case'x':
			myMode = MODE_UAV_HOLD;
			cout<<"���˻���ʼ��ͣ"<<endl;
			break;
		//---------------------------------------------------
		case'h':
			myMode = MODE_PTZ_LEFT;
			cout<<"��̨��ת"<<endl;
			break;
		case'k':
			myMode = MODE_PTZ_RIGHT;
			cout<<"��̨��ת"<<endl;
			break;
		case'u':
			myMode = MODE_PTZ_UP;
			cout<<"��̨̧ͷ"<<endl;
			break;
		case'n':
			myMode = MODE_PTZ_DOWN;
			cout<<"��̨��ͷ"<<endl;
			break;
		case'j':
			myMode = MODE_PTZ_HOLD;
			cout<<"��̨��ֹ"<<endl;
			break;
		//---------------------------------------------
		default:
			cout<<"�޴�ָ���رմ�д"<<endl;
			break;
		}
	}

	getchar();

}




