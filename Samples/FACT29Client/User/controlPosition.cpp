#include "controlPosition.h"
#include <math.h>

#define M_PI 3.14159265358979323846

using namespace std;

bool CPositionController::s_controlExit = false;
CMavLinkProcessor* CPositionController::SerialPortPtr1;
Mode* CPositionController::ControlModePtr;


CPositionController::CPositionController(CMavLinkProcessor* CMavLinkProcessorPtr1,Mode* ModePtr): m_controlThread(INVALID_HANDLE_VALUE) 
{
	m_controlThread = INVALID_HANDLE_VALUE;
	SerialPortPtr1 = CMavLinkProcessorPtr1;
	ControlModePtr = ModePtr;


}

CPositionController::~CPositionController()
{
	CloseControllerTread();  
}

bool CPositionController::OpenControllerThread()  
{  
	/** 检测线程是否已经开启了 */   
	if (m_controlThread != INVALID_HANDLE_VALUE)  
	{  
		/** 线程已经开启 */   
		return false;  
	}  

	s_controlExit = false;  
	/** 线程ID */   
	UINT controlThreadId;  
	/** 开启串口数据监听线程 */   
	m_controlThread = (HANDLE)_beginthreadex(NULL, 0, ControllerThread, this, 0, &controlThreadId);  
	if (!m_controlThread)  
	{  
		return false;  
	}  
	/** 设置线程的优先级,高于普通线程 */   

	if (!SetThreadPriority(m_controlThread, THREAD_PRIORITY_ABOVE_NORMAL))  
	{  
		return false;  
	}  

	return true;  
}  

bool CPositionController::CloseControllerTread()  
{     
	if (m_controlThread != INVALID_HANDLE_VALUE)  
	{  
		/** 通知线程退出 */   
		s_controlExit = true;  

		/** 等待线程退出 */   
		Sleep(10);  

		/** 置线程句柄无效 */   
		CloseHandle( m_controlThread );  
		m_controlThread = INVALID_HANDLE_VALUE;  
	}  
	return true;  
} 

UINT WINAPI CPositionController::ControllerThread( void* pParam)
{  
	/** 得到本类的指针 */   
	CPositionController *PositionControllerPtr = reinterpret_cast<CPositionController*>(pParam);  
	mavlink_message_t msg;

	bool initial_flag[4] = {true,true,true,true};
	int LatPre, LonPre, LatThis, LonThis;

	//---------for command long-------------
	float command_long_param_unlock[7] = {1,21196,0,0,0,0,0};
	float command_long_param_takeoff[7] = {0,0,0,0,0,0,10};

	//---------for heartbeat--------------
	uint8_t param_mode[5] = {6, 8, 0, 0, 3};
	
	//---------for mission item----------
	uint16_t mission_item_seq_command[2] = {0, 16};
	uint8_t mission_item_target_frame_etc[5] = {1, 1, 3, 2, 1};

	float NorthOffset = 15;
	float EastOffset = 0;
	double Leader_Init_Lat=0, Leader_Init_Lon, Leader_Init_Yaw, Leader_Current_Lat, Leader_Current_Lon, Leader_Current_Yaw, Leader_Current_Vx, Leader_Current_Vy;
	double Follower_Current_Lat_1, Follower_Current_Lon_1, Follower_Current_Lat_2, Follower_Current_Lon_2, Follower_Current_Yaw, Follower_Current_Vx, Follower_Current_Vy;
	int Attitude1[5], Attitude2[5], Attitude3[5];   //纬度，经度，偏航角, vx, vy
	float mission_item_param[7];

	float Pp = 0.1;
	
	//------------for set pos global int--------------
	int32_t lat_lon[2] = {0, 0};
	float param_set_pos_global_int[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t target_frame[3] = {1, 1, 6};
	uint16_t type_mask = 4039; //1991 to enable yaw rate
		

	//---------------------------------------------
	float Radius = 5;
	int k = 0;
	double x_pos_set_1, y_pos_set_1, z_pos_set_1, x_pos_set_2, y_pos_set_2, z_pos_set_2, x_pos_set_3, y_pos_set_3, z_pos_set_3;
	float vel_x_set_1, vel_y_set_1, vel_x_set_2, vel_y_set_2, vel_x_set_3, vel_y_set_3;
	float vel_z_set;
	double Circle_center_lat, Circle_center_lon;

	//-----------for MODE_SEARCH----------------
	float Rho = 1;
	double Src_Lat, Src_Lon;
	double Measure1, Measure2, Measure3;
	double K=5;

	//---------for PTZ------------------
	uint16_t chan_raw[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t target_comp[2] = {1, 1};
	int PWM_tilt = 1500;


	float command_long_param_servo[7] = {0,0,0,0,0,0,0};

	//--------for UAV control------------

	double V0 = 1;
		

	// 线程循环,轮询方式读取串口数据  
	while (true)   
	{  
		/*
		while (Leader_Init_Lat<10)
		{
			//if (SerialPortPtr1->OpenListenThread())
			//{}
			Sleep(1000);
			SerialPortPtr1->parseCharStream_2(Attitude1);

			//if (SerialPortPtr1->CloseListenTread())
			//{}

			cout << "wating GPS update" << endl;
			Sleep(100);
			

			cout << "Lat: " << Attitude1[0] <<  " Lon: " << Attitude1[1] << " Alt: " << Attitude1[2] << endl;
			Leader_Init_Lat = (double)Attitude1[0]/10000000;
			Leader_Init_Lon = (double)Attitude1[1]/10000000;
			Leader_Init_Yaw = (double)Attitude1[2]*M_PI/(100*180);

			Src_Lat = Leader_Init_Lat;
			Src_Lon = Leader_Init_Lon - 50/(30.9*3600);
			
		
			cout << "等待GPS信号" << endl;
			
			//break;
		}
		*/
		
		

		
		//cout<<"LatSet_1: "<< LatSet_1 << " LonSet_1: " << LonSet_1 << endl;
		

		Sleep(50);
		SerialPortPtr1->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
		

		
		switch(*ControlModePtr)
		{
			Sleep(100);
		case MODE_INIT:
			if(initial_flag[0] == true)
			{
				cout << "初始化,解锁（仅能触发一次）" << endl;
				//SerialPortPtr1->mavlink_msg_command_long_pack(255, 190, &msg, 1, 1, 400, 0, command_long_param_unlock);
				Sleep(100);
				initial_flag[0] = false;

				Sleep(50);
				//SerialPortPtr1->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
				//SerialPortPtr2->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
				break;
			}
			else
			{
				cout << "已执行解锁" << endl;
				Sleep(2000);

				Sleep(50);
				//SerialPortPtr1->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
				//SerialPortPtr2->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
				break;
			}
		case MODE_TAKEOFF:
			if(initial_flag[1] == true)
			{
				cout << "起飞（仅能触发一次）" << endl;
				SerialPortPtr1->mavlink_msg_command_long_pack(255,190,&msg,1, 1, 22, 0, command_long_param_takeoff);
				Sleep(100);
				initial_flag[1] = false;
				Sleep(1000);

				Sleep(50);
				//SerialPortPtr1->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
				//SerialPortPtr2->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
				break;
			}
			else
			{
				cout << "已执行起飞" << endl;

				Sleep(50);
				//SerialPortPtr1->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
				//SerialPortPtr2->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
				Sleep(2000);
				break;
			}
		case MODE_POINT:

			x_pos_set_1 = Leader_Init_Lat + NorthOffset*0.0001/11.088;
			y_pos_set_1 = Leader_Init_Lon + EastOffset*0.0001/(11.088*0.8660);
			mission_item_param[4] = x_pos_set_1;
			mission_item_param[5] = y_pos_set_1;
			mission_item_param[6] = 10;
			mission_item_target_frame_etc[0] = 1;

			SerialPortPtr1->mavlink_msg_mission_item_pack(255, 190, &msg,mission_item_param, mission_item_seq_command, mission_item_target_frame_etc);
			Sleep(50);

			Sleep(50);
			//SerialPortPtr1->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
			//SerialPortPtr2->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);

			cout << "飞定点" << endl;
			Sleep(500);
			break;

		case MODE_CIRCLE:


			//if (SerialPortPtr1->OpenListenThread())
			//{}
			Sleep(100);
			SerialPortPtr1->parseCharStream_1(Attitude1);


			//{}

			Leader_Current_Lat = (double)Attitude1[0]/10000000;
			Leader_Current_Lon = (double)Attitude1[1]/10000000;


			if (initial_flag[2] == true)
			{
				Circle_center_lat = Leader_Init_Lat;
				Circle_center_lon = Leader_Init_Lon;
				initial_flag[2] = false;
			}

			cout << Attitude1[0] << " " <<  Attitude1[1]  << endl;

			//北东地系，x正方向为北，y正方向为东，顺势针为正；故1号机领头，2号机位于右后方，3号机位于左后方

			x_pos_set_1 = Circle_center_lat + (Radius*cos(2*M_PI/16*(fmod(float(k),16))))*0.0001/11.088;
			y_pos_set_1 = Circle_center_lon + (Radius*sin(2*M_PI/16*(fmod(float(k),16))))*0.0001/(11.088*0.8660);


			vel_x_set_1 = -1.963 * sin(2*M_PI/16*(fmod(float(k),16))) + Pp*(x_pos_set_1 - Leader_Current_Lat)*10000*11.088;
			vel_y_set_1 = 1.963 * cos(2*M_PI/16*(fmod(float(k),16))) + Pp*(y_pos_set_1 - Leader_Current_Lon)*10000*11.088*0.8660;

			vel_z_set = 0;

			target_frame[0] = 1;

			param_set_pos_global_int[1] = vel_x_set_1;
			param_set_pos_global_int[2] = vel_y_set_1;
			param_set_pos_global_int[3] = vel_z_set;	

			SerialPortPtr1->mavlink_msg_set_pos_global_int_pack(255, 190, &msg, 0, lat_lon, param_set_pos_global_int, type_mask, target_frame);

			k++;


			Sleep(50);
			//SerialPortPtr1->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
			//SerialPortPtr2->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);

			Sleep(50);
			Sleep(400);
			cout << "绕圈" << endl;

			break;

		case MODE_RETURN:
			x_pos_set_1 = Leader_Init_Lat;
			y_pos_set_1 = Leader_Init_Lon;
			mission_item_param[4] = x_pos_set_1;
			mission_item_param[5] = y_pos_set_1;
			mission_item_param[6] = 10;
			mission_item_target_frame_etc[0] = 1;

			SerialPortPtr1->mavlink_msg_mission_item_pack(255, 190, &msg,mission_item_param, mission_item_seq_command, mission_item_target_frame_etc);
			Sleep(50);

			Sleep(50);
			//SerialPortPtr1->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
			//SerialPortPtr2->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);

			cout << "飞定点" << endl;
			Sleep(500);
			break;

		//=======================================================================================
		case MODE_UAV_LEFT:

			Sleep(100);
			SerialPortPtr1->parseCharStream_1(Attitude1);

			Leader_Current_Yaw = (double)Attitude1[2]*M_PI/(100*180);  //in rad

			vel_x_set_1 = V0*cos(-M_PI/2 + Leader_Current_Yaw);
			vel_y_set_1 = V0*sin(-M_PI/2 + Leader_Current_Yaw);
			vel_z_set = 0;

			target_frame[0] = 1;

			param_set_pos_global_int[1] = vel_x_set_1;
			param_set_pos_global_int[2] = vel_y_set_1;
			param_set_pos_global_int[3] = vel_z_set;	

			SerialPortPtr1->mavlink_msg_set_pos_global_int_pack(255, 190, &msg, 0, lat_lon, param_set_pos_global_int, type_mask, target_frame);

			k++;

			Sleep(200);
			cout << "左移" << endl;

			break;
		case MODE_UAV_RIGHT:

			Sleep(100);
			SerialPortPtr1->parseCharStream_1(Attitude1);

			Leader_Current_Yaw = (double)Attitude1[2]*M_PI/(100*180);  //in rad

			vel_x_set_1 = V0*cos(M_PI/2 + Leader_Current_Yaw);
			vel_y_set_1 = V0*sin(M_PI/2 + Leader_Current_Yaw);
			vel_z_set = 0;

			target_frame[0] = 1;

			param_set_pos_global_int[1] = vel_x_set_1;
			param_set_pos_global_int[2] = vel_y_set_1;
			param_set_pos_global_int[3] = vel_z_set;	

			SerialPortPtr1->mavlink_msg_set_pos_global_int_pack(255, 190, &msg, 0, lat_lon, param_set_pos_global_int, type_mask, target_frame);

			k++;

			Sleep(200);
			cout << "右移" << endl;

			break;
		case MODE_UAV_FORWARD:

			Sleep(100);
			SerialPortPtr1->parseCharStream_1(Attitude1);

			Leader_Current_Yaw = (double)Attitude1[2]*M_PI/(100*180);  //in rad

			vel_x_set_1 = V0*cos(Leader_Current_Yaw);
			vel_y_set_1 = V0*sin(Leader_Current_Yaw);
			vel_z_set = 0;

			target_frame[0] = 1;

			param_set_pos_global_int[1] = vel_x_set_1;
			param_set_pos_global_int[2] = vel_y_set_1;
			param_set_pos_global_int[3] = vel_z_set;	

			SerialPortPtr1->mavlink_msg_set_pos_global_int_pack(255, 190, &msg, 0, lat_lon, param_set_pos_global_int, type_mask, target_frame);

			k++;

			Sleep(200);
			cout << "前移" << endl;

			break;
		case MODE_UAV_BACKWARD:

			Sleep(100);
			SerialPortPtr1->parseCharStream_1(Attitude1);

			Leader_Current_Yaw = (double)Attitude1[2]*M_PI/(100*180);  //in rad

			vel_x_set_1 = V0*cos(-M_PI/2 + Leader_Current_Yaw);
			vel_y_set_1 = V0*sin(-M_PI/2 + Leader_Current_Yaw);
			vel_z_set = 0;

			target_frame[0] = 1;

			param_set_pos_global_int[1] = vel_x_set_1;
			param_set_pos_global_int[2] = vel_y_set_1;
			param_set_pos_global_int[3] = vel_z_set;	

			SerialPortPtr1->mavlink_msg_set_pos_global_int_pack(255, 190, &msg, 0, lat_lon, param_set_pos_global_int, type_mask, target_frame);

			k++;

			Sleep(200);
			cout << "后移" << endl;

			break;
		case MODE_UAV_UP:

			Sleep(100);
			SerialPortPtr1->parseCharStream_1(Attitude1);

			Leader_Current_Yaw = (double)Attitude1[2]*M_PI/(100*180);  //in rad

			vel_x_set_1 = 0;
			vel_y_set_1 = 0;
			vel_z_set = -0.5;

			target_frame[0] = 1;

			param_set_pos_global_int[1] = vel_x_set_1;
			param_set_pos_global_int[2] = vel_y_set_1;
			param_set_pos_global_int[3] = vel_z_set;	

			SerialPortPtr1->mavlink_msg_set_pos_global_int_pack(255, 190, &msg, 0, lat_lon, param_set_pos_global_int, type_mask, target_frame);

			k++;

			Sleep(200);
			cout << "左移" << endl;

			break;
		case MODE_UAV_DOWN:

			Sleep(100);
			SerialPortPtr1->parseCharStream_1(Attitude1);

			Leader_Current_Yaw = (double)Attitude1[2]*M_PI/(100*180);  //in rad

			vel_x_set_1 = 0;
			vel_y_set_1 = 0;
			vel_z_set = 0.5;

			target_frame[0] = 1;

			param_set_pos_global_int[1] = vel_x_set_1;
			param_set_pos_global_int[2] = vel_y_set_1;
			param_set_pos_global_int[3] = vel_z_set;	

			SerialPortPtr1->mavlink_msg_set_pos_global_int_pack(255, 190, &msg, 0, lat_lon, param_set_pos_global_int, type_mask, target_frame);

			k++;

			Sleep(200);
			cout << "左移" << endl;

			break;
		case MODE_UAV_HOLD:

			vel_x_set_1 = 0;
			vel_y_set_1 = 0;
			vel_z_set = 0;

			target_frame[0] = 1;

			param_set_pos_global_int[1] = vel_x_set_1;
			param_set_pos_global_int[2] = vel_y_set_1;
			param_set_pos_global_int[3] = vel_z_set;	

			SerialPortPtr1->mavlink_msg_set_pos_global_int_pack(255, 190, &msg, 0, lat_lon, param_set_pos_global_int, type_mask, target_frame);

			k++;

			Sleep(200);
			cout << "悬停" << endl;

			break;


		//=======================================================================================
	

		case MODE_PTZ_LEFT:

			command_long_param_servo[0] = 6;
			command_long_param_servo[1] = 1100;
			SerialPortPtr1->mavlink_msg_command_long_pack(255, 190, &msg, 1, 1, 183, 0, command_long_param_servo);
			Sleep(100);
			initial_flag[0] = false;

			Sleep(50);
			//SerialPortPtr1->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
			//SerialPortPtr2->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
			break;

		case MODE_PTZ_RIGHT:

			command_long_param_servo[0] = 6;
			command_long_param_servo[1] = 1900;
			SerialPortPtr1->mavlink_msg_command_long_pack(255, 190, &msg, 1, 1, 183, 0, command_long_param_servo);
			Sleep(100);
			initial_flag[0] = false;

			Sleep(50);
			//SerialPortPtr1->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
			//SerialPortPtr2->mavlink_msg_heartbeat_pack(255,190, &msg, 0, param_mode);
			break;
		case MODE_PTZ_UP:
			if(PWM_tilt < 1950)
				PWM_tilt = PWM_tilt+50;
			else
				cout<<"已到抬头极限"<<endl;
			chan_raw[5] = PWM_tilt;
			chan_raw[6] = 1500;

			target_comp[0] = 1;

			SerialPortPtr1->mavlink_msg_rc_channel_override_pack(255, 190, &msg, chan_raw, target_comp);

			Sleep(50);

			Sleep(50);

			cout << "云台抬头" << endl;
			Sleep(500);
			break;
		case MODE_PTZ_DOWN:
			if (PWM_tilt > 1050)
				PWM_tilt = PWM_tilt-50;
			else
				cout<<"已到低头极限"<<endl;

			chan_raw[5] = PWM_tilt;
			chan_raw[6] = 1500;

			target_comp[0] = 1;

			SerialPortPtr1->mavlink_msg_rc_channel_override_pack(255, 190, &msg, chan_raw, target_comp);

			Sleep(50);

			Sleep(50);

			cout << "云台低头" << endl;
			Sleep(500);
			break;

		case MODE_PTZ_HOLD:

			chan_raw[5] = PWM_tilt;
			chan_raw[6] = 1500;

			target_comp[0] = 1;

			SerialPortPtr1->mavlink_msg_rc_channel_override_pack(255, 190, &msg, chan_raw, target_comp);

			Sleep(50);

			Sleep(50);

			cout << "云台静止" << endl;
			Sleep(500);
			break;


		//==================================================================================

		
			
		default:
			//cout<<"等待命令"<<endl;
			break;
		}
		
		//cout << "在线程中" << endl;
		

	}  

	return 0;  
}  


float CPositionController::CalculateMinDistance(float Lat1, float Lon1, float Lat2, float Lon2, float Lat3, float Lon3) 
{
	float Distance_12, Distance_13, Distance_23;

	Distance_12 = sqrt (pow((Lat1-Lat2)*3600*30.9,2) + pow((Lon1-Lon2)*3600*30.9*cos(M_PI/6),2));

	Distance_13 = sqrt (pow((Lat1-Lat3)*3600*30.9,2) + pow((Lon1-Lon3)*3600*30.9*cos(M_PI/6),2));

	Distance_23 = sqrt (pow((Lat3-Lat2)*3600*30.9,2) + pow((Lon3-Lon2)*3600*30.9*cos(M_PI/6),2));

	if (Distance_12 > Distance_13 && Distance_12 > Distance_23)
		return Distance_12;
	else if (Distance_13 > Distance_12 && Distance_13 > Distance_23)
		return Distance_13;
	else if (Distance_23 > Distance_12 && Distance_23 > Distance_13)
		return Distance_23;
}

bool CPositionController::IsReachSetpoint(float Lat1, float Lon1, float set_x, float set_y)
{
	float Distance;
	float ReachRadius = 3;

	Distance = sqrt (pow((Lat1-set_x)*3600*30.9,2) + pow((Lon1-set_y)*3600*30.9*cos(M_PI/6),2));

	if (Distance < ReachRadius)
		return true;
	else
		return false;
}
