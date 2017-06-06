#include <process.h>  
#include <iostream>  
#include <tchar.h>
#include "processMavlink.h"
#include "genSensorData.h"

using namespace std;

enum Mode {MODE_INIT, MODE_TAKEOFF, MODE_POINT, MODE_CIRCLE, MODE_RETURN, MODE_UAV_LEFT, MODE_UAV_RIGHT, MODE_UAV_UP, MODE_UAV_DOWN, MODE_UAV_FORWARD, MODE_UAV_BACKWARD, MODE_UAV_HOLD, MODE_PTZ_LEFT, MODE_PTZ_RIGHT, MODE_PTZ_UP, MODE_PTZ_DOWN, MODE_PTZ_HOLD};

class CPositionController
{
public:
	CPositionController(CMavLinkProcessor* CMavLinkProcessorPtr1, Mode* ModePtr);
	~CPositionController();

	static CMavLinkProcessor* SerialPortPtr1;
	static Mode* ControlModePtr;

	bool OpenControllerThread();
	bool CloseControllerTread();

	float CalculateMinDistance(float Lat1, float Lon1, float Lat2, float Lon2, float Lat3, float Lon3);
	bool IsReachSetpoint(float Lat1, float Lon1, float set_x, float set_y);

	CSensorDataGenerator VirtualSensorDataGenerator;

private:


	/** 线程退出标志变量 */   
	static bool s_controlExit;  

	/** 线程句柄 */   
	volatile HANDLE    m_controlThread;  

	static UINT WINAPI ControllerThread(void* pParam);

};