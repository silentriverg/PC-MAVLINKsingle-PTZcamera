#include "SerialPort.h"  
#include "processMavlink.h"
#include <process.h>  
#include <iostream>  
#include <tchar.h>

using namespace std;

/** 线程退出标志 */   
bool CMavLinkProcessor::s_bExit = false;  
/** 当串口无数据时,sleep至下次查询间隔的时间,单位:秒 */   
const UINT SLEEP_TIME_INTERVAL = 5;  
 
CMavLinkProcessor::CMavLinkProcessor(void)  
: m_hListenThread(INVALID_HANDLE_VALUE)  
{  
    m_hComm = INVALID_HANDLE_VALUE;  
    m_hListenThread = INVALID_HANDLE_VALUE;  
 
    InitializeCriticalSection(&m_csCommunicationSync);  

	//==========begin write====================
	time_delay = 0.0;
	last_timestamp = 0;
	alpha = 0.0;
	fh = 50;
	fs = 100;
	pos_NED_x = 0.0;
	pos_NED_y = 0.0;
	pos_NED_z = 0.0;
	pos_NED_Vx = 0.0;
	pos_NED_Vy = 0.0;
	pos_NED_Vz = 0.0;
	time_stamp_prev = 0.0;
	//=========end write===============
 
}  
 
CMavLinkProcessor::~CMavLinkProcessor(void)  
{  
    CloseListenTread();  
    ClosePort();  
    DeleteCriticalSection(&m_csCommunicationSync);  
}  
 
bool CMavLinkProcessor::InitPort( UINT portNo /*= 1*/,UINT baud /*= CBR_9600*/,char parity /*= 'N'*/,  
                            UINT databits /*= 8*/, UINT stopsbits /*= 1*/,DWORD dwCommEvents /*= EV_RXCHAR*/ )  
{  
 
    /** 临时变量,将制定参数转化为字符串形式,以构造DCB结构 */   
    char szDCBparam[50];  
    sprintf_s(szDCBparam, "baud=%d parity=%c data=%d stop=%d", baud, parity, databits, stopsbits);  
 
    /** 打开指定串口,该函数内部已经有临界区保护,上面请不要加保护 */   
    if (!openPort(portNo))  
    {  
        return false;  
    }  
 
    /** 进入临界段 */   
    EnterCriticalSection(&m_csCommunicationSync);  
 
    /** 是否有错误发生 */   
    BOOL bIsSuccess = TRUE;  
 
    /** 在此可以设置输入输出的缓冲区大小,如果不设置,则系统会设置默认值.  
     *  自己设置缓冲区大小时,要注意设置稍大一些,避免缓冲区溢出  
     */ 
    /*if (bIsSuccess )  
    {  
        bIsSuccess = SetupComm(m_hComm,10,10);  
    }*/ 
 
    /** 设置串口的超时时间,均设为0,表示不使用超时限制 */ 
    COMMTIMEOUTS  CommTimeouts;  
    CommTimeouts.ReadIntervalTimeout         = 0;  
    CommTimeouts.ReadTotalTimeoutMultiplier  = 0;  
    CommTimeouts.ReadTotalTimeoutConstant    = 0;  
    CommTimeouts.WriteTotalTimeoutMultiplier = 0;  
    CommTimeouts.WriteTotalTimeoutConstant   = 0;   
    if ( bIsSuccess)  
    {  
        bIsSuccess = SetCommTimeouts(m_hComm, &CommTimeouts);  
    }  
 
    DCB  dcb;  
    if ( bIsSuccess )  
    {  
        // 将ANSI字符串转换为UNICODE字符串  
        DWORD dwNum = MultiByteToWideChar (CP_ACP, 0, szDCBparam, -1, NULL, 0);  
        wchar_t *pwText = new wchar_t[dwNum] ;  
        if (!MultiByteToWideChar (CP_ACP, 0, szDCBparam, -1, pwText, dwNum))  
        {  
            bIsSuccess = TRUE;  
        }  

        /** 获取当前串口配置参数,并且构造串口DCB参数 */    
		//bIsSuccess = GetCommState(m_hComm, &dcb) && BuildCommDCB(pwText, &dcb) ;//由于字符集问题，此句做过修改
		bIsSuccess = GetCommState(m_hComm, &dcb) && BuildCommDCB((LPCSTR)pwText, &dcb) ;
        /** 开启RTS flow控制 */   
        dcb.fRtsControl = RTS_CONTROL_ENABLE;   
 
        /** 释放内存空间 */   
        delete [] pwText;  
    }  
 
    if ( bIsSuccess )  
    {  
        /** 使用DCB参数配置串口状态 */   
        bIsSuccess = SetCommState(m_hComm, &dcb);  
    }  
          
    /**  清空串口缓冲区 */ 
    PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);  
 
    /** 离开临界段 */   
    LeaveCriticalSection(&m_csCommunicationSync);  
 
    return bIsSuccess==TRUE;  
}  
 
bool CMavLinkProcessor::InitPort( UINT portNo ,const LPDCB& plDCB )  
{  
    /** 打开指定串口,该函数内部已经有临界区保护,上面请不要加保护 */   
    if (!openPort(portNo))  
    {  
        return false;  
    }  
      
    /** 进入临界段 */   
    EnterCriticalSection(&m_csCommunicationSync);  
 
    /** 配置串口参数 */   
    if (!SetCommState(m_hComm, plDCB))  
    {  
        return false;  
    }  
 
    /**  清空串口缓冲区 */ 
    PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);  
 
    /** 离开临界段 */   
    LeaveCriticalSection(&m_csCommunicationSync);  
 
    return true;  
}  
 
void CMavLinkProcessor::ClosePort()  
{  
    /** 如果有串口被打开，关闭它 */ 
    if( m_hComm != INVALID_HANDLE_VALUE )  
    {  
        CloseHandle( m_hComm );  
        m_hComm = INVALID_HANDLE_VALUE;  
    }  
}  
 
bool CMavLinkProcessor::openPort( UINT portNo )  
{  
    /** 进入临界段 */   
    EnterCriticalSection(&m_csCommunicationSync);  
 
    /** 把串口的编号转换为设备名 */   
    char szPort[50];  
    sprintf_s(szPort, "COM%d", portNo);  
 
    /** 打开指定的串口 */   
    m_hComm = CreateFileA(szPort,  /** 设备名,COM1,COM2等 */   
              GENERIC_READ | GENERIC_WRITE, /** 访问模式,可同时读写 */     
              0,                            /** 共享模式,0表示不共享 */   
              NULL,                         /** 安全性设置,一般使用NULL */   
              OPEN_EXISTING,                /** 该参数表示设备必须存在,否则创建失败 */   
              FILE_ATTRIBUTE_SYSTEM | FILE_FLAG_WRITE_THROUGH,      
              0);      
 
    /** 如果打开失败，释放资源并返回 */   
    if (m_hComm == INVALID_HANDLE_VALUE)  
    {  
        LeaveCriticalSection(&m_csCommunicationSync);  
        return false;  
    }  
 
    /** 退出临界区 */   
    LeaveCriticalSection(&m_csCommunicationSync);  
 
    return true;  
}  
 
bool CMavLinkProcessor::OpenListenThread()  
{  
    /** 检测线程是否已经开启了 */   
    if (m_hListenThread != INVALID_HANDLE_VALUE)  
    {  
        /** 线程已经开启 */   
        return false;  
    }  
 
    s_bExit = false;  
    /** 线程ID */   
    UINT threadId;  
    /** 开启串口数据监听线程 */   
    m_hListenThread = (HANDLE)_beginthreadex(NULL, 0, ListenThread, this, 0, &threadId);  
    if (!m_hListenThread)  
    {  
        return false;  
    }  
    /** 设置线程的优先级,高于普通线程 */   
    if (!SetThreadPriority(m_hListenThread, THREAD_PRIORITY_ABOVE_NORMAL))  
    {  
        return false;  
    }  
 
    return true;  
}  
 
bool CMavLinkProcessor::CloseListenTread()  
{     
    if (m_hListenThread != INVALID_HANDLE_VALUE)  
    {  
        /** 通知线程退出 */   
        s_bExit = true;  
 
        /** 等待线程退出 */   
        Sleep(10);  
 
        /** 置线程句柄无效 */   
        CloseHandle( m_hListenThread );  
        m_hListenThread = INVALID_HANDLE_VALUE;  
    }  
    return true;  
}  
 
UINT CMavLinkProcessor::GetBytesInCOM()  
{  
    DWORD dwError = 0;  /** 错误码 */   
    COMSTAT  comstat;   /** COMSTAT结构体,记录通信设备的状态信息 */   
    memset(&comstat, 0, sizeof(COMSTAT));  
 
    UINT BytesInQue = 0;  
    /** 在调用ReadFile和WriteFile之前,通过本函数清除以前遗留的错误标志 */   
    if ( ClearCommError(m_hComm, &dwError, &comstat) )  
    {  
        BytesInQue = comstat.cbInQue; /** 获取在输入缓冲区中的字节数 */   
    }  
 
    return BytesInQue;  
}  
 
UINT WINAPI CMavLinkProcessor::ListenThread( void* pParam )  
{  
    /** 得到本类的指针 */   
    CMavLinkProcessor *pSerialPort = reinterpret_cast<CMavLinkProcessor*>(pParam);  
	int CurrentByte = 0;
	bool IsGlobalPos = false, IsHeartbeat = false, IsInPacket = false;
	uint8_t Payload_Length=0;
	uint8_t *TempPtr;
	uint8_t CurrentSys = 0;
	uint8_t *TempPtr2;
 
    // 线程循环,轮询方式读取串口数据  
    while (!pSerialPort->s_bExit)   
    {  
		
        UINT BytesInQue = pSerialPort->GetBytesInCOM();  
        /** 如果串口输入缓冲区中无数据,则休息一会再查询 */   
        if ( BytesInQue == 0 )  
        {  
            Sleep(SLEEP_TIME_INTERVAL); 
            continue;  
        }  
 
        /** 读取输入缓冲区中的数据并输出显示 */ 
        char cRecved = 0x00;  
		
        do 
        {  
            cRecved = 0x00;  
            if(pSerialPort->ReadChar(cRecved) == true)  
            {  

				//cout<<sizeof(cRecved);
                //std::cout << hex << cRecved ;  
				if ((unsigned char)cRecved == 0xfe && !IsInPacket)
				{
					//cout << endl;
					CurrentByte = 1;
					IsInPacket = true;
				}
				if (CurrentByte == MAVLINK_LENGTH_BYTE)
				{
					TempPtr = reinterpret_cast<uint8_t*>(&cRecved);
					Payload_Length = *TempPtr;

				}

				if (CurrentByte == MAVLINK_SYS_BYTE)
				{
					TempPtr2 = reinterpret_cast<uint8_t*>(&cRecved);
					CurrentSys = *TempPtr2;

				}
				if (CurrentByte == MAVLINK_HEADER_LENGTH)
				{

					if ((unsigned char)cRecved == 0x21)
					{
						//cout <<"global pos ";
						IsGlobalPos = true;
					}
					else
					{
						//cout << "heartbeat";
						IsGlobalPos = false;
					}
				
				}
				else if (CurrentByte <= MAVLINK_HEADER_LENGTH + Payload_Length && CurrentByte > MAVLINK_HEADER_LENGTH && IsGlobalPos == true)
				{
					if (CurrentSys == 1)
						pSerialPort->Payload_store_33_1[CurrentByte - MAVLINK_HEADER_LENGTH - 1] = cRecved;
					else if (CurrentSys == 2)
						pSerialPort->Payload_store_33_2[CurrentByte - MAVLINK_HEADER_LENGTH - 1] = cRecved;
					else if (CurrentSys == 3)
						pSerialPort->Payload_store_33_3[CurrentByte - MAVLINK_HEADER_LENGTH - 1] = cRecved;
				}


				if (CurrentByte > MAVLINK_HEADER_LENGTH + Payload_Length + 1)
				{
					IsInPacket = false;
					IsGlobalPos = false;
				}

				//printf("%02x ", (unsigned char)cRecved);
				CurrentByte++;
                continue;  
				
            }  
        }while(--BytesInQue); 
    }  
 
    return 0;  
}  
 
bool CMavLinkProcessor::ReadChar( char &cRecved )  
{  
    BOOL  bResult     = TRUE;  
    DWORD BytesRead   = 0;  
    if(m_hComm == INVALID_HANDLE_VALUE)  
    {  
        return false;  
    }  
 
    /** 临界区保护 */   
    EnterCriticalSection(&m_csCommunicationSync);  
 
    /** 从缓冲区读取一个字节的数据 */   
    bResult = ReadFile(m_hComm, &cRecved, 1, &BytesRead, NULL);  
    if ((!bResult))  
    {   
        /** 获取错误码,可以根据该错误码查出错误原因 */   
        DWORD dwError = GetLastError();  
 
        /** 清空串口缓冲区 */   
        PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);  
        LeaveCriticalSection(&m_csCommunicationSync);  
 
        return false;  
    }  
 
    /** 离开临界区 */   
    LeaveCriticalSection(&m_csCommunicationSync);  
 
    return (BytesRead == 1);  
 
}  
 
bool CMavLinkProcessor::WriteData( unsigned char* pData, unsigned int length )  
{  
    BOOL   bResult     = TRUE;  
    DWORD  BytesToSend = 0;  
    if(m_hComm == INVALID_HANDLE_VALUE)  
    {  
		printf("write data failed !!!!!\n");
        return false;  
    }  
 
    /** 临界区保护 */   
    EnterCriticalSection(&m_csCommunicationSync);  
 
    /** 向缓冲区写入指定量的数据 */   
    bResult = WriteFile(m_hComm, pData, length, &BytesToSend, NULL);  
    if (!bResult)    
    {  
		printf("write data failed !!!!!\n");
        DWORD dwError = GetLastError();  
        /** 清空串口缓冲区 */   
        PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);  
        LeaveCriticalSection(&m_csCommunicationSync);  
        return false;  
    }  

    /** 离开临界区 */   
	//printf("write data finished !!!!!\n");
    LeaveCriticalSection(&m_csCommunicationSync);  
    return true;  
}

bool CMavLinkProcessor::WriteFloatData(float floatData, bool displayFloat, bool displayUnsigned)
{  
	bool flag;
	unsigned char buff[4];
	//float data;
	memcpy(buff, &floatData, 4);
	//memcpy(&data,buff,4);
	//float* floatDataPoint = &floatData;
	//unsigned char *unsignedDataPoint=(unsigned char *) floatDataPoint;

	flag = WriteData(buff,4);//将float分成四字节数据发送
	cout<<int(flag)<<endl;

	if (displayFloat)//显示发送数据的float表示
	{
		cout<<"十进制：";
		cout<<buff[0]<<endl;
	}

	if (displayUnsigned)//显示发送数据的16进制表示
	{
		cout<<"十六进制：";
		for (int i = 0;i <4 ;i++)
		{
			printf("%02x ",buff[i]);
		}
		cout<<endl;
	}

	if (flag)
	{
		return true;
	} 
	else
	{
		return false;
	}


}

void CMavLinkProcessor::parseCharStream_1 (int Attitude[])
{
	int Lat, Lon, Yaw;
	
	int *ft = reinterpret_cast<int*>(&Payload_store_33_1[4]);			//纬度
	Lat = *ft;
	Attitude[0] = Lat;

	ft = reinterpret_cast<int*>(&Payload_store_33_1[8]);	//经度
	Lon = *ft;
	Attitude[1] = Lon;

	ft = reinterpret_cast<int*>(&Payload_store_33_1[26]);  //偏航角
	Yaw = *ft;
	Attitude[2] = Yaw;

	int16_t Vx, Vy;

	int16_t *ptr = reinterpret_cast<int16_t *>(&Payload_store_33_1[20]);	
	Vx = *ptr;
	Attitude[3] = Vx;

	ptr = reinterpret_cast<int16_t *>(&Payload_store_33_1[22]);	
	Vy = *ptr;
	Attitude[4] = Vy;

}

void CMavLinkProcessor::parseCharStream_2 (int Attitude[])
{
	int Lat, Lon, Yaw;

	int *ft = reinterpret_cast<int*>(&Payload_store_33_2[4]);			//纬度
	Lat = *ft;
	Attitude[0] = Lat;

	ft = reinterpret_cast<int*>(&Payload_store_33_2[8]);	//经度
	Lon = *ft;
	Attitude[1] = Lon;

	ft = reinterpret_cast<int*>(&Payload_store_33_2[26]);  //偏航角
	Yaw = *ft;
	Attitude[2] = Yaw;

	int16_t Vx, Vy;

	int16_t *ptr = reinterpret_cast<int16_t *>(&Payload_store_33_2[20]);	
	Vx = *ptr;
	Attitude[3] = Vx;

	ptr = reinterpret_cast<int16_t *>(&Payload_store_33_2[22]);	
	Vy = *ptr;
	Attitude[4] = Vy;

}

void CMavLinkProcessor::parseCharStream_3 (int Attitude[])
{
	int Lat, Lon, Yaw;

	int *ft = reinterpret_cast<int*>(&Payload_store_33_3[4]);			//纬度
	Lat = *ft;
	Attitude[0] = Lat;

	ft = reinterpret_cast<int*>(&Payload_store_33_3[8]);	//经度
	Lon = *ft;
	Attitude[1] = Lon;

	ft = reinterpret_cast<int*>(&Payload_store_33_3[26]);  //偏航角
	Yaw = *ft;
	Attitude[2] = Yaw;

	int16_t Vx, Vy;

	int16_t *ptr = reinterpret_cast<int16_t *>(&Payload_store_33_3[20]);	
	Vx = *ptr;
	Attitude[3] = Vx;

	ptr = reinterpret_cast<int16_t *>(&Payload_store_33_3[22]);	
	Vy = *ptr;
	Attitude[4] = Vy;

}



//==================begin write=====================================================================================


void CMavLinkProcessor::crc_init(uint16_t* crcAccum)
{
	*crcAccum = X25_INIT_CRC;
}

void CMavLinkProcessor::crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
	uint8_t tmp;
	tmp = data ^ (uint8_t)(*crcAccum &0xff);
	tmp ^= (tmp<<4);
	*crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}


uint16_t CMavLinkProcessor::crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
	uint16_t crcTmp;
	crc_init(&crcTmp);
	while (length--) {
		crc_accumulate(*pBuffer++, &crcTmp);
	}
	return crcTmp;
}

void CMavLinkProcessor::crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
	const uint8_t *p = (const uint8_t *)pBuffer;
	while (length--) {
		crc_accumulate(*p++, crcAccum);
	}
}

void CMavLinkProcessor::_mavlink_send_uart(const char *buf, uint8_t length)
{
	uint8_t temp = 0;
	//for(temp = 0; temp<length; temp++)
	//{
	//	printf("%02X ",(char)buf[temp]);
	//}
	//printf("haha\n");

	WriteData((unsigned char * )buf,length);
}

void CMavLinkProcessor::mavlink_finalize_message(mavlink_message_t* msg, uint8_t sysid, uint8_t compid, uint8_t length, uint8_t crc_extra)
{
	uint16_t checksum;
	uint8_t buf[MAVLINK_NUM_HEADER_BYTES];
	uint8_t ck[2];
	buf[0] = MAVLINK_STX;
	buf[1] = length;
	buf[2] = 0;
	buf[3] = sysid;
	buf[4] = compid;
	buf[5] = msg->msgid;
	checksum = crc_calculate((const uint8_t*)&buf[1], MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, _MAV_PAYLOAD_NON_CONST(msg), length);

	crc_accumulate(crc_extra, &checksum);

	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

	_mavlink_send_uart((const char *)buf, MAVLINK_NUM_HEADER_BYTES);
	_mavlink_send_uart(_MAV_PAYLOAD_NON_CONST(msg), length);
	_mavlink_send_uart((const char *)ck, 2);
}

void CMavLinkProcessor::mavlink_msg_vicon_position_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw)
{

	mavlink_vicon_position_estimate_t packet;
	//packet.usec = current_tx_seq;
	//printf("%llu\n",packet.usec);
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	//packet.z = current_tx_seq;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN);

	msg->msgid = MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_CRC);
	
}

void CMavLinkProcessor::mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system_id, uint8_t target_component_id, uint16_t command, uint8_t confirmation, float param[])
{

	/*

	mavlink_command_long_estimate_t packet;

	packet.param1 = param[0];
	packet.param2 = param[1];
	packet.param3 = param[2];
	packet.param4 = param[3];
	packet.param5 = param[4];
	packet.param6 = param[5];
	packet.param7 = param[6];
	packet.command = command;
	packet.target_system = target_system_id;
	packet.target_component = target_component_id;
	packet.confirmation = confirmation;

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);
	*/
	


	memcpy(&msg->payload64[0], &param[0], 4);
	memcpy(&msg->payload64[4], &param[1], 4);
	memcpy(&msg->payload64[8], &param[2], 4);
	memcpy(&msg->payload64[12], &param[3], 4);
	memcpy(&msg->payload64[16], &param[4], 4);
	memcpy(&msg->payload64[20], &param[5], 4);
	memcpy(&msg->payload64[24], &param[6], 4);
	memcpy(&msg->payload64[28], &command, 2);
	msg->payload64[30] = target_system_id;
	msg->payload64[31] = target_component_id;	
	msg->payload64[32] = confirmation;
	

	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMMAND_LONG_LEN, MAVLINK_MSG_ID_COMMAND_LONG_CRC);

}

void CMavLinkProcessor::mavlink_msg_set_pos_local_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t time, uint8_t target_system_id, uint8_t target_component_id, uint8_t coordinate, uint16_t type_mask, float attitude[])
{
	memcpy(&msg->payload64[0], &time, 4);
	msg->payload64[4] = target_system_id;
	msg->payload64[5] = target_component_id;
	msg->payload64[6] = coordinate;
	memcpy(&msg->payload64[7], &type_mask, 2);
	memcpy(&msg->payload64[9], &attitude[0], 4);
	memcpy(&msg->payload64[13], &attitude[1], 4);
	memcpy(&msg->payload64[17], &attitude[2], 4);
	memcpy(&msg->payload64[21], &attitude[3], 4);
	memcpy(&msg->payload64[25], &attitude[4], 4);
	memcpy(&msg->payload64[29], &attitude[5], 4);
	memcpy(&msg->payload64[33], &attitude[6], 4);
	memcpy(&msg->payload64[37], &attitude[7], 4);
	memcpy(&msg->payload64[41], &attitude[8], 4);
	memcpy(&msg->payload64[45], &attitude[9], 4);
	memcpy(&msg->payload64[49], &attitude[10], 4);

	
	msg->msgid = MAVLINK_MSG_ID_SET_POS_LOCAL_NED;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_POS_LOCAL_NED_LEN, MAVLINK_MSG_ID_SET_POS_LOCAL_NED_CRC);

}

void CMavLinkProcessor::mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t custom_mode, uint8_t mode_param[])
{

	mavlink_heartbeat_estimate_t packet;

	packet.custom_mode = custom_mode;
	packet.type = mode_param[0];
	packet.autopilot = mode_param[1];
	packet.base_mode = mode_param[2];
	packet.system_status = mode_param[3];
	packet.mavlink_version = mode_param[4];


	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
	
	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);

}

void CMavLinkProcessor::mavlink_msg_set_mode_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t custom_mode, uint8_t target_system, uint8_t base_mode)
{
	mavlink_set_mode_estimate_t packet;


	packet.custom_mode = custom_mode;
	packet.target_system = target_system;
	packet.base_mode = base_mode;

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_MODE_LEN);

	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_SET_MODE;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_MODE_LEN, MAVLINK_MSG_ID_SET_MODE_CRC);
}

void CMavLinkProcessor::mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float param[], uint16_t seq_command[], uint8_t target_frame_etc[])
{
	mavlink_mission_item_estimate_t packet;

	packet.param1 = param[0];
	packet.param2 = param[1];
	packet.param3 = param[2];
	packet.param4 = param[3];
	packet.x = param[4];
	packet.y = param[5];
	packet.z = param[6];
	packet.seq = seq_command[0];
	packet.command = seq_command[1];
	packet.target_system = target_frame_etc[0];
	packet.target_component = target_frame_etc[1];
	packet.frame = target_frame_etc[2];
	packet.current = target_frame_etc[3];
	packet.autocontinue = target_frame_etc[4];


	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_ITEM_LEN);

	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_MISSION_ITEM;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_ITEM_LEN, MAVLINK_MSG_ID_MISSION_ITEM_CRC);
}


void CMavLinkProcessor::mavlink_msg_set_pos_global_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t time, int32_t lat_lon[], float param[], uint16_t type_mask, uint8_t target_frame[])
{
	mavlink_set_pos_global_int_estimate_t packet;

	packet.time = time;
	packet.lat = lat_lon[0];
	packet.lon = lat_lon[1];
	packet.alt = param[0];
	packet.vx = param[1];
	packet.vy = param[2];
	packet.vz = param[3];
	packet.afx = param[4];
	packet.afy = param[5];
	packet.afz = param[6];
	packet.yaw = param[7];
	packet.yaw_rate = param[8];
	packet.type_mask = type_mask;
	packet.target_system = target_frame[0];
	packet.target_component = target_frame[1];
	packet.coordinate_frame = target_frame[2];


	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_POS_GLOBAL_INT_LEN);

	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_SET_POS_GLOBAL_INT;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_POS_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_POS_GLOBAL_INT_CRC);
}

void CMavLinkProcessor::mavlink_msg_rc_channel_override_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint16_t chan_raw[], uint8_t target_comp[])
{
	mavlink_rc_channel_override_t packet;

	packet.chan1_raw = chan_raw[0];
	packet.chan2_raw = chan_raw[1];
	packet.chan3_raw = chan_raw[2];
	packet.chan4_raw = chan_raw[3];
	packet.chan5_raw = chan_raw[4];
	packet.chan6_raw = chan_raw[5];
	packet.chan7_raw = chan_raw[6];
	packet.chan8_raw = chan_raw[7];
	packet.target_system = target_comp[0];
	packet.target_component = target_comp[1];

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE_LEN);

	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE_CRC);

}
//=========================end write======================