#pragma once

#include "SerialPort.h"

#define PAYLOAD_LENGTH_00 9
#define PAYLOAD_LENGTH_30 28
#define PAYLOAD_LENGTH_32 28
#define PAYLOAD_LENGTH_33 28
#define MAVLINK_HEADER_LENGTH 6
#define MAVLINK_CK_LENGTH 2
#define MAVLINK_LENGTH_BYTE 2
#define MAVLINK_SYS_BYTE 4

//===================begin write=========================================

#define COMNUM 3
#define BAUD 57600
#define DELAY_TIME   0.093//sec 10hz = 0.093s   0.031  0.0186

#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN 32
#define MAVLINK_MSG_ID_104_LEN 32
#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_CRC 56
#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE 104

#define MAVLINK_MSG_ID_COMMAND_LONG_LEN 33
#define MAVLINK_MSG_ID_76_LEN 33
#define MAVLINK_MSG_ID_COMMAND_LONG_CRC 152
#define MAVLINK_MSG_ID_COMMAND_LONG 76

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9
#define MAVLINK_MSG_ID_00_LEN 9
#define MAVLINK_MSG_ID_HEARTBEAT_CRC 50
#define MAVLINK_MSG_ID_HEARTBEAT 0

#define MAVLINK_MSG_ID_SET_MODE_LEN 6
#define MAVLINK_MSG_ID_11_LEN 6
#define MAVLINK_MSG_ID_SET_MODE_CRC 89
#define MAVLINK_MSG_ID_SET_MODE 11

#define MAVLINK_MSG_ID_SET_POS_LOCAL_NED_LEN 53
#define MAVLINK_MSG_ID_84_LEN 53
#define MAVLINK_MSG_ID_SET_POS_LOCAL_NED_CRC 143
#define MAVLINK_MSG_ID_SET_POS_LOCAL_NED 84

#define MAVLINK_MSG_ID_SET_POS_GLOBAL_INT_LEN 53
#define MAVLINK_MSG_ID_86_LEN 53
#define MAVLINK_MSG_ID_SET_POS_GLOBAL_INT_CRC 5
#define MAVLINK_MSG_ID_SET_POS_GLOBAL_INT 86

#define MAVLINK_MSG_ID_MISSION_ITEM_LEN 37
#define MAVLINK_MSG_ID_39_LEN 37
#define MAVLINK_MSG_ID_MISSION_ITEM_CRC 254
#define MAVLINK_MSG_ID_MISSION_ITEM 39

#define MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE_LEN 18
#define MAVLINK_MSG_ID_70_LEN 18
#define MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE_CRC 124
#define MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE 70


#define MAVLINK_CORE_HEADER_LEN 5 ///< Length of core header (of the comm. layer): message length (1 byte) + message sequence (1 byte) + message system id (1 byte) + message component id (1 byte) + message type id (1 byte)
#define MAVLINK_NUM_HEADER_BYTES (MAVLINK_CORE_HEADER_LEN + 1) ///< Length of all header bytes, including core and checksum
#define MAVLINK_STX 254
#define X25_INIT_CRC 0xffff
#define _MAV_PAYLOAD(msg) ((const char *)(&((msg)->payload64[0])))
#define _MAV_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload64[0])))


#ifndef __int8_t_defined
# define __int8_t_defined
typedef signed char             int8_t; 
typedef short int               int16_t;
typedef int                     int32_t;
# if __WORDSIZE == 64
typedef long int                int64_t;
# else
//__extension__
typedef long long int           int64_t;
# endif
#endif
/* Unsigned.  */
typedef unsigned char           uint8_t;
typedef unsigned short int      uint16_t;
#ifndef __uint32_t_defined
typedef unsigned int            uint32_t;
# define __uint32_t_defined
#endif
#if __WORDSIZE == 64
typedef unsigned long int       uint64_t;
#else
//__extension__
typedef unsigned long long int  uint64_t;
#endif


typedef struct __mavlink_vicon_position_estimate_t
{
	uint64_t usec; /*< Timestamp (microseconds, synced to UNIX time or since system boot)*/
	float x; /*< Global X position*/
	float y; /*< Global Y position*/
	float z; /*< Global Z position*/
	float roll; /*< Roll angle in rad*/
	float pitch; /*< Pitch angle in rad*/ 
	float yaw; /*< Yaw angle in rad*/
} mavlink_vicon_position_estimate_t;

typedef struct __mavlink_command_long_estimate_t
{
	float param1;
	float param2;
	float param3;
	float param4;
	float param5;
	float param6;
	float param7;
	uint16_t command;
	uint8_t target_system;
	uint8_t target_component;	
	uint8_t confirmation;
	
} mavlink_command_long_estimate_t;

typedef struct __mavlink_heartbeat_estimate_t
{
	uint32_t custom_mode;
	uint8_t type;
	uint8_t autopilot;
	uint8_t base_mode;
	uint8_t system_status;
	uint8_t mavlink_version;

} mavlink_heartbeat_estimate_t;

typedef struct __mavlink_set_mode_estimate_t
{
	uint32_t custom_mode;
	uint8_t target_system;
	uint8_t base_mode;

} mavlink_set_mode_estimate_t;

typedef struct __mavlink_mission_item_estimate_t
{
	float param1;
	float param2;
	float param3;
	float param4;
	float x;
	float y;
	float z;
	uint16_t seq;
	uint16_t command;
	uint8_t target_system;
	uint8_t target_component;
	uint8_t frame;
	uint8_t current;
	uint8_t autocontinue;
} mavlink_mission_item_estimate_t;

typedef struct __mavlink_set_pos_global_int_estimate_t
{
	uint32_t time;
	int32_t lat;
	int32_t lon;
	float alt;
	float vx;
	float vy;
	float vz;
	float afx;
	float afy;
	float afz;
	float yaw;
	float yaw_rate;
	uint16_t type_mask;
	uint8_t target_system;
	uint8_t target_component;
	uint8_t coordinate_frame;
} mavlink_set_pos_global_int_estimate_t;

typedef struct __mavlink_rc_channel_override_t
{
	uint16_t chan1_raw;
	uint16_t chan2_raw;
	uint16_t chan3_raw;
	uint16_t chan4_raw;
	uint16_t chan5_raw;
	uint16_t chan6_raw;
	uint16_t chan7_raw;
	uint16_t chan8_raw;
	uint8_t target_system;
	uint8_t target_component;
} mavlink_rc_channel_override_t;

typedef struct mavlink_message{
	uint8_t magic;
	uint8_t length;
	uint8_t seq;
	uint8_t sysid;
	uint8_t compid;
	uint8_t msgid;
	uint8_t payload64[255];
	uint8_t checksum[2];
} mavlink_message_t;

#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE 104

//===================end write=======================================

class CMavLinkProcessor:public CSerialPort
{
public:
	char Payload_store_00[PAYLOAD_LENGTH_00];
	char Payload_store_30[PAYLOAD_LENGTH_30];
	char Payload_store_32[PAYLOAD_LENGTH_32];
	char Payload_store_33_1[PAYLOAD_LENGTH_33];
	char Payload_store_33_2[PAYLOAD_LENGTH_33];
	char Payload_store_33_3[PAYLOAD_LENGTH_33];
	
	void parseCharStream_1 (int* );
	void parseCharStream_2 (int* );
	void parseCharStream_3 (int* );
public:  
    CMavLinkProcessor(void);  
    ~CMavLinkProcessor(void);  
 
public:  
      
    /** ��ʼ�����ں���  
     *  
     *  @param:  UINT portNo ���ڱ��,Ĭ��ֵΪ1,��COM1,ע��,������Ҫ����9  
     *  @param:  UINT baud   ������,Ĭ��Ϊ9600  
     *  @param:  char parity �Ƿ������żУ��,'Y'��ʾ��Ҫ��żУ��,'N'��ʾ����Ҫ��żУ��  
     *  @param:  UINT databits ����λ�ĸ���,Ĭ��ֵΪ8������λ  
     *  @param:  UINT stopsbits ֹͣλʹ�ø�ʽ,Ĭ��ֵΪ1  
     *  @param:  DWORD dwCommEvents Ĭ��ΪEV_RXCHAR,��ֻҪ�շ�����һ���ַ�,�����һ���¼�  
     *  @return: bool  ��ʼ���Ƿ�ɹ�  
     *  @note:   ��ʹ�����������ṩ�ĺ���ǰ,���ȵ��ñ��������д��ڵĳ�ʼ��  
     *����������   /n�������ṩ��һЩ���õĴ��ڲ�������,����Ҫ����������ϸ��DCB����,��ʹ�����غ���  
     *           /n������������ʱ���Զ��رմ���,�������ִ�йرմ���  
     *  @see:      
     */ 
    bool InitPort( UINT  portNo = 1,UINT  baud = CBR_9600,char  parity = 'N',UINT  databits = 8, UINT  stopsbits = 1,DWORD dwCommEvents = EV_RXCHAR);  
 
    /** ���ڳ�ʼ������  
     *  
     *  �������ṩֱ�Ӹ���DCB�������ô��ڲ���  
     *  @param:  UINT portNo  
     *  @param:  const LPDCB & plDCB  
     *  @return: bool  ��ʼ���Ƿ�ɹ�  
     *  @note:   �������ṩ�û��Զ���ش��ڳ�ʼ������  
     *  @see:      
     */ 
    bool InitPort( UINT  portNo ,const LPDCB& plDCB );  
 
    /** ���������߳�  
     *  
     *  �������߳���ɶԴ������ݵļ���,�������յ������ݴ�ӡ����Ļ���  
     *  @return: bool  �����Ƿ�ɹ�  
     *  @note:   ���߳��Ѿ����ڿ���״̬ʱ,����flase  
     *  @see:      
     */ 
    bool OpenListenThread();  
 
    /** �رռ����߳�  
     *  
     *    
     *  @return: bool  �����Ƿ�ɹ�  
     *  @note:   ���ñ�������,�������ڵ��߳̽��ᱻ�ر�  
     *  @see:      
     */ 
    bool CloseListenTread();  
 
    /** �򴮿�д����  
     *  
     *  ���������е�����д�뵽����  
     *  @param:  unsigned char * pData ָ����Ҫд�봮�ڵ����ݻ�����  
     *  @param:  unsigned int length ��Ҫд������ݳ���  
     *  @return: bool  �����Ƿ�ɹ�  
     *  @note:   length��Ҫ����pData��ָ�򻺳����Ĵ�С  
     *  @see:      
     */ 
    bool WriteData(unsigned char* pData, unsigned int length);  
 
    /** ��ȡ���ڻ������е��ֽ���  
     *  
     *    
     *  @return: UINT  �����Ƿ�ɹ�  
     *  @note:   �����ڻ�������������ʱ,����0  
     *  @see:      
     */ 
    UINT GetBytesInCOM();  
 
    /** ��ȡ���ڽ��ջ�������һ���ֽڵ�����  
     *  
     *    
     *  @param:  char & cRecved ��Ŷ�ȡ���ݵ��ַ�����  
     *  @return: bool  ��ȡ�Ƿ�ɹ�  
     *  @note:     
     *  @see:      
     */ 
    bool ReadChar(char &cRecved);  
 
private:  
 
    /** �򿪴���  
     *  
     *    
     *  @param:  UINT portNo �����豸��  
     *  @return: bool  ���Ƿ�ɹ�  
     *  @note:     
     *  @see:      
     */ 
    bool openPort( UINT  portNo );  
 
    /** �رմ���  
     *  
     *    
     *  @return: void  �����Ƿ�ɹ�  
     *  @note:     
     *  @see:      
     */ 
    void ClosePort();  
      
    /** ���ڼ����߳�  
     *  
     *  �������Դ��ڵ����ݺ���Ϣ  
     *  @param:  void * pParam �̲߳���  
     *  @return: UINT WINAPI �̷߳���ֵ  
     *  @note:     
     *  @see:      
     */ 

    static UINT WINAPI ListenThread(void* pParam);  
 
private:
 
    /** ���ھ�� */   
    HANDLE  m_hComm;  
 
    /** �߳��˳���־���� */   
    static bool s_bExit;  
 
    /** �߳̾�� */   
    volatile HANDLE    m_hListenThread;  
 
    /** ͬ������,�ٽ������� */   
    CRITICAL_SECTION   m_csCommunicationSync;       //!< �����������  
 

 
//------------------------------------------------------��ӵĺ�������--------------------------------------
public:
    /** �򴮿�дfloat���ݣ��ȴ����λ�ֽڣ�
     *  
     *  ���������е�����д�뵽����  
     *  @param:  float floatData ��Ҫ���͵�float���͵�����
	 *  @param:  bool displayFloat �Ƿ���ʾ���͵�float���ݣ�Ĭ�ϲ���ʾ
	 *  @param:  bool displayUnsigned �Ƿ���ʾ���͵�����ת����unsigned char����ʽ��Ĭ�ϲ���ʾ
     *  @return: bool  �����Ƿ�ɹ�  
     *  @note:   
     *  @see:      
     */ 
    bool WriteFloatData(float floatData,bool displayFloat = false,bool displayUnsigned = false); 

	//=======================begin write============================================

public:
	double last_time_get_pos;
	//DWORD time_delay = GetTickCount();
	double time_delay;
	double last_timestamp;
	double alpha;
	double fh;
	double fs;
	float pos_NED_x;
	float pos_NED_y;
	float pos_NED_z;
	float pos_NED_Vx;
	float pos_NED_Vy;
	float pos_NED_Vz;
	LARGE_INTEGER litmp; 
	LONGLONG QPart1,QPart2;  
	double dfMinus, dfFreq, dfTim; 
	double time_stamp_prev;

	//   -YJ-
	float *QRoll;
	float  *QPitch;
	float *QYaw;

	struct _pos{
		float x;
		float y;
		float z;
	} last_pos;
	struct _vel{
		float v_x;
		float v_y;
		float v_z;
	} vel;


	

	void crc_init(uint16_t* crcAccum);

	void crc_accumulate(uint8_t data, uint16_t *crcAccum);


	uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length);

	void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length);

	void _mavlink_send_uart(const char *buf, uint8_t length);

	void mavlink_finalize_message(mavlink_message_t* msg, uint8_t sysid, uint8_t compid, uint8_t length, uint8_t crc_extra);

	void mavlink_msg_vicon_position_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw);

	void mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system_id, uint8_t target_component_id, uint16_t command, uint8_t confirmation, float param[]);

	void mavlink_msg_set_pos_local_ned_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t time, uint8_t target_system_id, uint8_t target_component_id, uint8_t coordinate, uint16_t type_mask, float attitude[]);

	void mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t custom_mode, uint8_t mode_param[]);

	void mavlink_msg_set_mode_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t custom_mode, uint8_t target_system, uint8_t base_mode);

	void mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float param[], uint16_t seq_command[], uint8_t target_frame_etc[]);

	void mavlink_msg_set_pos_global_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t time, int32_t lat_lon[], float param[], uint16_t type_mask, uint8_t target_frame[]);

	void mavlink_msg_rc_channel_override_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint16_t chan_raw[], uint8_t target_comp[]);
	//===========================end write========================

};
