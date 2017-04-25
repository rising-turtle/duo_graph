#ifndef EXTERNALCOM_H
#define EXTERNALCOM_H

#include "Com.h"

typedef int (*CallBack_RecvData)(int nDataLen,char *pcData);
typedef int (*CallBack_Register)(int nSock,char *pcIP,int nStatus);
class ExternalCom:public Com
{
public:
	ExternalCom();
	~ExternalCom();

	virtual int ComRun();
	virtual int ComInit(char *pcData);

	static CallBack_RecvData m_cbRecvData;
	static CallBack_Register m_cbRegister;
	static int SendData2Cleint(int nDataLen,char *pcData,int nSock);
	//static CallBack_Reister m_cbRegister;
	//static CallBack_Data m_cbData;
private:


	pthread_t m_hThreadClientProcess;

	static void* ThreadClientProcess(void* lpParam);
	static int HeartBit();

	typedef struct  ThreadClientProcessParams
	{
			bool bRecv;
			ExternalCom *pCExternalCom;
			char cIP[4];
			int nSock;
	}ThreadClientProcessParams;
	static bool m_bStopThreadClientProcess;
	static ExternalCom *m_pThis;
	
	static pthread_mutex_t m_MutexSendData;


//	static int RegisterCenter(int nSock,int nID,int nType);
//	static int DataCenter(int nSock,int nID,char *pcData,int nDataLen);
};

#endif
