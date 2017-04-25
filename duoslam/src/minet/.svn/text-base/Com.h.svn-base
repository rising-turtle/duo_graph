#ifndef COM_H
#define COM_H


#include <iostream>
#include <string>

using namespace std;

#define BACKLOG 10


#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>   

#include <arpa/inet.h>
#include <sys/types.h>
#include <netinet/tcp.h>

#include "pthread.h"

#define NET_STATUS_OK_BIT_NUM 5
#define NET_STATUS_NORMAL_BIT_NUM 3
#define NET_STATUS_SLOW_BIT_NUM 1
#define NET_STATUS_UNKOWN_BIT_NUM 0
#define NET_STATUS_INIT_NUM 10

class Com
{
public:
	Com();
	~Com();
	bool m_bStopListen;

	virtual int ComInit(char *pcData);
	virtual int ComRun()=0;
	int SendStream(int nGoalSize,char *pcTmpBuff,int sockfd);

	
protected:
	struct sockaddr_in ServerAddr;
	int m_nServerSocket;
	int ParseIP(char *pcData,char *pcIP);
	int RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd);
	
	int SetAddr(int nPort,unsigned char *pucIP,sockaddr_in &addr);
	int SetSocket(int nPort,unsigned char *pucIP,sockaddr_in &addr);
	int SetSockOpt(int nSock);
	unsigned char m_ucIP[4];

	pthread_t m_hThreadConnectHealthMonitor;
	static bool m_bStopThreadHeartBitMonitor;
	static void*ThreadConnectHealthMonitor(void* lpParam);
	static int m_nNetStatus;
	static int m_nHeartBitCount;

};

#endif
