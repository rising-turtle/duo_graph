#ifndef CLIENTNET_H
#define CLIENTNET_H

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
#include <stdlib.h>

#include <string.h>
#define TRY_TIME 10

typedef int (*CallBack_RecvDataFromServer)(char *pcData,int nDataLen);
class ClientNet
{
public:
	ClientNet(void);
	~ClientNet(void);

	int ClientNetInit(char *pucConfig);
	int SendData(char *pcData,int nDataLen);
	//int SendSLAMData(char *pcData,int nDataLen);

	
	int Listen2Host();
	bool m_bStopListen;

	CallBack_RecvDataFromServer m_cbRecvDataFromServer;
	//CallBack_NetUpload m_cbNetUpload;
	//CallBack_LogFile m_cbLogFile;
private:
	int SetAddr(int nPort,unsigned char *pucIP,sockaddr_in &addr);
	int SetSocket(int nPort,unsigned char *pucIP,sockaddr_in &addr);
	int Try2CncHost();
	int RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd);
	int SendStream(int nGoalSize,char *pcTmpBuff,int sockfd);


	struct sockaddr_in m_HostAddr;
	struct sockaddr_in m_ClientAddr;
	
	int m_nClientSock;
	int ParseIP(char *pcData,char *pcIP);
};

#endif
