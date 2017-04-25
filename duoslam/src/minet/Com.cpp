//#include <winsock2.h>
#include "Com.h"
#include <string.h>
#include <stdlib.h>

int Com::m_nNetStatus;
int Com::m_nHeartBitCount;
bool Com::m_bStopThreadHeartBitMonitor;
Com::Com()
{
	m_bStopListen = false;
}

Com::~Com()
{

}

int Com::ComInit(char *pcData)
{
	int nRslt,nPort;
	char cIP[4],cTmp[17];
	cTmp[16]=0;
	memcpy(cTmp,pcData,16);
	ParseIP(cTmp,cIP);
	memcpy(&nPort,pcData+16,4);
	SetAddr(nPort,(unsigned char*)cIP,ServerAddr);
	m_nServerSocket=SetSocket(nPort,(unsigned char*)cIP,ServerAddr);

	return 0;
}


int Com::RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd)
{
	int nRecvCount,nLeftLen,nNumBytes;

	nLeftLen=nGoalSize;
	nRecvCount=0;
	while (nRecvCount<nGoalSize)
	{
		if ((nNumBytes=recv(sockfd, pcTmpBuff, nLeftLen, 0)) <=0) 
		{
			return -1;
		}
		nRecvCount+=nNumBytes;
		pcTmpBuff+=nNumBytes;
		nLeftLen=nGoalSize-nRecvCount;
	}
	return 1;
}



int Com::SendStream(int nGoalSize,char *pcTmpBuff,int sockfd)
{
	int nRecvCount,nLeftLen,nNumBytes;

	nLeftLen=nGoalSize;
	nRecvCount=0;
	while (nRecvCount<nGoalSize)
	{
		if ((nNumBytes=send(sockfd, pcTmpBuff, nLeftLen, 0)) <=0) 
		{
			return -1;
		}
		nRecvCount+=nNumBytes;
		pcTmpBuff+=nNumBytes;
		nLeftLen=nGoalSize-nRecvCount;
	}
	return 1;
}

int Com::ParseIP(char *pcData,char *pcIP)
{
	char cTmp[10];
	int i,j=0,k=0;
	for (i=0;i<strlen(pcData);i++)
	{
		if (pcData[i]=='.')
		{
			pcIP[k++]=atoi(cTmp);
			j=0;
			memset(cTmp,0,10);

		}
		else
		{
			cTmp[j++]=pcData[i];
		}
	}
	pcIP[k]=atoi(cTmp);

	return 0;
}

int Com::SetAddr(int nPort,unsigned char *pucIP,sockaddr_in &addr)
{
	char cIPStr[16];
	addr.sin_family=AF_INET;
	addr.sin_port=htons(nPort);

	memset(cIPStr,0,16);
	sprintf(cIPStr, "%d.%d.%d.%d", pucIP[0],pucIP[1],pucIP[2],pucIP[3]);
	//printf("My IP  :%s port:  %d  \n",cIPStr,nPort);

	addr.sin_addr.s_addr=inet_addr(cIPStr);
	//printf("host ip1 :%d  port:  %d  \n  ",addr.sin_addr.s_addr,addr.sin_port);
	memset(addr.sin_zero,0,8);
	//printf("host ip2 :%d  port:  %d  \n  ",addr.sin_addr.s_addr,addr.sin_port);
	return 0;
}


int Com::SetSocket(int nPort,unsigned char *pucIP,sockaddr_in &addr)
{
	int nSock,nRtn,nOn;
	char cIPStr[16];



	linger sLinger;
	int nKeepAlive=1,nKeepIdle=60,nKeepInterval=5,nKeepCount=3;

	if ((nSock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		return -3;
	}
	setsockopt(nSock ,SOL_SOCKET,SO_LINGER,(void*)&sLinger,sizeof(linger));


	setsockopt(nSock ,SOL_SOCKET,SO_KEEPALIVE,(void*)&nKeepAlive,sizeof(nKeepAlive));
	setsockopt(nSock ,SOL_TCP,TCP_KEEPIDLE,(void*)&nKeepIdle,sizeof(nKeepIdle));
	setsockopt(nSock ,SOL_TCP,TCP_KEEPINTVL,(void*)&nKeepInterval,sizeof(nKeepInterval));
	setsockopt(nSock ,SOL_TCP,TCP_KEEPCNT,(void*)&nKeepCount,sizeof(nKeepCount));

	nOn = 1;
	setsockopt( nSock, SOL_SOCKET, SO_REUSEADDR, &nOn, sizeof(nOn) );

	addr.sin_family=AF_INET;

	addr.sin_port=htons(nPort);


	memset(cIPStr,0,16);
	sprintf(cIPStr, "%d.%d.%d.%d", pucIP[0],pucIP[1],pucIP[2],pucIP[3]);
	addr.sin_addr.s_addr=inet_addr(cIPStr);
	memset(addr.sin_zero,0, 8);

	if(bind(nSock,(struct sockaddr *)&addr,sizeof(struct sockaddr))==-1)
	{
		return -4;
	}
	return nSock;
}


int Com::SetSockOpt(int nSock)
{
	int nOn;
	linger sLinger;
	sLinger.l_onoff = 1;
	sLinger.l_linger = 5;
	int nKeepAlive=1,nKeepIdle=60,nKeepInterval=5,nKeepCount=3;

	setsockopt(nSock ,SOL_SOCKET,SO_LINGER,(void*)&sLinger,sizeof(linger));
	setsockopt(nSock ,SOL_SOCKET,SO_KEEPALIVE,(void*)&nKeepAlive,sizeof(nKeepAlive));
	setsockopt(nSock ,SOL_TCP,TCP_KEEPIDLE,(void*)&nKeepIdle,sizeof(nKeepIdle));
	setsockopt(nSock ,SOL_TCP,TCP_KEEPINTVL,(void*)&nKeepInterval,sizeof(nKeepInterval));
	setsockopt(nSock ,SOL_TCP,TCP_KEEPCNT,(void*)&nKeepCount,sizeof(nKeepCount));

	nOn = 1;
	setsockopt( nSock, SOL_SOCKET, SO_REUSEADDR, &nOn, sizeof(nOn) );
}

void* Com::ThreadConnectHealthMonitor(void* lpParam)
{
	pthread_detach(pthread_self());
	int nLostCncCout=0;

	//initial state
	while(m_nNetStatus==0)
	{
		nLostCncCout++;
		sleep(1);
		if(nLostCncCout>NET_STATUS_INIT_NUM)
		{
			m_nNetStatus=-1;
			m_bStopThreadHeartBitMonitor=true;
			break;
		}
	}
	//stable....
	while(!m_bStopThreadHeartBitMonitor)
	{
		if(nLostCncCout==10)
		{
			m_bStopThreadHeartBitMonitor=1;
			m_nNetStatus=-1;
			break;
		}
		if(m_nNetStatus>=NET_STATUS_OK_BIT_NUM)
		{
			nLostCncCout=0;
			m_nNetStatus=1;
		}
		else if(m_nNetStatus>=NET_STATUS_NORMAL_BIT_NUM)
		{
			nLostCncCout=0;
			m_nNetStatus=2;
		}
		else if(m_nNetStatus>=NET_STATUS_SLOW_BIT_NUM)
		{
			m_nNetStatus=3;
		}
		else
		{
			if(m_nNetStatus==4)
			{
				nLostCncCout++;
			}
			m_nNetStatus=4;
		}
		m_nNetStatus=0;
		sleep(1);
	}
	pthread_exit(0);
}


