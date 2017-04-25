#include "ClientNet.h"
#include <unistd.h>

pthread_mutex_t g_MutexSendData;
ClientNet::ClientNet(void)
{
	m_bStopListen = false;
}

ClientNet::~ClientNet(void)
{
}


int ClientNet::ClientNetInit(char *pucConfig)
{
	//WORD wVersionRequested;
	//WSADATA wsaData;

	int nHostPort,nClientPort;
	unsigned char ucHostIP[4],ucClientIP[4];
	char cTmp[17];
	
	memcpy(cTmp,pucConfig,16);
	ParseIP(cTmp,(char *)ucClientIP);

	//memcpy(ucClientIP,pucConfig,4);
	memcpy(&nClientPort,pucConfig+16,4);


	memcpy(cTmp,pucConfig+20,16);
	ParseIP(cTmp,(char *)ucHostIP);

	//memcpy(ucHostIP,pucConfig+8,4);
	memcpy(&nHostPort,pucConfig+36,4);

	//wVersionRequested = MAKEWORD( 1, 1 );
	//WSAStartup( wVersionRequested, &wsaData );


	SetAddr(nHostPort,ucHostIP,m_HostAddr);

	pthread_mutex_init(&g_MutexSendData,0);


	if((m_nClientSock=SetSocket(nClientPort,ucClientIP,m_ClientAddr))<0)
	{
		return -1;
	}
	Try2CncHost();

	return 0;
}
int ClientNet::SetAddr(int nPort,unsigned char *pucIP,sockaddr_in &addr)
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


int ClientNet::SetSocket(int nPort,unsigned char *pucIP,sockaddr_in &addr)
{
	int nSock,nRtn;
	char cIPStr[16];
	


	linger sLinger;
	int nKeepAlive=1,nKeepIdle=60,nKeepInterval=5,nKeepCount=3;



	if ((nSock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		return -3;
	}

	int nReuseAddr=1;
	setsockopt(nSock,SOL_SOCKET,SO_REUSEADDR,&nReuseAddr,sizeof(nReuseAddr));
	setsockopt(nSock ,SOL_SOCKET,SO_LINGER,(void*)&sLinger,sizeof(linger));


	setsockopt(nSock ,SOL_SOCKET,SO_KEEPALIVE,(void*)&nKeepAlive,sizeof(nKeepAlive));
	setsockopt(nSock ,SOL_TCP,TCP_KEEPIDLE,(void*)&nKeepIdle,sizeof(nKeepIdle));
	setsockopt(nSock ,SOL_TCP,TCP_KEEPINTVL,(void*)&nKeepInterval,sizeof(nKeepInterval));
	setsockopt(nSock ,SOL_TCP,TCP_KEEPCNT,(void*)&nKeepCount,sizeof(nKeepCount));

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


int ClientNet::Try2CncHost()
{
	int nRtn=-1,i;
	//memcpy(cRecv,)
	for(i=0;i<TRY_TIME;i++)
	{
//printf("try to connect with host!!!! socket :  %d\n",m_nClientSock);
//printf("host ip3 :%d  port:  %d  \n  ",m_HostAddr.sin_addr.s_addr,m_HostAddr.sin_port);
		if((nRtn=connect(m_nClientSock,(struct sockaddr *)&m_HostAddr,sizeof(struct sockaddr)))!=-1)
		{
			printf("client connect to server success\n");
			m_bStopListen=false;
			return 0;
		//	break;
		}
		else
		{
			printf("try to connect with host falied reason:  %d!!!!\n",nRtn);
		}
		
		usleep(10);
	}
	return nRtn;
}


int ClientNet::RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd)
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



int ClientNet::SendStream(int nGoalSize,char *pcTmpBuff,int sockfd)
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
	return 0;
}


int ClientNet::SendData(char *pcData,int nDataLen)
{
	int nRtn=-1;
	pthread_mutex_lock(&g_MutexSendData);
	char cLen[4];
	memcpy(cLen,&nDataLen,4);
	if(SendStream(4,cLen,m_nClientSock)==0)
	{
		if(SendStream(nDataLen,pcData,m_nClientSock)==0)
		{
			nRtn=0;
		}
	}
	pthread_mutex_unlock(&g_MutexSendData);
	return 0;
}
/*int ClientNet::SendSLAMData(char *pcData,int nDataLen)
{
	SendStream(nDataLen,pcData,m_nClientSock);
	return 0;
}
*/

int ClientNet::Listen2Host()
{
	int nRtn,nRecvDataLen;

	struct timeval timeout={0,200};
	bool bJump=false;

	char cTmp[1000];
	printf("start to listen to host\n");
	while (!bJump&&!m_bStopListen)
	{
		fd_set fdR; 
		FD_ZERO(&fdR);
		FD_SET(m_nClientSock,&fdR);
		switch (select(m_nClientSock + 1, &fdR, NULL, NULL , &timeout)) 
		{
		case -1:
			bJump=1;
			printf("sock value :  %d  \n",m_nClientSock);
			printf("ThreadCmd Loss Connection with Host!!!!!\n");
			break;
		case 0:   //no new data come
			break;
		default:
			nRtn=FD_ISSET(m_nClientSock,&fdR);
			if (nRtn)
			{
				if(RcvStream(4,(char*)&nRecvDataLen,m_nClientSock)!=-1)   
				{
					if(RcvStream(nRecvDataLen,cTmp,m_nClientSock)!=-1)
					{
						//Parse Cmd
						m_cbRecvDataFromServer(cTmp,nRecvDataLen);
						//printf("B recv:%s \n",cTmp);
					}
					else
					{
						bJump=true;
					}
				}
				else
				{

					bJump=true;
				}
			}
		}

		usleep(20);
	}
	if (!bJump)
	{
		return 0;
	}
	else return-1;
	
}

int ClientNet::ParseIP(char *pcData,char *pcIP)
{
	char cTmp[10];
	int i,j=0,k=0;
	memset(cTmp,0,10);
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
