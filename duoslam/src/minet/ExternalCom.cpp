#include "ExternalCom.h"
#include "pthread.h"
#include "string.h"


bool ExternalCom::m_bStopThreadClientProcess;
pthread_mutex_t ExternalCom::m_MutexSendData;
ExternalCom *ExternalCom::m_pThis;


CallBack_RecvData ExternalCom::m_cbRecvData;
CallBack_Register ExternalCom::m_cbRegister;


ExternalCom::ExternalCom()
{
	pthread_mutex_init(&m_MutexSendData,0);




	m_pThis=this;
}

ExternalCom::~ExternalCom()
{

}

int ExternalCom::ComInit(char *pcData)
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

int ExternalCom::ComRun()
{
	int nNewFd;
	int nRtn;
	struct sockaddr_in ClientAddr = {0};
	int nAddrLenn=sizeof(ClientAddr);
	ThreadClientProcessParams stClientProcessParams;
	if(listen(m_nServerSocket,BACKLOG)==-1)
	{
		printf("R2Com A listen error!! \n");
		return -3;
	}

	while (!m_bStopListen)
	{
		printf("ComA am waiting for connecting request!!!!! \n");
		if((nNewFd=accept(m_nServerSocket,(struct sockaddr *)&ClientAddr,(socklen_t*)&nAddrLenn))==-1)
		{
			printf("accept error!!! \n");
			continue;
		}

		linger sLinger;
		sLinger.l_onoff = 1;
		sLinger.l_linger = 5;
		int nKeepAlive=1,nKeepIdle=1,nKeepInterval=1,nKeepCount=3;

		setsockopt(nNewFd ,SOL_SOCKET,SO_LINGER,(void*)&sLinger,sizeof(linger));
		setsockopt(nNewFd ,SOL_SOCKET,SO_KEEPALIVE,(void*)&nKeepAlive,sizeof(nKeepAlive));
		setsockopt(nNewFd ,SOL_TCP,TCP_KEEPIDLE,(void*)&nKeepIdle,sizeof(nKeepIdle));
		setsockopt(nNewFd ,SOL_TCP,TCP_KEEPINTVL,(void*)&nKeepInterval,sizeof(nKeepInterval));
		setsockopt(nNewFd ,SOL_TCP,TCP_KEEPCNT,(void*)&nKeepCount,sizeof(nKeepCount));




		printf("server :A  got connection from %s\n",
			inet_ntoa(ClientAddr.sin_addr));

		

		stClientProcessParams.pCExternalCom=this;
		stClientProcessParams.bRecv=false;
		stClientProcessParams.nSock=nNewFd;

		int nCMD=1;
		int nCMDContent=1;
		char cCMD[20];
		char *pcCMD=cCMD;
		memcpy(pcCMD,&nCMD,4);
		memcpy(pcCMD+4,&nCMDContent,4);
		memcpy(pcCMD+8,stClientProcessParams.cIP,4);


		ParseIP(inet_ntoa(ClientAddr.sin_addr),stClientProcessParams.cIP);

		m_cbRegister(nNewFd,stClientProcessParams.cIP,1);

		m_bStopThreadClientProcess=false;
		while((nRtn=pthread_create(&m_hThreadClientProcess,NULL,ThreadClientProcess,&stClientProcessParams))!=0)
		{
			printf("A  Create Thread Failed!!!!  %d \n",nRtn);
		}
		usleep(1000);
		m_bStopThreadHeartBitMonitor=false;
		while((nRtn=pthread_create(&m_hThreadConnectHealthMonitor,NULL,ThreadConnectHealthMonitor,NULL))!=0)
		{
			printf("A  Create Thread Failed!!!!  %d \n",nRtn);
		}
	}

	return 0;
}


//UINT R2ComA::ThreadClientProcess(LPVOID lpParam)
void* ExternalCom::ThreadClientProcess(void* lpParam)
{
	pthread_detach(pthread_self());
	ThreadClientProcessParams stCPParams,*CPParams;
	struct timeval timeout={0,200};


	int nRtn,nRecvDataLen;

	//char *pcData=new char [1024*1024];
	//memset(pcData,0,1024*1024);
	CPParams=(ThreadClientProcessParams*)lpParam;
	memcpy(&stCPParams,CPParams,sizeof(ThreadClientProcessParams));
	ExternalCom *pCExternalCom=(ExternalCom *)CPParams->pCExternalCom;
	bool bJump=false;
	int nListPos;
	CPParams->bRecv=true;


	m_bStopThreadClientProcess=false;
	m_nNetStatus=0;

	int nClientID;
	char *pcData=NULL;
	while (!m_bStopThreadClientProcess&&m_nNetStatus!=-1)
	{
		fd_set fdR;
		FD_ZERO(&fdR);
		FD_SET(stCPParams.nSock,&fdR);
		switch (select(stCPParams.nSock + 1, &fdR, NULL, NULL , &timeout))
		{
			case -1:

				//m_stCallBackSet.cb3dsFile(-1,0,NULL,NULL);
				printf("ThreadCmd Loss Connection with Host!!!!!\n");
				m_bStopThreadHeartBitMonitor=true;
				m_bStopThreadClientProcess=true;
				m_nNetStatus=-1;
				break;
			case 0:   //no new data come
				break;
			default:
				nRtn=FD_ISSET(stCPParams.nSock,&fdR);
				if (nRtn)
				{
					if(pCExternalCom->RcvStream(4,(char*)&nRecvDataLen,stCPParams.nSock)!=-1)
					{
						pcData=new char [nRecvDataLen];
						if(pCExternalCom->RcvStream(nRecvDataLen,pcData,stCPParams.nSock)!=-1)
						{
							m_cbRecvData(nRecvDataLen,pcData);
						}
						else
						{
							m_bStopThreadHeartBitMonitor=true;
							m_bStopThreadClientProcess=true;
							m_nNetStatus=-1;
							//Lost Connection
							printf("M lost conection!!!2\n");
							break;
						}
						delete [] pcData;
						pcData=NULL;
					}
					else
					{
						//Lost Connection
						m_bStopThreadHeartBitMonitor=true;
						m_bStopThreadClientProcess=true;
						m_nNetStatus=-1;
						printf("M lost conection!!!3\n");
						break;
					}
				}
				else
				{
					m_bStopThreadHeartBitMonitor=true;
					m_bStopThreadClientProcess=true;
					m_nNetStatus=-1;
					printf("ThreadCmd Loss Connection with Host!!!!!\n");
					bJump=true;
					break;
				}
				break;
		}
		usleep(5000);
		m_nNetStatus=true;
	}

	if(pcData!=NULL)
	{
		delete [] pcData;
	}
	m_cbRegister(stCPParams.nSock,NULL,-1);
	printf("M thread end!!!\n");
	pthread_exit(0);

	return 0;
}


int ExternalCom::SendData2Cleint(int nDataLen,char *pcData,int nSock)
{
	//g_Critical_Section_R2ComA.Lock();
	pthread_mutex_lock(&m_MutexSendData);
	int nRtn=0;
	char cLen[4];
	memcpy(cLen,&nDataLen,4);
	if(m_pThis->SendStream(4,cLen,nSock)>=0)
	{
		if (m_pThis->SendStream(nDataLen,pcData,nSock)<0)
		{
			nRtn=-1;
		}
	}
	else
	{
		nRtn=-1;
	}
	pthread_mutex_unlock(&m_MutexSendData);
	//g_Critical_Section_R2ComA.Unlock();
	return nRtn;
}
