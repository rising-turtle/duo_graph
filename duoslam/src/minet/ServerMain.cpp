#include "ExternalCom.h"
#include "pthread.h"
#include <string.h>
#include <unistd.h>
int g_nTestSock=-1;


int RecvDataFromClient(int nDataLen,char *pcData)
{
	char cShow[1000];
	memset(cShow,1000,0);
	memcpy(cShow,pcData,nDataLen);
	printf("Server Recv Data Len:%d  \n, Content:%s  \n",nDataLen,cShow);
	return 0;
}

void* ThreadClientProcess(void* lpParam)
{
	ExternalCom * pCExternalCom=(ExternalCom *) lpParam;
	pCExternalCom->ComRun();
}


void* ThreadSendData2Client(void* lpParam)
{
	ExternalCom * pCExternalCom=(ExternalCom *) lpParam;

	char *pcData="I am Server";
	while(1)
	{
		if(g_nTestSock>=0)
		{
			pCExternalCom->SendData2Cleint(strlen(pcData),pcData,g_nTestSock);
			sleep(1);
		}
		usleep(10000);
	}
}

int Register(int nSock,char *pcIP,int nStatus)
{
	if(nStatus==1)
	{
		g_nTestSock=nSock;
	}
	else
	{
		g_nTestSock=-1;
	}
}

int main()
{
	char *pcIP="127.0.0.1";
	int nPort=6787;
	char cIPConf[20];


	memset(cIPConf,0,20);
	memcpy(cIPConf,pcIP,strlen(pcIP));
	memcpy(&cIPConf[16],&nPort,4);

	ExternalCom CExternalCom;
	CExternalCom.ComInit(cIPConf);
	CExternalCom.m_cbRecvData=RecvDataFromClient;
	CExternalCom.m_cbRegister=Register;

	pthread_t m_hThreadClientProcess;
	pthread_t m_hThreadSendData2Client;
	
	pthread_create(&m_hThreadClientProcess,NULL,ThreadClientProcess,&CExternalCom);
	pthread_create(&m_hThreadSendData2Client,NULL,ThreadSendData2Client,&CExternalCom);
	while(1)
	{
		sleep(1);
	}
	return 0;
}
