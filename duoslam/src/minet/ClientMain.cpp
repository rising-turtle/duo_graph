#include "ClientNet.h"
#include <string.h>
#include <unistd.h>
int RecvDataFromServer(char *pcData,int nDataLen)
{
	char cShow[1000];
	memset(cShow,0,1000);
	memcpy(cShow,pcData,nDataLen);
	printf("Cleint Recv Data Len:%d  \n, Content:%s  \n",nDataLen,cShow);
	return 0;
}
void* ThreadListen(void* lpParam)
{
	ClientNet *pCClientNet=(ClientNet*)lpParam;
	pCClientNet->Listen2Host();
}

void * ThreadSend2Client(void* lpParam)
{
	ClientNet *pCClientNet=(ClientNet*)lpParam;
	char *pcData="I am Cleint";
	while(1)
	{
		pCClientNet->SendData(pcData,strlen(pcData));
		sleep(1);
	}
}
int main()
{
	char *pcIPClient="127.0.0.1";
	int nPortClient=8765;

	char *pcIPHost="127.0.0.1";
	int nPortHost=6787;

	char cClientConf[40];
	memset(cClientConf,0,40);

	memcpy(cClientConf,pcIPClient,strlen(pcIPClient));
	memcpy(&cClientConf[16],&nPortClient,4);

	memcpy(&cClientConf[20],pcIPHost,strlen(pcIPHost));
	memcpy(&cClientConf[36],&nPortHost,4);

	ClientNet CClientNet;
	CClientNet.ClientNetInit(cClientConf);
	CClientNet.m_cbRecvDataFromServer=RecvDataFromServer;

	pthread_t m_hThreadListen;
	pthread_t m_hThreadSendData2Server;

	pthread_create(&m_hThreadListen,NULL,ThreadListen,&CClientNet);
	pthread_create(&m_hThreadSendData2Server,NULL,ThreadSend2Client,&CClientNet);

	while(1)
	{
		sleep(100);
	}
	
	return 0;
}
