/*****************************************************************************
Title: PC_TCP_v0
Features:
1. commnication between PC(Cpp) <-> RV-2A (for Vive edition, cf. Tablet)
2. [NOT YET] Unity(C#) <-> {PC(Cpp) <-> RV-2A}
Author: Tsai,Chia-Wen
Date: 2017/02/20
Last update: 2017/02/20
Status: OK! 但功能有限, 詳見Notes
Notes:
- 目前只完成PC & RV-2A的連線(純粹給j1~6, 並move)
- 待完成: 加thread
- 待完成: 與Unity連線
******************************************************************************/

#include <windows.h> 
#include "stdafx.h"
#include <iostream> ///
using namespace std; ///
#include <winsock.h> 
#include <stdio.h> 
#include <conio.h> 
#include <string.h> 
#include <math.h> 
#include <cstdio> ///
#include "strdef.h" 
#define NO_FLAGS_SET 0 
#define MAXBUFLEN 512
#define PI 3.14159265359

					 //robot angle (degree)
double actualAngle[6] = { 0, 0, 90, -25, 0, 90 };		// robot 實際角度
double commandAngle[6] = { 0, 0, 0, 0, 0, 0 };	        // 從actualAngle ~ targetAngle 之間的過渡(慢慢加)
double targetAngle[6] = { 30, 10, 120, 20, 0, 90 };		// 目標角度

/*
	targetAngle[0] = 0;    //range: ( -159.93 ~ +159.97 )  ; bound: +- 150 deg
	targetAngle[1] = 0;    //range: ( -45     ~ +93.71  )  ; bound: +- 40  deg
	targetAngle[2] = 90;   //range: ( +50.09  ~ +169.89 )  ; bound: +- 35  deg
	targetAngle[3] = -25;  //range: ( -159.87 ~ +138.11 )  ; bound: +- 130 deg
	targetAngle[4] = 0;    //range: ( -119.97 ~ +119.9  )  ; bound: +- 115 deg
	targetAngle[5] = 0;    //range: ( -199.88 ~ +200    )  ; bound: +- 190 deg
*/

WSADATA Data;
SOCKADDR_IN destSockAddr;
SOCKET destSocket;
unsigned long destAddr;
int status;
int numsnt;
int numrcv;
char sendText[MAXBUFLEN];
char recvText[MAXBUFLEN];
char buf[MAXBUFLEN];
char type = MXT_TYP_JOINT;			//	Send Joint Data
unsigned short IOSendType = MXT_IO_NULL;  // Send input/output signal data designation	//	x output signal
unsigned short IORecvType = MXT_IO_IN;  // Reply input/output signal data designation  //	x output signal


MXTCMD   MXTsend;		// 送到機器手臂的資料
MXTCMD   MXTrecv;		// 回傳到personal computer的資料
JOINT    jnt_now;		// Joint Coordinate System
POSE     pos_now;		// XYZ Coordinate System
PULSE    pls_now;		// Pulse Coordinate system

unsigned long  counter = 0;
int loop = 1;		// 1. Loop, 0. No Loop
int disp = 1;		// 1. display, 0. No display
int retry;
fd_set   SockSet;    // Socket group used with select
timeval  sTimeOut;   // For timeout setting


/*******************************************************************************
					 Robot互動相關
*******************************************************************************/
void RobotConnect()		// 連接機器手臂的程式
{
	// Windows Socket DLL initialization 
	status = WSAStartup(MAKEWORD(1, 1), &Data);
	if (status != 0)
		cerr << "ERROR: WSAStartup unsuccessful" << endl;

	// IP address, port, etc., setting
	memset(&destSockAddr, 0, sizeof(destSockAddr));
	destAddr = inet_addr("140.113.149.10");			//	輸入ROBOT CONTROLLER IP 位址
	memcpy(&destSockAddr.sin_addr, &destAddr, sizeof(destAddr));
	destSockAddr.sin_port = htons(10000);				// 輸入 port
	destSockAddr.sin_family = AF_INET;

	// Socket creation
	destSocket = socket(AF_INET, SOCK_DGRAM, 0);
	if (destSocket == INVALID_SOCKET)
	{
		cerr << "ERROR: socket unsuccessful" << endl;
		status = WSACleanup();
		if (status == SOCKET_ERROR)
			cerr << "ERROR: WSACIeanup unsuccessful" << endl;
	}
	// memset用來對一段內存空間全部設置為某個字符，一般用在對定義的字符串進行初始化
	memset(&MXTsend, 0, sizeof(MXTsend));
	memset(&jnt_now, 0, sizeof(JOINT));
	memset(&pos_now, 0, sizeof(POSE));
	memset(&pls_now, 0, sizeof(PULSE));
}

/*---------------------------------------------------------------------------------*/
void RobotDataRead()		// 讀寫機器手臂資料
{
	memset(&MXTsend, 0, sizeof(MXTsend));
	memset(&MXTrecv, 0, sizeof(MXTrecv));

	// ******************** Send: send 'CommandAngle' ******************** // 
	// Transmission data creation 
	if (loop == 1)
	{  // Only first time  //
		MXTsend.Command = MXT_CMD_NULL;
		MXTsend.SendType = MXT_TYP_NULL;
		MXTsend.RecvType = type;
		MXTsend.SendIOType = MXT_IO_NULL;
		MXTsend.RecvIOType = IOSendType;
		MXTsend.CCount = counter = 0;
	}
	else
	{ // Second and following times
		MXTsend.Command = MXT_CMD_MOVE;
		MXTsend.SendType = 2; 	// When transmitting from the persona; computer to the controller, designate the type of position data transmitted from the personal. 
								// 0 no data, 1 XYZ data, 2 Joint data, 3 Motor pulse data

		MXTsend.RecvType = 2;	// When receiving (monitoring) from the controller to the personal computer, indicate the type pf position data replied from the controller.
								// 0 no data, 1 XYZ data, 2 Joint data, 3 Motor pulse data
								// 4 XYZ data(position after filter process) 5 Joint data (Position after filter process) 6 Motor pulse data (Position after filter process)
								// 7 XYZ data(Encoder feedback value) 8 Joint data (Encoder feedback value) 9 Motor pulse data (Encoder feedback value)
								// 10 Current command[%] 11 Current feedback[%]

								/*------------------------------------------------------------------------------------*/
		memcpy(&MXTsend.dat.jnt, &jnt_now, sizeof(JOINT));			//	jnt_now 寫進 MXTsend.dat.jnt

																	// * RV-2A command use 'radian'!
		MXTsend.dat.jnt.j1 = (float)(commandAngle[0] * PI / 180.0);
		MXTsend.dat.jnt.j2 = (float)(commandAngle[1] * PI / 180.0);
		MXTsend.dat.jnt.j3 = (float)(commandAngle[2] * PI / 180.0);
		MXTsend.dat.jnt.j4 = (float)(commandAngle[3] * PI / 180.0);
		MXTsend.dat.jnt.j5 = (float)(commandAngle[4] * PI / 180.0);
		MXTsend.dat.jnt.j6 = (float)(commandAngle[5] * PI / 180.0);

		/*------------------------------------------------------------------------------------*/
		MXTsend.SendIOType = IOSendType;
		MXTsend.RecvIOType = IORecvType;
		MXTsend.BitTop = 0;		    // 不知道幹麻用 先設為 0  Input head bit No.  (0~32767)					
		MXTsend.BitMask = 0xffff;	// 不知道幹麻用 先設為 0	nput bit mask pattern for output as hexadecimal  (0000~FFFF)
		MXTsend.loData = 0;         // 不知道幹麻用 先設為 0 	Input bit data for output as hexadecimal  (0000~FFFF)
		MXTsend.CCount = counter;
	}	//	end else
		/*------------------------------------------------------------------------------------*/
		/*if(endGrab)
		{
		MXTsend.Command = MXT_CMD_END;
		}*/

		/*...detla++ */

	memset(sendText, 0, MAXBUFLEN);
	memcpy(sendText, &MXTsend, sizeof(MXTsend));
	if (disp) {
		sprintf_s(buf, "Send   (%ld):", counter);
		cout << buf << endl;
	}

	numsnt = sendto(destSocket, sendText, sizeof(MXTCMD), NO_FLAGS_SET, (LPSOCKADDR)&destSockAddr, sizeof(destSockAddr));
	if (numsnt != sizeof(MXTCMD))
	{
		cerr << "ERROR: sendto unsuccessful" << endl;
		status = closesocket(destSocket);
		if (status == SOCKET_ERROR)
			cerr << "ERROR: closesocket unsuccessful" << endl;
		status = WSACleanup();
		if (status == SOCKET_ERROR)
			cerr << "ERROR: WSACIeanup unsuccessful" << endl;
	}	// end if

		// ******************** Receive: get 'actualAngle' ******************** // 
	memset(recvText, 0, MAXBUFLEN);

	retry = 1;                                    // No. of reception retries
	while (retry)  // ??????????????????????????????
	{
		FD_ZERO(&SockSet);                        // SockSet initialization
		FD_SET(destSocket, &SockSet);             // Socket registration
		sTimeOut.tv_sec = 1;                      // Transmission timeout setting (sec)
		sTimeOut.tv_usec = 0;                     //                             (myu sec)
		status = select(0, &SockSet, (fd_set *)NULL, (fd_set *)NULL, &sTimeOut);

		// If it receives by the time-out 
		if ((status > 0) && (FD_ISSET(destSocket, &SockSet) != 0))
		{
			numrcv = recvfrom(destSocket, recvText, MAXBUFLEN, NO_FLAGS_SET, NULL, NULL);
			if (numrcv == SOCKET_ERROR)
			{
				cerr << "ERROR: recvfrom unsuccessful" << endl;
				status = closesocket(destSocket);
				if (status == SOCKET_ERROR)
					cerr << "ERROR: closesocket unsuccessful" << endl;
				status = WSACleanup();
				if (status == SOCKET_ERROR)
					cerr << "ERROR: WSACIeanup unsuccessful" << endl;
			}

			memcpy(&MXTrecv, recvText, sizeof(MXTrecv));
			char   str[10];
			if (MXTrecv.SendIOType == MXT_IO_IN)
				sprintf_s(str, "IN%04x", MXTrecv.loData);
			else if (MXTrecv.SendIOType == MXT_IO_OUT)
				sprintf_s(str, "OT%04x", MXTrecv.loData);
			else
				sprintf_s(str, "------");

			int    DispType;
			void   *DispData;		// 顯示的 Data

			DispType = MXTrecv.SendType;
			DispData = &MXTrecv.dat;

			/*------------------------------------------------------*/
			if (loop == 1)
			{
				memcpy(&jnt_now, DispData, sizeof(JOINT));
				loop = 2;  ///這三小0.0??
			}
			JOINT  *j = (JOINT*)DispData;

			/*------------------------------------------------------*/
			// 將真實機器手臂的角度回傳到虛擬機器手臂上

			actualAngle[0] = (j->j1)*180.0 / PI;
			actualAngle[1] = (j->j2)*180.0 / PI;
			actualAngle[2] = (j->j3)*180.0 / PI;
			actualAngle[3] = (j->j4)*180.0 / PI;
			actualAngle[4] = (j->j5)*180.0 / PI;
			actualAngle[5] = (j->j6)*180.0 / PI;

			sprintf_s(buf, "Receive (%ld): TCount=%d Type(JOINT)=%d\n %7.2f,%7.2f,%7.2f,%7.2f,%7.2f,%7.2f,%7.2f,%7.2f(%s)", MXTrecv.CCount, MXTrecv.TCount, DispType, j->j1, j->j2, j->j3, j->j4, j->j5, j->j6, j->j7, j->j8, str);
			cout << buf << endl;

			printf("Update:  targetAngle[0~6]=(%f, %f, %f, %f, %f, %f)\n", targetAngle[0], targetAngle[1], targetAngle[2], targetAngle[3], targetAngle[4], targetAngle[5]);
			printf("Update:  actualAngle[0~6]=(%f, %f, %f, %f, %f, %f)\n", actualAngle[0], actualAngle[1], actualAngle[2], actualAngle[3], actualAngle[4], actualAngle[5]);
			printf("Update:  commandAngle[0~6]=(%f, %f, %f, %f, %f, %f)\n", commandAngle[0], commandAngle[1], commandAngle[2], commandAngle[3], commandAngle[4], commandAngle[5]);

			counter++;              // Count up only when communication is successful
			retry = 0;                // Leave reception loop

		}	//	end if 
		else
		{ // Reception timeout
			cout << "... Receive Timeout!  <Push [Enter] to stop the program>" << endl;
			retry--;                 // No. of retries subtraction
			if (retry == 0)
				loop = 0;  // End program if No. of retries is 0
		}	// end else
	} /* while(retry) */

}
/*---------------------------------------------------------------------------------*/

void CloseRobot()
{
	// Close socket 
	status = closesocket(destSocket);			// 中斷連線時機器手臂夾關閉????
	if (status == SOCKET_ERROR)
		cerr << "ERROR: closesocket unsuccessful" << endl;
	status = WSACleanup();
	if (status == SOCKET_ERROR)
		cerr << "ERROR: WSACIeanup unsuccessful" << endl;
}
/*---------------------------------------------------------------------------------*/

/*******************************************************************************
									執行緒
*******************************************************************************/
//void Send(int grip); ///佳琪's

DWORD WINAPI threadFunction1(LPVOID lpParameter) // 141Hz for dealing real robot
{
	RobotConnect();  //here or Main?
	while (loop) {

		//if (status != SOCKET_ERROR)
		RobotDataRead();

		for (int i = 0; i<6; i++) {

			double delta = targetAngle[i] - actualAngle[i];
			if (fabs(delta)>0.5)  //差值>0.2
				commandAngle[i] = actualAngle[i] + delta / fabs(delta) * 0.5;
			else if (fabs(delta)>0.2)  //0.5>差值>0.2
				commandAngle[i] = targetAngle[i];
			else  //差小於一個值 => 忽略 (以免機械手微調造成異常
				commandAngle[i] = actualAngle[i];
		}

	}
	CloseRobot();  //here or Main?

	cout << "end thread1" << endl;
	return 0;
}


int main(int argc, char *argv[])
{
	HANDLE thread1;//, thread2, hMutex;
	thread1 = CreateThread(NULL, 0, threadFunction1, NULL, 0, NULL);
	WaitForSingleObject(thread1, INFINITE);



	CloseHandle(thread1);
	return 0;
}
