/*****************************************************************************
Title: PC_TCP
Features: TCP commnication between PC(Cpp) <-> RV-2A
Author: Tsai,Chia-Wen
Date: 2017/02/20
Last update: 2017/02/20
Status: CAN'T COMPILE!!!
Notes: 
  - Revised from "機器人改5/HelloSphere.cpp"
******************************************************************************/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <winsock.h>
#include <conio.h>
#include <string.h>
#include <time.h> ///改5
#include <cmath>  ///改5
#include <iomanip.h>  ///改5
#include <math.h>
#include <assert.h>  ///改5
#include "strdef.h"  //define RV-2A variable structure
#include "dos.h"   ///改5
#include<fstream.h>  //記事本 ///改5
using std::getline;  ///改5
using std::string;  ///改5

#include "stdafx.h" ///
using namespace std; ///
#include <string> ///
#include <cstdio> ///


////////////////////////////////////////////////////////////////////////////////////////
// Control mode
#define MODE_RESET		6
#define MODE_KEYBOARD	5  //
#define MODE_ROTATE		2

#define PI 3.14159265359

int flag_delta[6]; 

struct NODE_DATA
{
	int id;
	double position_m[3];	// w.r.t master frame
	double position_s[3];	// w.r.t slave frame
	//double position_people[6][3];
//	double position_people[4][8][3];  // for person's position
	double position_cam[4][3];

	double position_tar[4][3];
};

struct NODE			 // declaration of a new structure
{         
	NODE* next;      // The next node in the list
	NODE* prev;      // The previous node in the list
	NODE_DATA data;
};

class LIST
{                        //declaration of a new class
	public:
		int node_num;		// Number of nodes
		NODE* FirstNode;	// The first node in the list
		NODE* LastNode;		// The last node in the list
		NODE* ptr;		     // pointer

		LIST()//建立者函數
		{	
			node_num = 0;
			FirstNode = NULL;  //This initialises 'FirstNode' to NULL
			LastNode = NULL;   //This initialises 'LastNode' to NULL
			ptr = NULL;   //This initialises 'LastNode' to NULL
		}

		NODE* add_node(NODE_DATA data)  //Add a new node
	
		{   

			NODE* current;       //狀態:包含現在的點資料  還有之前跟下一個
			if ( FirstNode == NULL ) 
			{
				current = new NODE;
				FirstNode = current;
				LastNode = current;
				current->next = NULL;
				current->prev = NULL;
				current->data = data;
				current->data.id = 1;
				node_num = current->data.id;
				return current;
			} 
			if ( FirstNode != NULL ) 
			{
				current = new NODE;
				current->next = NULL;

				LastNode->next = current;
				current->prev = LastNode;
				LastNode = current;
				
				current->data = data;
				current->data.id = current->prev->data.id + 1;
				node_num = current->data.id;
				return current;
			}
			return NULL;		
		}

};
LIST end_point;//用建立者函數宣告一個LIST類別
LIST point_tar;
LIST point_obs;

/***** for 執行緒 *****/
HANDLE thread1;

/*HANDLE thread5, thread99;  //for 平板
//HANDLE hMutex;  //for 平板
//SOCKET m_socket;  //for 平板: 平板連線socket: threadFunction5
//char recvbuf[200];  //for 平板: threadFunction99
*/

/************** 記錄系統時間 ***************/
static LARGE_INTEGER systemStartTime;
static LARGE_INTEGER systemNowTime;  ///in "void initRECORD()" <-好像不能刪..
double systemPassTime;  ///跟systemNowTime有關


// estimated matrix A for old force sensor 
double matrixA[3][3]={{1019.7 , 216.14 , 80.81},{-187.43 , 883.49 , -21.649},{-3.5994 , -220.66 , 988.23}};
//本來是HLdouble
double Transform_keyboard[16] = {1,0,0,0, 0,-1,0,0, 0,0,-1,0, 350,0,100,1};//校正用位置
//bool xyzMode = 1;  //for 平板 thread99 收data


////////////// 鍵盤控制 (for IK) //////////////
//////////////相對位置控制的變數(paper的方法)////////////////

//thread freq
/*double freq_vision, freq_force, freq_robot;
double time_vision, time_force, time_robot;
long unsigned int frame_vision=0, frame_force=0, frame_robot=0; */

//robot angle
double theta[6] = {0}; //IK function用 (算出來並不一定在合理範圍)
double xyz_angel[3]={-PI/2, -PI, PI/2};
double actualAngle[6] = {0, -PI/2, 0, 0, 0, 0};		// robot rendering
double commandAngle[6] = {0, -PI/2, 0, 0, 0, 0};	// processed by desiredAngle and then send to robot
double desiredAngle[6] = {0, 0, 0, 0, 0, 0};		// derived from Phantom

short int CtrlMode = MODE_KEYBOARD;
bool showRobot = 1;
//int clawGrab = 0;  //for 平板

//bool flag_robot_go = 0;		// 0: stop, 1: go
/// 平板! 平板!
bool endGrab = 0; 
bool resetMode = 0; //自動將角度調回初始狀態
int isPress[9] = {0}; //判斷鍵盤是否按下
int tablet = 0;    //判斷平板是否連線

static LARGE_INTEGER TimerFrequency;

double d0 = 80;
double d1 = 350;
double a1 = 100;
double a2 = 250;
double a3 = 130;
double d3 = 0;
double d4 = 250;

double d6 = 117;	// 117mm center of force sensor
					//210+57(force sensor) //257->抓住試管處
					//301->六角扳手控制點(前方球心)

double d7;    		//600畫準線用
					//184 -> force sensor中心 到六角扳手控制點
					//22.75-> force sensor中心 到小梅花扳手握柄之長度
					//22.95-> force sensor中心 到大梅花扳手握柄之長度
double a7;  		//110->小梅花扳手之力臂
					//134->大梅花扳手之力臂


#define TOOL_CLAW		        3
int tool=TOOL_CLAW;  //工具為夾爪


/********************  紀錄路徑相關  ********************/
/*struct listNode 
{            
   //double velocity[3];
   double noa[3][3];
   double position[3];
   double q[6];// 機器人6軸
   double sysTime;
   int order; //標示此link排在第幾個
   int all_order; //表示本來是第幾點
   int grip;

   struct listNode *nextPtr; //* pointer to next node/ 
   struct listNode *prePtr;	 //* pointer to previous node/ 
};
typedef struct listNode ListNode; //* synonym for struct listNode /
typedef ListNode *ListNodePtr; //* synonym for ListNode* /
int isRecordTrajectory = -1; //是否開始記錄軌跡
bool alreadyRecordTrajectory = false; //是否記錄過軌跡
int save = 0;     //確認是否是存點的flag   
int program = 0;  //紀錄現在有幾個存成檔案的程式
int load_program = -1; //紀錄是否要讀取程式
ListNodePtr startPtr = NULL; //* initially there are no nodes /
ListNodePtr currentPtr;  //紀錄目前路徑記錄到哪一點
ListNodePtr allstart = NULL;
ListNodePtr allcurrent = NULL;

ListNodePtr trajCurPtr; //用來記錄目前phantom最接近哪一點
ListNodePtr recordNearestPtr; //用在搜尋"所有"list點中最近的點的時候，紀錄離游標最近的點
int nearestPt4Trajectory = 0; //路徑中離游標最近的點的order
unsigned int rank = 0; //標示此link排在list中的第幾個
bool startTrajectoryForce = 0;
bool isTouchTraj = 0;
/////// For time trajectory ///////
ListNodePtr startTimePtr = NULL;

bool replay = 0, semi_auto = 0;
bool flag_preReplay = 0; //判斷前一個mode是否為replay (在time space之間切換時用的flag)
bool replay_reverse = 0; //控制replay時是否逆向

//****************** Guide Line 相關 ********************
bool guideLine = 0; //是否啟動guideline導引
Vertex startPoint, endPoint; //直線導引的起點與終點
//double guideLineSamplingPt[][];
ListNodePtr startGlPtr, currentGlPtr;//
int rank4GL;//紀錄目前為第幾個node
bool alreadyDefineGL = 0; //用來確定guiding line是否已經產生 產生後才開啟力導引

//********************** 紀錄文件 **********************
FILE *fptr; //機器人和游標的路徑
FILE *tptr; //軌跡 
FILE *eptr; //紀錄末端位置
bool startRecord = 0;// 是否將資料寫入記事本
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// real robot
#define NO_FLAGS_SET 0 
#define MAXBUFLEN 512
#pragma comment(lib, "wsock32.lib")

/*********************真實&虛擬機器手臂&座標轉換******************************/
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
unsigned short IOSendType = MXT_IO_OUT;  // Send input/output signal data designation	//	output signal
unsigned short IORecvType = MXT_IO_OUT;  // Reply input/output signal data designation  //	output signal

MXTCMD   MXTsend;		// 送到機器手臂的資料
MXTCMD   MXTrecv;		// 回傳到personal computer的資料
JOINT    jnt_now;		// Joint Coordinate System
POSE     pos_now;		// XYZ Coordinate System
PULSE    pls_now;		// Pulse Coordinate system

unsigned long  counter = 0;
int loop = 1;		// 1. Loop, 0. No Loop
int disp = 0;		// 1. display, 0. No display
int ch;
long ratio=1;		// 移動角度的比例，先設成1
int retry;
fd_set     SockSet;                      // Socket group used with select
timeval    sTimeOut;                     // For timeout setting

bool isConnectRobot = 0; //是否連接到真實手臂


//*************************************************************************//
//        設定要用哪個tool (1:六角(EXP1) 2:六角(EXP2) 3:大梅花)
//*************************************************************************//
void set_A7(int in) 
{
	if(tool==TOOL_CLAW) //為夾爪 (其它我刪了..20170218
	{		
		if ( in == 0 ) 
		{
			a7 = 0;
			d7=158;
		}
		double A7_temp[4][4] = {{1,  0,  0, a7} ,
								{0,  1,  0,  0} , 
								{0,  0,  1, d7}, 
								{0,  0,  0,  1}   };

		for(int i=0; i<4; i++)
			for(int j=0; j<4; j++) 
				A7[i][j] = A7_temp[i][j] ;
	}
	invTrans(A7 , A7_inverse);
}


/*******************************************************************************
								Robot互動相關
*******************************************************************************/

// 連接機器手臂的程式
void RobotConnect()
{
	// Windows Socket DLL initialization 
	status=WSAStartup(MAKEWORD(1, 1), &Data); 
	if (status != 0)
	cerr << "ERROR: WSAStartup unsuccessful"<< endl;

	// IP address, port, etc., setting
	memset(&destSockAddr, 0, sizeof(destSockAddr));
	destAddr=inet_addr("140.113.149.10");			//	輸入ROBOT CONTROLLER IP 位址
	memcpy(&destSockAddr.sin_addr, &destAddr, sizeof(destAddr));
	destSockAddr.sin_port=htons(10000);				// 輸入 port
	destSockAddr.sin_family=AF_INET;

	// Socket creation
	destSocket=socket(AF_INET, SOCK_DGRAM, 0);
	if (destSocket == INVALID_SOCKET) 
	{
		cerr << "ERROR: socket unsuccessful"   << endl;
		status=WSACleanup();
		if (status == SOCKET_ERROR)
			cerr << "ERROR: WSACIeanup unsuccessful"   << endl;
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

	// Transmission data creation 
	if(loop==1) 
	{  // Only first time
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

		MXTsend.dat.jnt.j1 += (float)(commandAngle[0]*ratio); 
		MXTsend.dat.jnt.j2 += (float)((commandAngle[1]+PI/2)*ratio); 
		MXTsend.dat.jnt.j3 += (float)(commandAngle[2]*ratio); 
		MXTsend.dat.jnt.j4 += (float)(commandAngle[3]*ratio); 
		MXTsend.dat.jnt.j5 += (float)(commandAngle[4]*ratio); 
		//MXTsend.dat.jnt.j5 += (float)(0.018*PI/180.0); 
		MXTsend.dat.jnt.j6 += (float)(commandAngle[5]*ratio);
		
/*------------------------------------------------------------------------------------*/
		MXTsend.SendIOType = IOSendType;
		MXTsend.RecvIOType = IORecvType;
		MXTsend.BitTop = 0;		//	不知道幹麻用 先設為 0  Input head bit No.  (0~32767)					
		MXTsend.BitMask = 0;	// 不知道幹麻用 先設為 0	nput bit mask pattern for output as hexadecimal  (0000~FFFF)
		MXTsend.loData = 0;     // 不知道幹麻用 先設為 0 	Input bit data for output as hexadecimal  (0000~FFFF)
		MXTsend.CCount = counter; 
	}	//	end else
/*------------------------------------------------------------------------------------*/
	if(endGrab)
	{
		MXTsend.Command = MXT_CMD_END;
	}
	memset(sendText, 0, MAXBUFLEN); 
	memcpy(sendText, &MXTsend, sizeof(MXTsend)); 
	if(disp) 
	{
		sprintf(buf, "Send   (%ld):",counter);
		cout << buf << endl;
	}

	numsnt=sendto(destSocket, sendText, sizeof(MXTCMD), NO_FLAGS_SET, (LPSOCKADDR) &destSockAddr, sizeof(destSockAddr)); 
	if (numsnt != sizeof(MXTCMD)) 
	{
		cerr << "ERROR: sendto unsuccessful" << endl;
		status=closesocket(destSocket);
		if (status == SOCKET_ERROR)		
			cerr << "ERROR: closesocket unsuccessful" << endl; 
		status=WSACleanup(); 
		if (status == SOCKET_ERROR)		
			cerr << "ERROR: WSACIeanup unsuccessful" << endl; 
	}	// end if

	memset(recvText, 0, MAXBUFLEN);

	retry = 1;                                    // No. of reception retries
	while(retry)	// ??????????????????????????????
	{
		FD_ZERO(&SockSet);                        // SockSet initialization
		FD_SET(destSocket, &SockSet);             // Socket registration
		sTimeOut.tv_sec = 1;                      // Transmission timeout setting (sec)
		sTimeOut.tv_usec = 0;                     //                             (myu sec)
		status = select(0,  &SockSet, (fd_set *)NULL, (fd_set *)NULL, &sTimeOut); 

		if((status > 0) && (FD_ISSET(destSocket, &SockSet) != 0)) 
		{
			numrcv=recvfrom(destSocket, recvText, MAXBUFLEN, NO_FLAGS_SET, NULL, NULL); 
			if (numrcv == SOCKET_ERROR) 
			{
				cerr << "ERROR: recvfrom unsuccessful" << endl;
				status=closesocket(destSocket);
				if (status == SOCKET_ERROR)
					cerr << "ERROR: closesocket unsuccessful" << endl; 
				status=WSACleanup(); 
				if (status == SOCKET_ERROR)
					cerr << "ERROR: WSACIeanup unsuccessful" << endl; 
			}	// end if

			memcpy(&MXTrecv, recvText, sizeof(MXTrecv));
			char   str[10];
			if (MXTrecv.SendIOType==MXT_IO_IN)
				sprintf(str,"IN%04x", MXTrecv.loData); 
			else if (MXTrecv.SendIOType==MXT_IO_OUT)
				sprintf(str,"OT%04x", MXTrecv.loData); 
			else    
				sprintf(str,"------");

			int    DispType; 
			void   *DispData;		// 顯示的 Data

			DispType = MXTrecv.SendType;
			DispData = &MXTrecv.dat; 
///#endif
/*------------------------------------------------------*/				
			if(loop==1)
			{
				memcpy(&jnt_now, DispData, sizeof(JOINT)); 
				loop = 2; 
			} 
			JOINT  *j=(JOINT*)DispData;
//			sprintf(buf, "Receive (%ld): TCount=%d Type(JOINT)=%d\n %7.2f,%7.2f,%7.2f,%7.2f,%7.2f,%7.2f,%7.2f,%7.2f(%s)" ,MXTrecv.CCount,MXTrecv.TCount,DispType ,j->j1, j->j2, j->j3 ,j->j4, j->j5, j->j6, j->j7, j->j8, str); 
			/*------------------------------------------------------*/
			// 將真實機器手臂的角度回傳到虛擬機器手臂上

			actualAngle[0] = (j->j1);
			actualAngle[1] = (j->j2)-PI/2;
			actualAngle[2] = (j->j3)-PI/2;
			actualAngle[3] = (j->j4);
			actualAngle[4] = (j->j5);
			actualAngle[5] = (j->j6);
		
			counter++;              // Count up only when communication is successful
			retry=0;                // Leave reception loop

		}	//	end if 
		else 
		{ // Reception timeout
			cout << "... Receive Timeout!  <Push [Enter] to stop the program>" << endl;
			retry--;                 // No. of retries subtraction
			if(retry==0)   
				loop=0;  // End program if No. of retries is 0
		}	// end else
	} /* while(retry) */ 

}
/*---------------------------------------------------------------------------------*/

void CloseRobot()
{
	// Close socket 
	status=closesocket(destSocket);			// 中斷連線時機器手臂夾關閉????
	if (status == SOCKET_ERROR)
		cerr << "ERROR: closesocket unsuccessful" << endl; 
	status=WSACleanup(); 
	if (status == SOCKET_ERROR)
		cerr << "ERROR: WSACIeanup unsuccessful" << endl;
}
/*---------------------------------------------------------------------------------*/

/*******************************************************************************
								執行緒 
*******************************************************************************/
void Send(int grip);

DWORD WINAPI threadFunction1(LPVOID lpParameter) // 141Hz for dealing real robot
{
	double base_angle = 0.01; // high speed
	const static double deltaAngle[6] = {base_angle, base_angle, base_angle*1.2, base_angle*1.6, base_angle*1.2, base_angle*2.2};
	while(1)
	{
		//thread頻率  ///這三小0.0?!
		/*if(frame_robot % 500 == 0)
		{
			freq_robot = 500 / (systemPassTime - time_robot);
			time_robot = systemPassTime;
		}
		frame_robot++;*/

		if(isConnectRobot) 
		{
			RobotDataRead();	// read robot joint angles
		}
		else 
		{
			Sleep(6);  // control thread freq ///??
		}

		if(CtrlMode == MODE_ROTATE) ///teaching mode! (joint)
		{
			int check=0;
					
			if((isConnectRobot)&&(isRecordTrajectory >= 0)&&(rank==100)) //真實手臂要執行教導任務
			{
				for(int i=0; i<6; i++)
				{		
					double delta = desiredAngle[i] - actualAngle[i];    //fabs():絕對值
					
					if( fabs(delta) > 0.005)     //差大於一個值 => 慢慢加上
					{
						commandAngle[i] += delta*0.005;
						flag_delta[i] = 1;
					}
					else                        //差小於一個值 => 忽略 (以免機械手微調造成異常
					{
						flag_delta[i] = 0;
						check++;
					}
				}
					
				if(check==6)
				{
					if(isRecordTrajectory == 0)
					{
						CtrlMode = MODE_KEYBOARD;
						isRecordTrajectory = -1;
						Send(100);
					}
					else
						rank = 2;
				}
				else
					Send(-1);
			}
				
			else if((!isConnectRobot)&&(isRecordTrajectory >= 0)&&(rank==100)) //bear 虛擬手臂要執行教導任務
			{
				for(int i=0; i<6; i++)
				{																							
					double delta = desiredAngle[i] - actualAngle[i];
					if( fabs(delta) > 0.0001)
						actualAngle[i] = actualAngle[i] + (delta * 0.05);
					else
					{
						actualAngle[i] = desiredAngle[i];
						check++;
					}
				}
				
				if(check==6)
				{
					if(isRecordTrajectory == 0)
					{
						CtrlMode = MODE_KEYBOARD;
						isRecordTrajectory = -1;
						Send(100);
					}
					else
						rank = 10;
				}
				else
					Send(-1);
			}
		}


		else if(CtrlMode == MODE_KEYBOARD)  //&& !xyzMode //joint ///this!20170217  //!!!
		{
			for(int i=0; i<6; i++)
			{
					if(isConnectRobot)
					{
						if(isPress[i] == 1) commandAngle[i] += 0.002;
						else if(isPress[i] == -1) commandAngle[i] -= 0.002;
					}
					else if(!isConnectRobot)
					{
						if(isPress[i] == 1) actualAngle[i] += 0.005;
						else if(isPress[i] == -1) actualAngle[i] -= 0.005;
					}
			}
			//----------------算角度和同步joint & xyz model
			double temp;
			xyz_angel[2]=atan2(A70[1][2],A70[0][2])+PI;
			temp = A70[0][2]*cos(xyz_angel[2]) + A70[1][2]*sin(xyz_angel[2]);
			xyz_angel[1]=atan2(temp,A70[2][2]);
			xyz_angel[0]=atan2(-A70[0][0]*sin(xyz_angel[2])+A70[1][0]*cos(xyz_angel[2]), -A70[0][1]*sin(xyz_angel[2])+A70[1][1]*cos(xyz_angel[2]));
			for(i=0;i<4;i++)
				for(int j=0;j<4;j++)
					Transform_keyboard[i+j*4] = A70[i][j];   
		//------------------------------------------
		}
		else if(CtrlMode == MODE_RESET && resetMode)
		{
			const static double deltaAngle[6] = {0.005, 0.005, 0.005, 0.005, 0.005, 0.005};//0.015, 0.015, 0.02, 0.02, 0.025, 0.03
			desiredAngle[0] = 0; 
			desiredAngle[1] = -90*PI/180; 
			desiredAngle[2] = 0; 
			desiredAngle[3] = 0; 
			desiredAngle[4] = 0; 
			desiredAngle[5] = 0; 

			double threshold = 0.035 * PI/180;
			for(int i=0; i<6; i++)
				{	
					double delta = desiredAngle[i] - actualAngle[i];
					if( fabs(delta)  > threshold)
					{
						commandAngle[i] += delta/fabs(delta) * deltaAngle[i]* 0.12;
						actualAngle[i] += delta/fabs(delta) * deltaAngle[i] * 0.12;
					}
				}
		}

		DHmodel();

	}// end while(1)

	cout<<"end thread1"<<endl;
	return 0;
}


int main(int argc, char *argv[])
{
	set_A7(0);	// set tool and matrix A7
	RobotConnect();

	QueryPerformanceFrequency(&TimerFrequency);
	QueryPerformanceCounter(&systemStartTime);//開始計時

	thread1 = CreateThread(NULL, 0, threadFunction1, NULL, 0, NULL);
	thread5 = CreateThread(NULL, 0, threadFunction5, NULL, 0, NULL);

	CloseHandle(thread1);

	hMutex = CreateMutex(NULL, FALSE, NULL);

	
	CloseRobot();

    return 0;
}



