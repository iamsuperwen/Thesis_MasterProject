using UnityEngine;
using System.Collections;
using System.Net.Sockets;
using System;

public class Client : MonoBehaviour
{
	//*** 宣告變數: 跟連線有關的變數, Unity為Client
	private ClientThread ct;
	private bool isSend;

	//*** 宣告變數: 為了使用jointsRotate.cs中的變數
	GameObject Robot;
	jointsRotate Robot_jR;

	//*** 連線時,機械手臂實際姿態(角度)
	GameObject joint1_a, joint2_a, joint3_a, joint4_a, joint5_a, joint6_a, endEff_a;

	///initial(home) theta = { 0, 90, 0, 0, 0, 0 }
	//public ???
	//float[] theta_tar = new float[] { 123, 44, 67, 89, -32, 1 };  //VR robot的 target theta
	float[] theta_tar_a = new float[] { 0, 90, 0, 0, 0, 0 };  //VR robot的 target theta(actual) 讀取RV-2A實際角度 
	float[] theta_now_a = new float[] { 0, 90, 0, 0, 0, 0 };  //VR robot的 present theta(now) 虛擬手臂旋轉用
	float[] dtheta_a = new float[] { 0, 0, 0, 0, 0, 0 };	  //delta theta(diff) VR robot專用!!(跟jointsRotate不同喔!)
	float[] theta_0 = new float[] { 0, 90, 0, 0, 0, 0 };      //VR robot的 initial theta(home)  //!!!??
	float[] theta_user = new float[] { 0, 0, 0, 0, 0, 0 };    //theta_tar -> targetAngle(PC) 的相對角度(中繼站)

	void Awake()
	{
		Robot = GameObject.Find("RV_2A");
		Robot_jR = Robot.GetComponent<jointsRotate>();
		//*** Robot_jR.xxx : 即可取得jointsRotate.cs中的變數 xxx
		// Ex. Robot_jR.theta_tar

		joint1_a = GameObject.Find("Joint1_actual");
		joint2_a = GameObject.Find("Joint2_actual");
		joint3_a = GameObject.Find("Joint3_actual");
		joint4_a = GameObject.Find("Joint4_actual");
		joint5_a = GameObject.Find("Joint5_actual");
		joint6_a = GameObject.Find("Joint6_actual");
		endEff_a = GameObject.Find("end_effector_actual");

	}

	private void Start()
	{
		/// 之後整合要移走: ON/OFF - line +開關條件
		ct = new ClientThread(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp, "127.0.0.1", 5566);  //localhost
		ct.StartConnect();
		isSend = true;

	}

	private void Update()
	{
		if (ct.receiveMessage != null)  //***** Update 'actual angle'  /// W/out protect yet!
		{
			string[] recv_msg = ct.receiveMessage.Split (',');  //用逗號分割字串; using System;
			//string[] recv_msg = ct.receiveMessage.Split (new string[] { "," }, StringSplitOptions.RemoveEmptyEntries);  //用逗號分割字串; using System;
			Debug.Log ("ct.receiveMessage:" + ct.receiveMessage);
			if (recv_msg.Length > 6) {
				for (int i = 0; i < 6; i++)
					theta_tar_a [i] = -float.Parse (recv_msg [i]) + theta_0[i];  //Convert: string -> float
			}
			else {
				//Debug.Log ("ct.receiveMessage ERROR: " + ct.receiveMessage);
				Debug.Log ("ct.receiveMessage ERROR: recv_msg.Length = " + recv_msg.Length);
			}
			ct.receiveMessage = null;

			Debug.Log ("recv_msg : " + recv_msg[0] + " // ");
		}

		if (isSend == true) {    //***** Send 'target angle' to PC
			for (int i = 0; i < 6; i++)
				theta_user [i] = -(Robot_jR.theta_tar [i] - theta_0 [i]);

/*
	targetAngle[0] = 0;    //range: ( -159.93 ~ +159.97 )  ; bound: +- 150 deg
	targetAngle[1] = 0;    //range: ( -45     ~ +93.71  )  ; bound: +- 40  deg
	targetAngle[2] = 90;   //range: ( +50.09  ~ +169.89 )  ; bound: +- 35  deg
	targetAngle[3] = -25;  //range: ( -159.87 ~ +138.11 )  ; bound: +- 130 deg
	targetAngle[4] = 0;    //range: ( -119.97 ~ +119.9  )  ; bound: +- 115 deg
	targetAngle[5] = 0;    //range: ( -199.88 ~ +200    )  ; bound: +- 190 deg
*/

			SendTargetAngle ();
		}

		ct.Receive();


		// -------------------------------------------------- //
		for (int i = 0; i < 6; i++)
			dtheta_a [i] = theta_tar_a [i] - theta_now_a [i];

		joint1_a.transform.RotateAround (Vector3.zero, Vector3.up, dtheta_a [0]);
		joint2_a.transform.Rotate (Vector3.up, dtheta_a [1]);
		joint3_a.transform.Rotate (Vector3.up, dtheta_a [2]);
		joint4_a.transform.Rotate (Vector3.up, dtheta_a [3]);
		joint5_a.transform.Rotate (Vector3.up, dtheta_a [4]);
		joint6_a.transform.Rotate (Vector3.up, dtheta_a [5]);

		for (int i = 0; i < 6; i++)		//refresh: after rotate
			theta_now_a [i] = theta_tar_a [i];



		Debug.Log ("~ theta_now_a = ( " + theta_now_a[0] + ", " + theta_now_a[1] + ", " + theta_now_a[2] + ", " + theta_now_a[3] + ", " + theta_now_a[4] + ", " + theta_now_a[5]+ ")\n");
		Debug.Log ("~ theta_tar_a = ( " + theta_tar_a[0] + ", " + theta_tar_a[1] + ", " + theta_tar_a[2] + ", " + theta_tar_a[3] + ", " + theta_tar_a[4] + ", " + theta_tar_a[5]+ ")\n");
		Debug.Log ("~~ theta_tar = ( " + Robot_jR.theta_tar[0] + ", " + Robot_jR.theta_tar[1] + ", " + Robot_jR.theta_tar[2] + ", " + Robot_jR.theta_tar[3] + ", " + Robot_jR.theta_tar[4] + ", " + Robot_jR.theta_tar[5]+ ")\n");
		Debug.Log ("~ theta_user = ( " + theta_user[0] + ", " + theta_user[1] + ", " + theta_user[2] + ", " + theta_user[3] + ", " + theta_user[4] + ", " + theta_user[5]+ ")\n\n");
	}

	void SendTargetAngle()  //***** Send 'target angle' to PC:  Function ~
	{
		isSend = false;
		/// yield return new WaitForSeconds(1);
		string send_msg = theta_user[0].ToString("f4") + "," + theta_user[1].ToString("f4") + "," + theta_user[2].ToString("f4") + "," + theta_user[3].ToString("f4") + ","+ theta_user[4].ToString("f4") + "," + theta_user[5].ToString("f4") + ",999.9999";
		ct.Send(send_msg);
		isSend = true;
	}

	private void OnApplicationQuit() //Sent to all game objects before the application is quit.
	{
		ct.StopConnect();  //Close Socket
	}
}