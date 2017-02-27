using UnityEngine;
using System.Collections;
using System.Net.Sockets;
using System;

public class Client : MonoBehaviour
{
	private ClientThread ct;
	private bool isSend;

	///initial(home) theta = { 0, 90, 0, 0, 0, 0 }
	//public ???
	float[] theta_tar = new float[] { 123, 44, 67, 89, -32, 1 };  //VR robot的 target theta
	float[] theta_now = new float[] { 0, 90, 0, 0, 0, 0 };  //VR robot的 present theta(now)
	float[] theta_0 = new float[] { 0, 90, 0, 0, 0, 0 };  //VR robot的 initial theta(home)
	float[] theta_user = new float[] { 0, 0, 0, 0, 0, 0 };  //theta_tar -> targetAngle(PC) 的相對角度(中繼站)

	private void Start()
	{
		/// 之後整合要移走: ON/OFF - line
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
					theta_now [i] = -float.Parse (recv_msg [i]) + theta_0[i];  //Convert: string -> float
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
				theta_user [i] = -(theta_tar [i] - theta_0 [i]);

			SendTargetAngle ();
		}

		ct.Receive();

		Debug.Log ("~ theta_now = ( " + theta_now[0] + ", " + theta_now[1] + ", " + theta_now[2] + ", " + theta_now[3] + ", " + theta_now[4] + ", " + theta_now[5]+ ")\n");
		Debug.Log ("~ theta_tar = ( " + theta_tar[0] + ", " + theta_tar[1] + ", " + theta_tar[2] + ", " + theta_tar[3] + ", " + theta_tar[4] + ", " + theta_tar[5]+ ")\n");
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