using UnityEngine;
using System.Collections;
using System.Net.Sockets;

public class Client : MonoBehaviour
{
	private ClientThread ct;
	private bool isSend;

	///initial(home) theta = { 0, 90, 0, 0, 0, 0 }
	public float[] theta_tar = new float[] { 0, 90, 0, 0, 0, 0 };  //target theta  <- targetAngle(PC) 相對角度(須再處理)
	public float[] theta_now = new float[] { 0, 90, 0, 0, 0, 0 };  //present theta(now)  <- actualAngle(PC) 相對角度(須再處理)
	
	private void Start()
	{
		/// 之後整合要移走: ON/OFF - line
		ct = new ClientThread(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp, "127.0.0.1", 5566);  //localhost
		ct.StartConnect();
		isSend = true;
	}

	private void Update()
	{
		if (ct.receiveMessage != null)  // Update 'actualAngle'  /// W/out protect yet!
		{
			string[] recv_msg;
			recv_msg = client_recv.message.Split (new string[] { "," }, StringSplitOptions.RemoveEmptyEntries);//用逗號分割字串
			if(recv_msg.Length == 6){
				for (int i = 0; i < 6; i++)
					theta_now[i] = - float.Parse(recv_msg[i]);
				Debug.Log("Recieve actualAngle:" + ct.receiveMessage);
			}
			else
				Debug.Log("Recieve ERROR: " + ct.receiveMessage);
			ct.receiveMessage = null;
			theta_now[1] += 90;
		}
		if (isSend == true)
			SendTargetAngle();

		ct.Receive();
	}

	void SendTargetAngle()  // Send 'targetAngle' to PC
	{
		isSend = false;
		/// yield return new WaitForSeconds(1);
		string[] send_msg = (-theta_tar[0]).ToString("f4") + "," + (-theta_tar[1]+90).ToString("f4") + "," + (-theta_tar[2]).ToString("f4") + "," + (-theta_tar[3]).ToString("f4") + ","+ (-theta_tar[4]).ToString("f4") + "," + (-theta_tar[5]).ToString("f4");
		ct.Send(send_msg);
		isSend = true;
	}

	private void OnApplicationQuit() //Sent to all game objects before the application is quit.
	{
		ct.StopConnect();  //Close Socket
	}
}