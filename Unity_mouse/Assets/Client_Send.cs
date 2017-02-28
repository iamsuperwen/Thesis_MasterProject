using System;
using System.Net;
using System.Net.Sockets;
//using System.Text;
//using System.Threading;
using System.IO;

public class Client_Send
{
	TcpClient tc = null;
	NetworkStream ns = null;
	StreamWriter sw = null;

	public void Run ()
	{
		this.tc = new TcpClient ();  
		this.tc.Connect ("127.0.0.1", 5566);  //if use "try-catch": 等待連線，若未連線則會停在這行?

		ns = tc.GetStream ();
		sw = new StreamWriter (ns);
	}		
	public void Send (string sendMsg)  //傳送訊息
	{	
		if (sendMsg.Length < 50)  //7*6+5+10=51 (6 joints * '(-)0.0000', round off to the 4th decimal + ',-999.9999' )
			throw new NullReferenceException("ERROR: 'targetAngle[1~6]' send to PC go wrong!");
		else if (ns != null) {
			//sw = new StreamWriter (ns);
			sw.Write (sendMsg);
			sw.Flush ();
		}
	}
}