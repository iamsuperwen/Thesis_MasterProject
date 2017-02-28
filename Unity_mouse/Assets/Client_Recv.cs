using System;
using System.Net;
using System.Net.Sockets;
//using System.Text;
//using System.Threading;
using System.IO;

public class Client_Recv
{
	public TcpClient tc = null;
	public bool co_ready = false;
	public byte[] data;
	public string recvMsg;

	public void Run ()
	{
		this.tc = new TcpClient ();  
		this.tc.Connect ("127.0.0.1", 5566);  //if use "try-catch": 等待連線，若未連線則會停在這行?
		data = new byte[this.tc.ReceiveBufferSize];  
		this.tc.GetStream ().BeginRead (data, 0, System.Convert.ToInt32 (this.tc.ReceiveBufferSize), ReceiveMessage, null);
		co_ready = true;
		recvMsg = null; //!?

	}		
	public void ReceiveMessage (IAsyncResult ar)
	{  
		try 
		{
			int bytesRead;
			bytesRead = this.tc.GetStream ().EndRead (ar);

			if (bytesRead < 1) 
			{  
				return;  
			} 
			else 
			{  
				recvMsg = System.Text.Encoding.ASCII.GetString (data, 0, bytesRead);
			}  
			this.tc.GetStream ().BeginRead (data, 0, System.Convert.ToInt32 (this.tc.ReceiveBufferSize), ReceiveMessage, null); 
		} 
		catch (Exception) 
		{  

		}

	}
}