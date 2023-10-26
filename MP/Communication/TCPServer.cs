using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using System.Net;
using System.Net.Sockets;
using System.IO;

namespace MP
{
    public class TCPServer
    {
        // receiving Thread
        Thread receiveThread;

        StreamReader streamReader1;  // 데이타 읽기 위한 스트림리더
        StreamWriter streamWriter1;  // 데이타 쓰기 위한 스트림라이터    
        private string Received_Data = "0" ;

        #region Original code
        // public
        public TCPServer()
        {
            //Initialize(); //시작
        }

        public void Close()
        {
            if (receiveThread != null)
            {
                receiveThread.Abort();
            }
        }


        private void Initialize()
        {
            Close();

            receiveThread = new Thread(new ThreadStart(ReceiveData));
            receiveThread.IsBackground = true;
            receiveThread.Start();
        }

        private void ReceiveData() // thread1에 연결된 함수. 메인폼과는 별도로 동작
        {
            TcpListener tcpListener1 = new TcpListener(IPAddress.Parse("127.0.0.1"), 22345); // 서버 객체 생성 및 IP주소와 Port번호를 할당
            tcpListener1.Start();  // 서버 시작
            Debug.WriteLine("TCP서버 준비...클라이언트 기다리는 중...");

            TcpClient tcpClient1 = tcpListener1.AcceptTcpClient(); // 클라이언트 접속 확인
            Debug.WriteLine("클라이언트 연결됨...");

            streamReader1 = new StreamReader(tcpClient1.GetStream());  // 읽기 스트림 연결
            streamWriter1 = new StreamWriter(tcpClient1.GetStream());  // 쓰기 스트림 연결
            streamWriter1.AutoFlush = true;  // 쓰기 버퍼 자동으로 뭔가 처리..
            while (tcpClient1.Connected)
            {
                string receiveData1 = streamReader1.ReadLine();  // 수신 데이타를 읽어서 receiveData1 변수에 저장
                if (receiveData1 != null) if (receiveData1.Length < 1) SendData("NA");
                    else {
                        Received_Data = receiveData1;
                    }
                Debug.WriteLine("receiveData1: " + receiveData1); // 데이타를 수신창에 쓰기 
            }
        }

        public void SendData(string sendData1)  // '보내기' 하려면
        {
            streamWriter1.WriteLine(sendData1);  // 스트림라이터를 통해 데이타를 전송             
        }


        public string DataString()
        {
            return Received_Data;
        }

        #endregion
        //----------------------------------------------------------------------------------------

        void TCPSimulcall(string i)
        {
            string serverIP = "127.0.0.1";
            int port = 1702; // Q-sys 포트 번호1702
            try
            {
                //TCP 서버에 연결
                TcpClient client = new TcpClient(serverIP, port);
                // 네트워크 스트림 생성
                NetworkStream stream = client.GetStream();
                string message = i + Environment.NewLine;//"클라이언트에서 보낼 메시지";sg
                // 서버로 데이터 보내기
                byte[] buffer = Encoding.ASCII.GetBytes(message);
                byte[] responseBytes = new byte[1024];

                stream.Write(buffer, 0, buffer.Length);
                Console.WriteLine("메시지를 보냈습니다.");
                // 서버로부터 응답 받기
                Console.WriteLine("Response 대기중.");
                int bytesRead = stream.Read(responseBytes, 0, responseBytes.Length);
                string response = Encoding.ASCII.GetString(responseBytes, 0, bytesRead);
                Console.WriteLine("서버로부터 받은 응답: " + response);

                // 연결 종료
                stream.Close();
                client.Close();
            }
            catch (Exception e)
            {
                Console.WriteLine("Error: " + e.Message);
            }

        }
        static void TCPReceive()
        {
            IPAddress ipAddress = IPAddress.Parse("127.0.0.1");
            int port = 12345;

            TcpListener server = new TcpListener(ipAddress, port);
            server.Start();
            Console.WriteLine("Server listening on port " + port);

            while (true)
            {
                TcpClient client = server.AcceptTcpClient();
                Console.WriteLine("Client connected.");

                NetworkStream stream = client.GetStream();

                byte[] data = new byte[1024];
                int bytesRead = stream.Read(data, 0, data.Length);
                string message = Encoding.ASCII.GetString(data, 0, bytesRead);
                Console.WriteLine("Received: " + message);

                byte[] response = Encoding.ASCII.GetBytes("Message received.");
                stream.Write(response, 0, response.Length);

                client.Close();
                Console.WriteLine("Client disconnected.");
            }
        }
    }
}
