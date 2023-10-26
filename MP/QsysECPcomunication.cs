/*Q-sys 와 ECP 통신을 하기위해 만들어지 Class
 * 
 */
#define Monitoring
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace MP
{
    public class QsysECPcomunication
    {
        private Thread monitoring;
        private string QsysMessage;

        public QsysECPcomunication()
        {
            Awake();
        }
        void Awake()
        {
            //Thread - UI update
            if (monitoring == null)
            {
                monitoring = new Thread(Run);
                monitoring.IsBackground = true;
                monitoring.Start();
            }
        }
        void Run()
        {
            while (true)
            {
                Monitoring_Update();
                Thread.Sleep(1000);//50ms ECP는 지속적 통신을 하지 않으면 사용하지 않는다 판단하고 끊어진다.(반응속도가 느려진다.)
            }
        }

        private void Monitoring_Update()
        {
                    TCPcall("sg");
        }

        private void btnHDConnect_Click(object sender, EventArgs e)
        {
            TCPcall("cg LSMGain");
            string[] world = QsysMessage.Split('"');

        }

        private void btnLRConnect_Click(object sender, EventArgs e)
        {
            TCPcall("cg LSMMute");
            string[] world = QsysMessage.Split('"');


        }

        private void btnWTConnect_Click(object sender, EventArgs e)
        {
            TCPcall("cg AudioPlayerTime");
            string[] world = QsysMessage.Split('"');

        }


        //Q-syu telnet connect (In this case Q-sys is server)
        void TCPcall(string i)
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
                Console.WriteLine("대기중.");
                int bytesRead = stream.Read(responseBytes, 0, responseBytes.Length);
                Console.WriteLine("대기중..");
                string response = Encoding.ASCII.GetString(responseBytes, 0, bytesRead);
                QsysMessage = response;
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
    }
}
