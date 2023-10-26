using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Net;
using System.Net.Sockets;

namespace MP
{
    public class UDPClientTKV
    {
        List<float> Joint_array = new List<float>();
        public float[] JointAngleArray = new float[29];
        public struct Header
        {
            public byte Check_sum;
            public byte Sender;
            public byte Receiver;
            public byte Ack;
            public byte Code;
            public byte Value;
            public short Command_size;
            public byte Spare;
        }

        #region Secondery Code Sinc to simulator UDP _send____ Secondery Code Sinc to simulator UDP _____
        /* udp 로 시뮬레이터의 로봇 데이터를 송신한다.
         * 헤더(7바이트) Check_sum , Sender, Receiver, Ack, Code, Value, Data_size
         * +각도 데이터(29*4바이트)
         * +재생 상태 데이터(1바이트)
         * 를 보낸다.
         */
        //send
        Socket sender;
        private IPEndPoint porta;
        public struct UDPwControlHeader
        {
            public byte Check_sum;
            public byte Sender;
            public byte Receiver;
            public byte Ack;
            public byte Code;
            public byte Value;
            public byte Data_size;
            public byte Spare;
        }
        UDPwControlHeader UDPheader = new UDPwControlHeader { Check_sum = 0, Sender = 10, Receiver = 16, Ack = 0, Code = 0, Value = 0, Data_size = 0, Spare = 0 };
        byte Chacksum_old = 0;

        //외부에서 클래스를 부르면서 송신ip와 Port 설정
        public UDPClientTKV(string udpip, int UDPwControlPort)
        {
            UDPwControlinit(udpip, UDPwControlPort);
        }
        void UDPwControlinit(string udpip, int UDPwControlPort)
        {
            // 송신 소켓 생성
            sender = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            // 송신 주소 및 포트 설정
            IPAddress addressa = IPAddress.Parse(udpip);
            porta = new IPEndPoint(addressa, UDPwControlPort);
        }
        
        //상태 보내기
        public void UDPwCurrentSend(float[] Angle_data)
        {
            // 보낼 데이터 통합
            byte[] HeaderData = new byte[] { UDPheader.Check_sum, UDPheader.Sender, UDPheader.Receiver, UDPheader.Ack, UDPheader.Code, UDPheader.Value, UDPheader.Data_size };
            for (int i = 0; i < 29; i++) Joint_array.Add(JointAngleArray[i]); //각 슬라이더에서 조인트 각도 값 List로 받아오기(나중에 i를 진짜 값으로 업데이트 필요)

            //float[] floatData = Joint_array.ToArray();
            float[] floatData = Angle_data;
            byte[] data = new byte[HeaderData.Length + floatData.Length * 4 + 1]; //실제로 보낼 데이터 크기 헤더 7byte + 조인트 각도수*4(1float->4byte) + spare 1byte
            for (int i = 0; i < HeaderData.Length; i++)
            {
                data[i] = HeaderData[i]; //헤더데이터들 붙여넣기
            }
            for (int i = 0; i < floatData.Length; i++)
            {
                BitConverter.GetBytes(floatData[i]).CopyTo(data, HeaderData.Length + i * 4);// 헤더 다음에 Float인 조인트 어레이들 4byte로 쪼게서 붙이기
            }
            //체크섬 업데이트
            data[0] = CalcChecksum(data);
            // 데이터 전송
            if (data[0] != Chacksum_old)
            {
                sender.SendTo(data, data.Length, SocketFlags.None, porta);
                Chacksum_old = data[0];
                Debug.WriteLine("UDPSend :" + data[0] + "  Angle :" + floatData[floatData.Length - 1]);
            }
            sender.SendTo(data, data.Length, SocketFlags.None, porta);
            Joint_array.Clear();
        }

        public void UDPwControlClose()
        {
            if (sender != null)
            {
                // 송신 소켓 종료
                sender.Close();
                sender = null;
            }
        }

        //chack sum
        byte CalcChecksum(byte[] packet)
        {
            uint checksum = 0;            

            try
            {
                for (int i = 0; i < packet.Length; i++)
                {
                    checksum += packet[i];
                }
                checksum = (checksum & 0xFF);
            }
            catch
            {
                Debug.WriteLine("StructToBytes Error!");
            }
            return (byte)checksum;
        }

        #endregion Secondery Code Sinc to simulator ---------Secondery Code Sinc to simulator ---------Secondery Code Sinc to simulator ---------
    }
}
