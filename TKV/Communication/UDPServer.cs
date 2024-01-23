//시뮬레이터에서 자세 제어 통신 받는 udp 서버
using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Threading;

public class UDPServer
{
    // receiving Thread
    Thread receiveThread;
    Thread SimulReceiveThread;
    // udpclient object
    UdpClient socket;

    int mPort = 0;
    int mSleep = 1;

    public bool SimulsyncRequest = false;
    public bool SafeModeRequest = false;

    //private static Timer UDPTimer;
    Action<string, byte[]> actWork;

    #region Original code
    // public
    public UDPServer(int port, int sleep, Action<string, byte[]> work)
    {
        Initialize(port);
        UDPwControlReceiveInit(50001);

        mPort = port;
        mSleep = sleep;

        actWork = work;
    }

    public void Close()
    {
        if (receiveThread != null)
        {
            receiveThread.Abort();
            //           Debug.WriteLine("Close Server Socket IP: " + ((IPEndPoint)socket.Client.LocalEndPoint).Address
            //                           + "  Port: " + ((IPEndPoint)socket.Client.LocalEndPoint).Port);
            socket.Close();
        }
    }

    public void Restart()
    {
        Initialize(mPort);
    }

    private void Initialize(int port)
    {
        Close();

        socket = new UdpClient(port);

        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();

        Debug.WriteLine("Create Server Socket IP: " + ((IPEndPoint)socket.Client.LocalEndPoint).Address
                        + "  Port: " + ((IPEndPoint)socket.Client.LocalEndPoint).Port);
    }

    private void ReceiveData()
    {
        while (true)
        {
            try
            {
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, mPort);
                byte[] rawdata = socket.Receive(ref anyIP);

                if (rawdata.Length > 0)
                {
                    if (actWork != null) actWork(anyIP.Address.ToString(), rawdata);
                }
                //Thread.Sleep(mSleep);
            }
            catch (Exception err)
            {
                Debug.WriteLine("Server Socket Error!(port: " + mPort + ")" + err.ToString());
            }

        }
    }
    #endregion Original code

    #region Sinc Simul Receive _________Sinc Simul Receive _________Sinc Simul Receive _________ Sinc Simul Receive _________Sinc Simul Receive _________

    UdpClient receiver;
    public float[] SimulAngle = new float[29];
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
    UDPwControlHeader UDPheader = new UDPwControlHeader { Check_sum = 0, Sender = 0, Receiver = 0, Ack = 0, Code = 0, Value = 0, Data_size = 0, Spare = 0 };
    byte Chacksum_old = 0;

    void UDPwControlReceiveInit(int UDPwControlPort)
    {
        UDPwControlReceiveClose();
        //쓰레드 생성
        SimulReceiveThread = new Thread(new ThreadStart(SimulReceiveData));
        SimulReceiveThread.IsBackground = true;
        SimulReceiveThread.Start();

        // 수신 소켓 생성
        receiver = new UdpClient(UDPwControlPort);
    }
    private void SimulReceiveData()
    {
        while (true)
        {
            try
            {
                //IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 50001);
                //byte[] rawdata = receiver.Receive(ref anyIP);
                //if (rawdata[0] != Chacksum_old)
                //{
                //Console.WriteLine("Headerdata= " + rawdata[0] + ", " + rawdata[1] + ", " + rawdata[2] + ", " + rawdata[3] + ", " + rawdata[4] + ", asdffa");
                //    Chacksum_old = rawdata[0];
                //}

                UDPwControlReceive(50001);
                //Thread.Sleep(mSleep);
            }
            catch (Exception err)
            {
                Debug.WriteLine("Server Socket Error!(port: " + mPort + ")" + err.ToString());
            }

        }
    }
    void UDPwControlReceive(int UDPwControlPort)
    {
        // 데이터 수신  
        IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, UDPwControlPort);
        byte[] receivedBytes = receiver.Receive(ref anyIP);
        if (receivedBytes.Length > 0)//데이터가 들어왔을때
        {
            if (Chacksum_old != receivedBytes[0]) //데이터가 이전과 다를때
            {
                if (receivedBytes[0] == CalcChecksum(receivedBytes)) //데이터가 정상적일때
                {
                    // 수신 데이터 처리
                    //헤더 데이터
                    byte[] HeaderData = new byte[7];
                    for (int i = 0; i < 6; i++)//0~6 헤더데이터( Check_sum;  Sender;  Receiver;  Ack;  Code;  Value;  Data_size;)
                    {
                        HeaderData[i] = receivedBytes[i];
                    }
                    //  Console.WriteLine("Headerdata= " + HeaderData[0] + ", " + HeaderData[1] + ", " + HeaderData[2] + ", " + HeaderData[3] + ", " + HeaderData[4] + ", " + HeaderData[5] + ", " + HeaderData[6] + ", 1234fa");
                    switch (HeaderData[4])
                    {
                        case 0:
                            //싱크 각도 보내옴
                            break;
                        case 9://시뮬레이션 싱크 요청 수신(valu 1=sync, 00=break)
                            if (HeaderData[5] == 0) SimulsyncRequest = false;
                            else SimulsyncRequest = true;
                            break;
                        case 10: //안전모드해제 요청 (01= enable,00=disable)
                            if (HeaderData[5] == 0) SafeModeRequest = false;
                            else SafeModeRequest = true;
                            break;
                        default:

                            break;
                    }
                    //각도 데이터
                    float[] floatData = new float[(receivedBytes.Length - 7) / 4];
                    for (int i = 0; i < floatData.Length; i++) //각도 데이터 다시 묶기 byte 4개 -> float 1개
                    {
                        floatData[i] = BitConverter.ToSingle(receivedBytes, HeaderData.Length + i * 4);
                    }
                    //Console.WriteLine("floatdata= " + floatData[0] + ", " + floatData[1] + ", " + floatData[2] + ", " + floatData[3] + ", " + floatData[4] + ", " + floatData[5] + ", " + floatData[floatData.Length - 1] + "<qff52");
                    if (SimulAngle.Length == floatData.Length) SimulAngle = floatData; //각도값 개수(29개)가 정확히 들어왔으면 각도값 업데이트
                    else Console.WriteLine("The number of angle values is not matching");
                }
                Chacksum_old = receivedBytes[0]; //이번데이터 체크섬 업데이트
            }
        }
    }
    void UDPwControlReceiveClose()
    {
        // 수신 소켓 종료
        if (receiver != null)
        {
            receiver.Close();
            SimulReceiveThread.Abort();
        }


    }

    public byte CalcChecksum(byte[] packet)
    {
        uint checksum = 0;

        try
        {
            for (int i = 1; i < packet.Length; i++)
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

    #endregion Sinc Simul ------- Sinc Simul -------Sinc Simul -------Sinc Simul -------
}
