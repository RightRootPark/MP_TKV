using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Net.NetworkInformation;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

using KnrHeader;
using System.ComponentModel;

[Serializable]
public class JsonTargetSocket
{
    public List<string> name;
    public List<string> IP;
    public List<int> port;
}

public class LCU_Communication
{
    //https://nshj.tistory.com/entry/C-%EA%B8%B0%EC%B4%88%EB%AC%B8%EB%B2%95-11-%EC%93%B0%EB%A0%88%EB%93%9CThread%EC%99%80-%ED%85%8C%EC%8A%A4%ED%81%ACTask
    private readonly object thisLock = new object();

    public const int MOTION_LIST_COUNT_MAX = 10;
    private const string FILE_PATH_SETTING = @"setting\";
    private const string FILE_NAME_TARGET_SOCKET = "targetsocket";
    private const string FILE_EXTENSION = ".txt";

    private const int THREAD_LOOP_TIME = 20; //ms

    public bool isConnect { get; private set; }

    public bool isEmergencyStop { get; private set; }
    public bool isCanConnect { get; private set; }
    //public bool isOnLimit { get; private set; }
    public bool isPosDiffMonitor { get; private set; }

    public byte mode; //{ get; private set; }
    public byte masterDevice { get; private set; }
    public byte gripState { get; private set; }

    //public int gripVelocityLevel { get; private set; }
    public int headerVersion { get; private set; }
    public int replyTime { get; private set; }
    public int keepTime { get; private set; }

    public float[] tcpState { get; private set; }

    public int lcuSender { get; private set; }

    //여러개의 LCU 에서 데이터를 받아오기위해 다수의 LCU 헤더 만듦
    public LcuCommunication.t_lcu_data_monitor monitoringData;
    public LcuCommunication.t_lcu_data_monitor2 monitoringData2;
    public LcuCommunication.t_lcu_data_monitor3 monitoringData3;
    public LcuCommunication.t_lcu_data_monitor4 monitoringData4;
    public LcuCommunication.t_lcu_data_monitor5 monitoringData5;

    public LcuCommunication.t_lcu_data_joint_cmd _jointVelCmd;
    public LcuCommunication.t_lcu_data_world_cmd _worldVelCmd;

    public LcuCommunication.t_lcu_data_joint_cmd_pos _jointPosCmd;
    public LcuCommunication.t_lcu_data_world_cmd_pos_M _worldPosCmd_M;
    public LcuCommunication.t_lcu_data_world_cmd_pos_D _worldPosCmd_D;
    public LcuCommunication.t_lcu_data_force _forceCmd;

    public LcuCommunication.t_lcu_data_can_udp_packet _canUDP;
    public LcuCommunication.t_lcu_data_can_udp_rx_packet _canUDPRx;
    private LcuCommunication.t_lcu_data_limit _jointLimit;

    public bool LCU_MODECHECK_VEL = true; //속도모드 체크용 플래그

    //public LcuCommunication.t_lcu_data_SV svState;
    public struct TargetJointSimulation
    {
        public bool isOnJointJoystick;
        public bool isOnJointProfile;

        public float[] targetDistance;
        public float[] targetMaxVelocity;
        public float timeTarget;
        public float timeNow;
    }

    private byte _deviceNumber;

    private UdpClient _udpClient;
    private IPEndPoint _ipEndPoint;
    private LcuCommunication.t_lcu_packet_const_size _sendCommandData;
    private LcuCommunication.t_lcu_packet_const_size _recvData;

    private byte _sender;
    private byte _receiver;

    private Dictionary<byte, bool> _isWaitAckMap;
    private List<string> _ErrorMessageList;
    private List<string> _ErrorTextList;
    //private List<string> _CanMessageList;
    private bool _isCanQueryDataUpdate;
    private string _canQueryData = "";

    //private List<string> _timeTestList;

    private Thread _thread;
    private Thread _thread2;


    private TargetJointSimulation _lcuJointSimulation;

    private bool _isUpdateSendPacket = false;
    private double _disConnectCount = 0;
    public double _sleepCount = 0;
    public bool _sleepFlag = false;
    private bool _SDO_Check = true;

    //ip socket
    private JsonTargetSocket _ipTargetSocket;

    //설명: LCU 생성자. 기본 설정 및 변수 초기화.
    public LCU_Communication(UdpClient client, IPEndPoint ipEndPoint, byte deviceNumber)
    {
        //SetDeviceHeader(LcuCommon.HEADER_RM, LcuCommon.HEADER_LCU1);
        switch (deviceNumber)
        {
            case 1:
                SetDeviceHeader(LcuCommon.HEADER_PC1, LcuCommon.HEADER_LCU1);
                break;
            case 2:
                SetDeviceHeader(LcuCommon.HEADER_PC1, LcuCommon.HEADER_LCU2);
                break;
            case 3:
                SetDeviceHeader(LcuCommon.HEADER_PC1, LcuCommon.HEADER_LCU3);
                break;
            case 4:
                SetDeviceHeader(LcuCommon.HEADER_PC1, LcuCommon.HEADER_LCU4);
                break;
            case 5:
                SetDeviceHeader(LcuCommon.HEADER_PC1, LcuCommon.HEADER_LCU5);
                break;
        }
        //SetDeviceHeader(LcuCommon.HEADER_PC1, LcuCommon.HEADER_LCU1);

        this._udpClient = client;
        this._ipEndPoint = ipEndPoint;

        monitoringData = new LcuCommunication.t_lcu_data_monitor();
        monitoringData.fJointPos = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        monitoringData.fWorldPos = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        monitoringData.fJointPosCur = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        monitoringData.fWorldPosCur = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        monitoringData.fDiffPressure = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        monitoringData.fPosError = new float();
        monitoringData.fForceZ = new float();
        monitoringData.fRmsCheck = new byte();
        monitoringData.fGripperState = new byte();
        monitoringData.fGasconState = new byte();
        monitoringData.nAutoControl = new byte();
        //monitoringData.nEmergency = new UInt16();



        mode = LcuCommunication.LCU_VAL_MODE_JOINT_VEL;

        _sendCommandData = new LcuCommunication.t_lcu_packet_const_size();
        _sendCommandData.uSender = _sender;
        _sendCommandData.uReceiver = _receiver;
        _sendCommandData.uAcknowledge = LcuCommon.NONE;

        _sendCommandData.uCode = LcuCommunication.LCU_SVC_VEL_COMMAND;
        _sendCommandData.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_joint_cmd>();
        _sendCommandData.uValue = mode;

        _sendCommandData.uData = new byte[LcuCommon.PACKET_DATA_LEN];

        //SendToLCU(_sendCommandData);

        _jointVelCmd = new LcuCommunication.t_lcu_data_joint_cmd();
        _jointVelCmd.fJointCmd = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];

        _worldVelCmd = new LcuCommunication.t_lcu_data_world_cmd();
        _worldVelCmd.fWorldCmd = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];

        _jointPosCmd = new LcuCommunication.t_lcu_data_joint_cmd_pos();
        _jointPosCmd.fJointCmd = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];

        _worldPosCmd_M = new LcuCommunication.t_lcu_data_world_cmd_pos_M();
        _worldPosCmd_M.fWorldCmd = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];

        _worldPosCmd_D = new LcuCommunication.t_lcu_data_world_cmd_pos_D();
        _worldPosCmd_D.fWorldCmd = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];

        _forceCmd = new LcuCommunication.t_lcu_data_force();
        _forceCmd.fForceCmd = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];

        //LCU에 명령을 보내는 경우 응답이 오는지 확인하기 위한 플레그 모음
        //내부에서 동작은 하고 있으나 현재 사용되지는 않는다.
        _isWaitAckMap = new Dictionary<byte, bool>();

        for (byte i = 101; i < 255; i++)
        {
            _isWaitAckMap.Add(i, false);
        }

        _ErrorMessageList = new List<string>();
        _ErrorTextList = new List<string>();
        _isCanQueryDataUpdate = false;

        //Simulation
        _lcuJointSimulation.targetDistance = new float[6];
        _lcuJointSimulation.targetMaxVelocity = new float[6];

        //로봇팔에 속도명령을 보내기위한 쓰레드
        _thread = new Thread(new ThreadStart(Control));
        _thread.IsBackground = true;
        _thread.Start();

        if (File.Exists(GetFileNameWith_deviceNumber(FILE_NAME_TARGET_SOCKET, _deviceNumber)))
        {
            _ipTargetSocket = JsonFileIO.Load<JsonTargetSocket>(GetFileNameWith_deviceNumber(FILE_NAME_TARGET_SOCKET, _deviceNumber));
        }
        else
        {
            SaveTargetSocketInit();
        }

        tcpState = new float[6];

        Debug.WriteLine("Create LCU2 Client Socket IP: " + ipEndPoint.Address + "  Port: " + ipEndPoint.Port);
    }

    //설명: 로봇팔 속도 변경. 쓰레드에서 지속적으로 보내고있는 속도(변수의 값)을 바꾼다.
    public void SetArmVelCmd(float[] velCommand, ushort stopState)
    {
        switch (this.mode)
        {
            case LcuCommunication.LCU_VAL_MODE_JOINT_VEL:
                _jointVelCmd.fJointCmd = velCommand;
                _jointVelCmd.nEmergency = stopState;
                break;
            case LcuCommunication.LCU_VAL_MODE_WORLD_VEL:
                _worldVelCmd.fWorldCmd = velCommand;
                //_worldVelCmd.th_7 = tool;
                _worldVelCmd.nEmergency = stopState;
                break;
            //case LcuCommunication.LCU_VAL_MODE_TOOL_VEL:
            //    _worldVelCmd.fWorldCmd = velCommand;
            //    _worldVelCmd.nEmergency = stopState;
            //    break;
            //case LcuCommunication.LCU_VAL_MODE_JOINT_POS:
            //    //_jointVelCmd.fJointCmd = velCommand;
            //    //_jointVelCmd.nEmergency = stopState;
            //    _jointPosCmd.fJointCmd = velCommand;                        
            //    break;
            default:
                _jointVelCmd.fJointCmd = velCommand;
                _jointVelCmd.nEmergency = stopState;
                break;
        }
    }

    //설명: 로봇팔 위치 제어. 
    public void SetArmPosCmd(float[] posCommand, float ToolCmd, float RMS)
    {
        switch (this.mode)
        {
            case LcuCommunication.LCU_VAL_MODE_JOINT_POS:
                if (posCommand[0] == 0) posCommand[0] = +0.0001f;//LCU 1,2 조인트 0되면 통신에러로 생각하고 전부 끊어지는 안전기능 무력화
                if (posCommand[1] == 0) posCommand[1] = +0.0001f;
                _jointPosCmd.fJointCmd = posCommand;
                SendCommandPosition(_jointPosCmd);
                Debug.WriteLine(_jointPosCmd.fJointCmd[0] + " " + _jointPosCmd.fJointCmd[1] + " " + _jointPosCmd.fJointCmd[2]);// + " " + _jointPosCmd.fJointCmd[6] + " " + _jointPosCmd.fJointCmd[7]);
                break;
            case LcuCommunication.LCU_VAL_MODE_WORLD_POS_M:
                _worldPosCmd_M.fWorldCmd = posCommand;
                _worldPosCmd_M.toolCmd = ToolCmd;
                _worldPosCmd_M.RMS = RMS;
                SendCommandPosition_World(_worldPosCmd_M);
                Debug.WriteLine(_worldPosCmd_M.fWorldCmd[0] + " " + _worldPosCmd_M.fWorldCmd[1] + " " + _worldPosCmd_M.fWorldCmd[2] + " " + _worldPosCmd_M.toolCmd + " " + _worldPosCmd_M.RMS);
                break;
            case LcuCommunication.LCU_VAL_MODE_WORLD_POS_D:
                _worldPosCmd_D.fWorldCmd = posCommand;
                _worldPosCmd_D.toolCmd = ToolCmd;
                _worldPosCmd_D.RMS = RMS;
                SendCommandPosition_World(_worldPosCmd_D);
                Debug.WriteLine(_worldPosCmd_D.fWorldCmd[0] + " " + _worldPosCmd_D.fWorldCmd[1] + " " + _worldPosCmd_D.fWorldCmd[2] + " " + _worldPosCmd_D.toolCmd + " " + _worldPosCmd_D.RMS);
                break;
            default:
                _jointPosCmd.fJointCmd = posCommand;
                //_jointVelCmd.nEmergency = stopState;
                break;
        }
    }

    //설명: 로봇팔 위치 제어. 
    public void SetArmForceCmd(float[] forceCommand)
    {
        switch (this.mode)
        {
            case LcuCommunication.LCU_VAL_MODE_FORCE:
                _forceCmd.fForceCmd = forceCommand;
                SendCommandPosition_Force(_forceCmd);
                Debug.WriteLine(_forceCmd.fForceCmd[2] + " " + _forceCmd.fForceCmd[5]);
                break;
        }
    }

    //설명: 로봇팔 및 모바일 명령을 보내는 쓰레드 루프
    void Control()
    {
        //Process proc = Process.GetCurrentProcess();
        //double startProcessorTime = proc.UserProcessorTime.TotalMilliseconds;
        //double integralTime = 0;
        //ulong timeCount = 1;

        Process proc = Process.GetCurrentProcess();
        double startProcessorTime = proc.UserProcessorTime.TotalMilliseconds;
        double previousTime = proc.UserProcessorTime.TotalMilliseconds;
        double currentTime = 0;
        double deltaTime = 0;

        while (true)
        {
            currentTime = proc.UserProcessorTime.TotalMilliseconds;
            deltaTime = currentTime - previousTime;

            //integralTime = proc.UserProcessorTime.TotalMilliseconds - startProcessorTime;
            //if (integralTime >= THREAD_LOOP_TIME * timeCount)
            if (deltaTime >= 10)// & _started == true)
            {
                //Check Disconnection of LCU  
                //3초간 응답이 없을 경우 연결을 끊음
                if (_disConnectCount >= 300f)// / (float)THREAD_LOOP_TIME)
                {
                    isConnect = false;
                    isCanConnect = false;
                }
                else
                {
                    _disConnectCount++;
                }
                //if ( _sleepCount>= 2000f)// 20초 조작없음
                //{
                //    if (_sleepFlag) { } //true일때 미작동
                //    else _sleepFlag = true; //false 일때 true로 변경
                //}
                //else _sleepCount++;

                //    //시뮬레이션일 경우
                //    if (_lcuJointSimulation.isOnJointJoystick)
                //    {
                //        if (_lcuJointSimulation.isOnJointProfile)
                //        {
                //            for (int i = 0; i < KnrHeader.LcuCommon.ROBOT_AXIS_CNT_MAX; i++)
                //            {
                //                monitoringData.fJointPos[i] += _lcuJointSimulation.targetMaxVelocity[i] * (float)THREAD_LOOP_TIME / 1000.0f;
                //            }

                //            //시뮬레이션 가상 조인트 데이터 생성
                //            _lcuJointSimulation.timeNow += (float)THREAD_LOOP_TIME;

                //            if (_lcuJointSimulation.timeNow >= _lcuJointSimulation.timeTarget)
                //            {
                //                _lcuJointSimulation.isOnJointProfile = false;
                //                Debug.WriteLine("profile simulation state: " + _lcuJointSimulation.isOnJointProfile);
                //            }

                //        }

                //        for (int i = 0; i < KnrHeader.LcuCommon.ROBOT_AXIS_CNT_MAX; i++)
                //        {
                //            monitoringData.fJointPos[i] += _jointVelCmd.fJointCmd[i] * (float)THREAD_LOOP_TIME / 1000.0f;
                //        }
                //    }
                //    else
                //    {
                //일반 상태의 경우
                if (isConnect && LCU_MODECHECK_VEL)// && masterDevice == LcuCommon.HEADER_PC1)
                {
                    switch (this.mode)
                    {
                        //로봇팔의 모드에 따라 속도 명령을 보냄. 
                        case LcuCommunication.LCU_VAL_MODE_JOINT_VEL:
                            _SDO_Check = false; //SDO 플래그 해제
                            SendCommandVelocity(_jointVelCmd);
                            //Debug.WriteLine("Joint - Vel");
                            break;
                        case LcuCommunication.LCU_VAL_MODE_WORLD_VEL:
                            _SDO_Check = false;
                            SendCommandVelocity(_worldVelCmd);
                            break;
                        //case LcuCommunication.LCU_VAL_MODE_TOOL_VEL:
                        //    SendCommandVelocity(_worldVelCmd);
                        //    Debug.WriteLine("SendCommandVelocity(_worldVelCmd);");
                        ////    break;
                        //case LcuCommunication.LCU_VAL_MODE_JOINT_POS:
                        //    SendCommandPosition(_jointPosCmd);
                        //    break;
                        //case LcuCommunication.LCU_VAL_MODE_WORLD_POS_M:
                        //    SendCommandPosition_World(_worldPosCmd_M);
                        //    break;
                        //case LcuCommunication.LCU_VAL_MODE_WORLD_POS_D:
                        //    SendCommandPosition_World(_worldPosCmd_D);
                        //    break;
                        default:
                            //SendCommandVelocity(_jointVelCmd);
                            break;

                    }
                }
            }
            //startProcessorTime = proc.UserProcessorTime.TotalMilliseconds;
            //timeCount++;
            //Debug.WriteLine(_jointVelCmd.fJointCmd[0] + " " + _jointVelCmd.fJointCmd[1] + " " + _jointVelCmd.fJointCmd[2]);
            Thread.Sleep(1);
        }
    }

    //설명: 외부에서 호출하여 쓰레드를 종료시킴.
    public void Close()
    {
        if (_thread != null)
        {
            _thread.Abort();
            Debug.WriteLine("Abort LCU thread");

            /*
            if (File.Exists(@"c:\test\threadTimeCheck.csv"))
            {
                File.Delete(@"c:\test\threadTimeCheck.csv");
            }
            StreamWriter file = new StreamWriter(@"c:\test\threadTimeCheck.csv");
            _timeTestList.ForEach(file.WriteLine);
            file.Close();
            UnityEngine.Debug.Log("file write done");
            */
        }
    }

    //설명: 디바이스의 헤더(넘버링)를 설정함. ex) LCU-1번, LCU-2번 ...
    public void SetDeviceHeader(byte thisDevice, byte targetDevice)
    {
        _sender = thisDevice;
        _receiver = targetDevice;

        _isUpdateSendPacket = true;
        _sendCommandData.uSender = _sender;
        _sendCommandData.uReceiver = _receiver;
        _isUpdateSendPacket = false;
    }

    //설명: UDP 통신을 활용하여 LCU로 데이터를 보냄.
    //*Use Const size packet for Covertring Union to Pointer Error
    public void SendToLCU(LcuCommunication.t_lcu_packet_const_size packet)
    {
        if (_SDO_Check) _sleepCount = 0;//SDO 신호 보낼때 sleep count 리셋
        _SDO_Check = true; //SDO플래그 재설정
                           //구조체를 바이트 배열로 변환
                           // first of All, packet should have all data before use ClacCheckSum()
                           //*Use Const size packet for Covertring Union to Pointer Error


        byte[] bytes = TypeConvert.StructToBytes(packet, LcuCommon.PACKET_HEADER_LEN + LcuCommon.PACKET_NULL_LEN + 119); // packet.nDataSize);


        for (int i = 0; i < packet.nDataSize; i++)
        {
            bytes[LcuCommon.PACKET_HEADER_LEN + i] = packet.uData[i];
            //Debug.WriteLine("SendToLCU : " + packet.uData[i]);// bytes.Length);
        }
        bytes[bytes.Length - 1] = 0; //마지막 바이트: NULL
        bytes[0] = LcuCommon.CalcChecksum(bytes); //checksum Index = 0

        ////ParseReceive(bytes); //test        

        //응답 확인 여부 결정
        //Check Wait Acknowledge
        if (packet.uAcknowledge > 0) SetIsWaitAck(packet.uCode, true);
        else SetIsWaitAck(packet.uCode, false);

        try
        {
            //LCU로 해당데이터를 보냄
            _udpClient.Send(bytes, bytes.Length, _ipEndPoint);
            //Debug.WriteLine("PC/ Send to LCU!");
            //Debug.WriteLine("bytes lenth = " + bytes.Length);
            //Debug.WriteLine("packet data size = " + (packet.nDataSize + LcuCommon.PACKET_HEADER_LEN + LcuCommon.PACKET_NULL_LEN).ToString());
        }
        catch (Exception err)
        {
            Debug.WriteLine(err.ToString());
        }
    }

    //설명: 응답 확인 여부 결정
    public void SetIsWaitAck(byte Code, bool isWaitAck)
    {
        if (Code == LcuCommunication.LCU_SVC_MONITORING_DATA ||
            Code == LcuCommunication.LCU_SVC_VEL_COMMAND ||
            Code == LcuCommunication.LCU_SVC_POS_COMMAND ||     // Added by Hmshin _ 20230207
            Code == LcuCommunication.LCU_SVC_TEMP)
            return;

        if (_isWaitAckMap.ContainsKey(Code))
        {
            //UnityEngine.Debug.Log("ACK Code:" + Code + "/ State: " + isWaitAck);
            _isWaitAckMap[Code] = isWaitAck;
        }
    }

    //설명: 응답 완료 여부 확인
    public bool GetIsWaitAck(byte Code)
    {
        if (!_isWaitAckMap.ContainsKey(Code)) return true;

        return _isWaitAckMap[Code];
    }

    //설명: LCU2 초기화
    public void SendInitStart()
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = RmCommunication.RM_SVC_SYS_INI_START;
        packet.uValue = 0;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_sys_init>();

        LcuCommunication.t_lcu_data_sys_init data = new LcuCommunication.t_lcu_data_sys_init();
        data.nVersion = RmCommunication.nRM_VERSION;

        packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);


        SendToLCU(packet);
    }

    //설명: PID 게인 중 I 게인의 에러를 0으로 초기화
    public void SendErrorSumClear()
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_CLEAR_ERRSUM;
        packet.uValue = 0;
        packet.nDataSize = 0;

        SendToLCU(packet);
    }

    //설명: LCU가 에러 상태인 경우 에러 상태를 해제 시킴.
    public void SendErrorReset()
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = RmCommunication.RM_SVC_ERROR_RESET;
        packet.uValue = 0;
        packet.nDataSize = 0;

        SendToLCU(packet);
    }

    //설명: LUC의 모드를 변경.
    public void SendSetMode(byte mode)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_MODE;
        packet.uValue = mode;
        packet.nDataSize = 0;

        SendToLCU(packet);

        _isUpdateSendPacket = true;
        _sendCommandData.uValue = mode;
        this.mode = mode;// LCU 회신 오류 무시용
        switch (mode)
        {
            case LcuCommunication.LCU_VAL_MODE_JOINT_VEL:
                _sendCommandData.uCode = LcuCommunication.LCU_SVC_VEL_COMMAND;
                _sendCommandData.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_joint_cmd>();
                Debug.WriteLine("LCU_VAL_MODE_JOINT_VEL");
                break;
            case LcuCommunication.LCU_VAL_MODE_WORLD_VEL:
                _sendCommandData.uCode = LcuCommunication.LCU_SVC_VEL_COMMAND;
                _sendCommandData.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_world_cmd>();
                Debug.WriteLine("LCU_VAL_MODE_WORLD_VEL");
                break;
            //case LcuCommunication.LCU_VAL_MODE_TOOL_VEL:
            //    _sendCommandData.uCode = LcuCommunication.LCU_SVC_VEL_COMMAND;
            //    _sendCommandData.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_world_cmd>();
            //    break;
            case LcuCommunication.LCU_VAL_MODE_JOINT_POS:
                _sendCommandData.uCode = LcuCommunication.LCU_SVC_POS_COMMAND;
                _sendCommandData.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_joint_cmd_pos>();
                Debug.WriteLine("LCU_VAL_MODE_JOINT_POS");
                break;
            case LcuCommunication.LCU_VAL_MODE_WORLD_POS_M:
                _sendCommandData.uCode = LcuCommunication.LCU_SVC_POS_COMMAND;
                _sendCommandData.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_world_cmd_pos_M>();
                Debug.WriteLine("LCU_VAL_MODE_WORLD_POS-Mea");
                break;
            case LcuCommunication.LCU_VAL_MODE_WORLD_POS_D:
                _sendCommandData.uCode = LcuCommunication.LCU_SVC_POS_COMMAND;
                _sendCommandData.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_world_cmd_pos_D>();
                Debug.WriteLine("LCU_VAL_MODE_WORLD_POS-Des");
                break;
            case LcuCommunication.LCU_VAL_MODE_FORCE:
                _sendCommandData.uCode = LcuCommunication.LCU_SVC_POS_COMMAND;
                _sendCommandData.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_force>();
                Debug.WriteLine("LCU_VAL_MODE_FORCE");
                break;
                //case LcuCommunication.LCU_VAL_MODE_JOINT_POS_PROF:
                //    _sendCommandData.uCode = LcuCommunication.LCU_SVC_POS_COMMAND;
                //    _sendCommandData.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_joint_cmd_pos_prof>();
                //    break;
                //case LcuCommunication.LCU_VAL_MODE_WORLD_POS_PROF:
                //    _sendCommandData.uCode = LcuCommunication.LCU_SVC_POS_COMMAND;
                //    _sendCommandData.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_world_cmd_pos_prof>();
                //    break;
                /*default:
                    _sendCommandData.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_joint_cmd>();
                    break;*/
        }
        _isUpdateSendPacket = false;
    }

    //설명: LCU의 Keeptime(명령없이도 마지막 명령을 유지하는 최대 시간)을 ms단위로 설정.
    public void SendSetKeepTime(int keepTime)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_KEEP_TIME;
        packet.uValue = 0;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_keep_time>();

        packet.uData = new byte[packet.nDataSize];
        LcuCommunication.t_lcu_data_keep_time data = new LcuCommunication.t_lcu_data_keep_time();
        data.nKeepTime = (ushort)keepTime;

        packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }

    //설명: LCU에서 보내는 주기적 데이터(조인트각도, 월드 좌표, 압력 등등)의 주기를 ms단위로 설정
    public void SendSetReplyTime(byte replyTime)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_DATA_REPLY;
        packet.uValue = replyTime;
        packet.nDataSize = 0;

        SendToLCU(packet);
    }

    //설명: LCU로 긴급정지 명령을 보냄. 솔레노이드 밸브가 닫힘.
    public void SendSetEmergency()
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = RmCommunication.RM_SVC_SYSTEM_STOP;
        packet.uValue = RmCommunication.RM_VAL_SYSTEM_STOP_EMERGENCY;
        packet.nDataSize = 0;

        SendToLCU(packet);
        /*switch (this._mode)
        {
            case LcuCommunication.LCU_VAL_MODE_JOINT_VEL:
                _sendCommandData.uData.jointCmd.nEmergency = LcuCommon.TURN_ON;
                break;
            case LcuCommunication.LCU_VAL_MODE_WORLD_VEL:
                _sendCommandData.uData.worldCmd.nEmergency = LcuCommon.TURN_ON;
                break;
            case LcuCommunication.LCU_VAL_MODE_TOOL_VEL:
                _sendCommandData.uData.worldCmd.nEmergency = LcuCommon.TURN_ON;
                break;
            default:
                _sendCommandData.uData.jointCmd.nEmergency = LcuCommon.TURN_ON;
                break;
        }*/
    }

    //설명: LCU의 최대 오차 각도를 설정. 해당 각도 이상의 이상 값 - 엔코더 값의 차이(위치 오차)가 발생시 로봇이 정지상태에 들어감. 솔레노이드 밸브는 닫히지 않음.
    //리셋 명령을 보내면 해당 정지상태를 해제할 수 있다.
    //기본값 20degree
    public void SendSetLimitStop(byte limitAngel)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_LIMIT_ERROR;
        if (limitAngel > 0) packet.uValue = limitAngel;
        else packet.uValue = LcuCommunication.LCU_VAL_LIMIT_ERROR_DISABLE;
        packet.nDataSize = 0;

        SendToLCU(packet);
    }

    //설명: LCU의 위치 오차를 모니터링할 수 있다. 현재 조인트각도에 실제 엔코더 값이 대입되고 압력 값에 위치 오차 값이 대입된다. 
    public void SendSetPosDiffMonitor(bool isOn)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_POSDIFF_MONITOR;
        if (isOn) packet.uValue = LcuCommon.TURN_ON;
        else packet.uValue = LcuCommon.TURN_OFF;
        packet.nDataSize = 0;

        SendToLCU(packet);
    }

    //설명: LCU에 조이스틱(RM) 외에 다른 PC가 연결될 때 명령이 충돌되는 것을 막기 위해 사용. 마지막으로 명령을 보낸 디바이스가 마스터가 된다.
    public void SendSetMasterControl()
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_SET_CONTROL;
        packet.uValue = _sender;
        packet.nDataSize = 0;

        SendToLCU(packet);
    }

    //설명: LCU - Simulator mode 설정.
    public void SendSetSimulator_ON()
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_SET_SIMULATOR_ON;
        packet.uValue = _sender;
        packet.nDataSize = 0;

        SendToLCU(packet);
    }
    //설명: LCU - Simulator mode 설정.
    public void SendSetSimulator_OFF()
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_SET_SIMULATOR_OFF;
        packet.uValue = _sender;
        packet.nDataSize = 0;

        SendToLCU(packet);
    }

    //설명: LCU의 IP 및 port를 변경할 때 사용. LCU의 IP를 잃어버릴 가능성이 있기때문에 사용에 유의.
    public void SendSetIP(byte ipValue, LcuCommunication.t_lcu_data_ip_packet ipData)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_IP_CHANGE;
        packet.uValue = ipValue;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_ip_packet>();

        packet.uData = new byte[packet.nDataSize];
        LcuCommunication.t_lcu_data_ip_packet data = new LcuCommunication.t_lcu_data_ip_packet();
        data = ipData;

        packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }

    //설명: LCU 로봇팔의 TCP(tool center point)를 변경한다. TOOL 모드에만 사용.
    public void SendSetTCP(LcuCommunication.t_lcu_data_tcp tcpData)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_TCP;
        packet.uValue = LcuCommunication.LCU_VAL_TCP_SET;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_tcp>();

        packet.uData = new byte[packet.nDataSize];
        LcuCommunication.t_lcu_data_tcp data = new LcuCommunication.t_lcu_data_tcp();
        data = tcpData;

        packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }

    //설명: LCU 로봇팔의 TCP(tool center point)를 요청.
    public void SendGetTCP()
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_TCP;
        packet.uValue = LcuCommunication.LCU_VAL_TCP_STATE;
        packet.nDataSize = 0;

        SendToLCU(packet);
    }

    //설명: LCU의 CAN 데이터를 상위(조이스틱)까지 전송하게 함. 또는 끔.
    public void SendSetCanConnect(bool isOn)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_CAN_CONNECT;
        if (isOn) packet.uValue = LcuCommunication.LCU_VAL_CAN_CONNECT;
        else packet.uValue = LcuCommunication.LCU_VAL_CAN_DISCONNECT;
        packet.nDataSize = 0;

        SendToLCU(packet);
    }

    //설명: canData를 LCU로 전송.
    //@override
    public void SendCanUDPForcePacket(LcuCommunication.t_lcu_data_can_udp_packet canUDPData)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_VAL_MODE_FORCE;
        packet.uValue = 0;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_can_udp_packet>();

        packet.uData = new byte[packet.nDataSize];
        LcuCommunication.t_lcu_data_can_udp_packet data = new LcuCommunication.t_lcu_data_can_udp_packet();
        data = canUDPData;

        packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }


    //설명: canData를 LCU로 전송.
    //@override
    public void SendCanUDPPacket(LcuCommunication.t_lcu_data_can_udp_packet canUDPData)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_CAN_SEND;
        packet.uValue = 0;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_can_udp_packet>();

        packet.uData = new byte[packet.nDataSize];
        LcuCommunication.t_lcu_data_can_udp_packet data = new LcuCommunication.t_lcu_data_can_udp_packet();
        data = canUDPData;

        packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }

    //설명: canData를 LCU로 전송.
    //@override
    private void SendCanPacket(LcuCommunication.t_lcu_data_can_packet canData)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_CAN_SEND;
        packet.uValue = 0;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_can_packet>();

        packet.uData = new byte[packet.nDataSize];
        LcuCommunication.t_lcu_data_can_packet data = new LcuCommunication.t_lcu_data_can_packet();
        data = canData;

        packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }

    //설명: canData를 LCU로 전송. 응답 여부를 결정.
    //@override
    private void SendCanPacket(LcuCommunication.t_lcu_data_can_packet canData, bool isOnAck)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = Convert.ToByte(isOnAck);
        packet.uCode = LcuCommunication.LCU_SVC_CAN_SEND;
        packet.uValue = 0;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_can_packet>();

        packet.uData = new byte[packet.nDataSize];
        LcuCommunication.t_lcu_data_can_packet data = new LcuCommunication.t_lcu_data_can_packet();
        data = canData;

        packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }

    //설명: Grip 명령을 보냄. 그리퍼가 움직이는 속도를 0~10까지 설정가능.
    public void SendCommandGrip(bool isOpen, int velocityLevel)
    {
        if (velocityLevel > 10) velocityLevel = 10;
        else if (velocityLevel < 0) velocityLevel = 0;

        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_GRIP;
        if (isOpen) packet.uValue = LcuCommunication.LCU_VAL_GRIP_ON_OPEN;
        else packet.uValue = LcuCommunication.LCU_VAL_GRIP_ON_CLOSE;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_grip_vel>();

        LcuCommunication.t_lcu_data_grip_vel data = new LcuCommunication.t_lcu_data_grip_vel();
        data.nVelLevel = (ushort)velocityLevel;
        packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }

    //설명: 그리퍼를 멈춤. 서보밸브 중립이기때문에 한쪽 방향으로 흐름.
    public void SendCommandGripStop()
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_GRIP;
        packet.uValue = LcuCommunication.LCU_VAL_GRIP_OFF;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_grip_vel>();

        packet.uData = new byte[packet.nDataSize];
        //LcuCommunication.t_lcu_data_grip_vel data = new LcuCommunication.t_lcu_data_grip_vel();
        //data .nVelLevel= 0;

        //packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }

    //설명: Grip 명령을 보냄. 그리퍼가 움직이는 속도를 0~10까지 설정가능.
    public void SendCommandGasCon()
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_GAS_CON;
        packet.uValue = LcuCommunication.LCU_VAL_GAS_CON_ON;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_grip_vel>();

        packet.uData = new byte[packet.nDataSize];
        //LcuCommunication.t_lcu_data_grip_vel data = new LcuCommunication.t_lcu_data_grip_vel();
        //data .nVelLevel= 0;

        //packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }

    //설명: 그리퍼를 멈춤. 서보밸브 중립이기때문에 한쪽 방향으로 흐름.
    public void SendCommandGasConStop()
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_GAS_CON;
        packet.uValue = LcuCommunication.LCU_VAL_GAS_CON_OFF;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_grip_vel>();

        packet.uData = new byte[packet.nDataSize];
        //LcuCommunication.t_lcu_data_grip_vel data = new LcuCommunication.t_lcu_data_grip_vel();
        //data .nVelLevel= 0;

        //packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }


    //설명: LCU에 설정된 각도 Limit을 요청. limit stop과는 다른 개념. limit stop은 오차 발생시 정지. 
    //여기서 각도 Limit은 엑추에이터의 스팩을 HW limit으로 표현
    //사용자가 움직임을 제한하는 각도는 SW limit으로 표현
    public void SendLCU2GetLimit(int limitKindValue)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_LIMIT_CHANGE;
        packet.uValue = (byte)limitKindValue;
        packet.nDataSize = 0;

        SendToLCU(packet);
    }

    //설명: LCU에 각도 Limit을 설정.
    public void SendLCU2SetLimit(LcuCommunication.t_lcu_data_limit limitData, int limitKindValue)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_LIMIT_CHANGE;
        packet.uValue = (byte)limitKindValue;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_limit>();

        packet.uData = new byte[packet.nDataSize];
        LcuCommunication.t_lcu_data_limit data = new LcuCommunication.t_lcu_data_limit();
        data = limitData;

        packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }

    //설명: LCU의 Sender(송신시 사용되는 디바이스 넘버)를 변경.
    private void SendLCU2Sender(LcuCommunication.t_lcu_data_sender sender, int state)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_SENDER_CHANGE;
        packet.uValue = (byte)state;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_sender>();

        packet.uData = new byte[packet.nDataSize];
        LcuCommunication.t_lcu_data_sender data = new LcuCommunication.t_lcu_data_sender();
        data = sender;

        packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }

    //설명: LCU로 로봇팔의 속도 명령을 보내는 함수. 조인트 속도.
    public void SendCommandVelocity(LcuCommunication.t_lcu_data_joint_cmd jointVelCmd)
    {
        if (!_isUpdateSendPacket)
        {
            jointVelCmd.nEmergency = 0;
            _sendCommandData.uData = TypeConvert.StructToBytes(jointVelCmd, _sendCommandData.nDataSize);

            SendToLCU(_sendCommandData);
        }
    }

    //설명: LCU로 로봇팔의 속도 명령을 보내는 함수. 월드 속도.
    public void SendCommandVelocity(LcuCommunication.t_lcu_data_world_cmd worldVelCmd)
    {
        if (!_isUpdateSendPacket)
        {
            worldVelCmd.nEmergency = 0;
            _sendCommandData.uData = TypeConvert.StructToBytes(worldVelCmd, _sendCommandData.nDataSize);

            SendToLCU(_sendCommandData);
        }
    }

    //설명: LCU로 로봇팔의 속도 명령을 보내는 함수. 모바일 속도.
    public void SendCommandVelocity(LcuCommunication.t_lcu_data_mobile_cmd mobileVelCmd)
    {
        if (!_isUpdateSendPacket)
        {
            mobileVelCmd.nEmergency = 0;
            _sendCommandData.uData = TypeConvert.StructToBytes(mobileVelCmd, _sendCommandData.nDataSize);
            SendToLCU(_sendCommandData);
        }
    }

    //설명: LCU로 로봇팔의 위치 명령을 보내는 함수. 조인트 위치 이동(한 스텝에 순식간에 이동. 위험).
    public void SendCommandPosition(LcuCommunication.t_lcu_data_joint_cmd_pos jointPosCmd)
    {
        _sendCommandData.uData = TypeConvert.StructToBytes(jointPosCmd, _sendCommandData.nDataSize);
        SendToLCU(_sendCommandData);
    }

    //설명: LCU로 로봇팔의 위치 명령을 보내는 함수. 월드 위치 이동(가감속 적용).
    public void SendCommandPosition_World(LcuCommunication.t_lcu_data_world_cmd_pos_M worldPosCmd)
    {
        _sendCommandData.uData = TypeConvert.StructToBytes(worldPosCmd, _sendCommandData.nDataSize);
        SendToLCU(_sendCommandData);
    }
    //설명: LCU로 로봇팔의 위치 명령을 보내는 함수. 월드 위치 이동(가감속 적용).
    public void SendCommandPosition_World(LcuCommunication.t_lcu_data_world_cmd_pos_D worldPosCmd)
    {
        //_sendCommandData.uCode = LcuCommunication.LCU_SVC_POS_COMMAND;
        //_sendCommandData.uValue = mode;
        _sendCommandData.uData = TypeConvert.StructToBytes(worldPosCmd, _sendCommandData.nDataSize);
        SendToLCU(_sendCommandData);
    }
    //설명: LCU로 로봇팔의 힘 명령을 보내는 함수
    public void SendCommandPosition_Force(LcuCommunication.t_lcu_data_force fForceCmd)
    {
        //_sendCommandData.uCode = LcuCommunication.LCU_SVC_POS_COMMAND;
        _sendCommandData.uData = TypeConvert.StructToBytes(fForceCmd, _sendCommandData.nDataSize);
        SendToLCU(_sendCommandData);
    }
    //설명: LCU로 로봇팔의 위치 명령을 보내는 함수. 조인트 자동 위치 이동(지정한 시간만큼 이동).
    public void SendCommandPositionProfile(LcuCommunication.t_lcu_data_joint_cmd_pos_prof jointPosProfCmd)
    {
        _sendCommandData.uData = TypeConvert.StructToBytes(jointPosProfCmd, _sendCommandData.nDataSize);
        SendToLCU(_sendCommandData);
    }

    //설명: LCU로 로봇팔의 속도 명령을 보내는 함수. 월드 자동 위치 이동(지정한 시간만큼 이동. 제대로 동작하지 않음. xyz만 적용).
    public void SendCommandPositionProfile(LcuCommunication.t_lcu_data_world_cmd_pos_prof worldPosProfCmd)
    {
        _sendCommandData.uData = TypeConvert.StructToBytes(worldPosProfCmd, _sendCommandData.nDataSize);
        SendToLCU(_sendCommandData);
    }

    //설명: Conversion gain 설정 - Target 수렴 속도 조절
    public void SendSetPosGain(float _conveGain, float _toolVel, float[] _velLimit, float[] _accelLimit, float[] _jerkLimit)
    {
        LcuCommunication.t_lcu_packet_const_size packet = new LcuCommunication.t_lcu_packet_const_size();

        packet.uSender = _sender;
        packet.uReceiver = _receiver;
        packet.uAcknowledge = LcuCommon.ACK;
        packet.uCode = LcuCommunication.LCU_SVC_POS_GAIN;
        packet.uValue = 0;
        packet.nDataSize = TypeConvert.SizeOfStruct<LcuCommunication.t_lcu_data_pos_gain>();

        packet.uData = new byte[packet.nDataSize];
        LcuCommunication.t_lcu_data_pos_gain data = new LcuCommunication.t_lcu_data_pos_gain();
        data.fVelLimit = _velLimit;
        data.fAccelLimit = _accelLimit;
        data.fJerkLimit = _jerkLimit;
        data.fConveGain = _conveGain;
        data.fToolVel = _toolVel;

        packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
        Debug.WriteLine(data.fAccelLimit[0] + " " + data.fAccelLimit[1] + " " + data.fAccelLimit[2] + " " + data.fJerkLimit[0] + " " + data.fJerkLimit[1] + " " + data.fJerkLimit[2]);
    }

    //설명: UDP 통신으로 LCU에서 보낸 데이터 파싱.
    //패킷정의는 KNR_LCU_Packet_ ServiceList.xlsx 참고
    public void ParseReceive(byte[] bytes)
    {
        if (bytes.Length <= 0)
        {
            return;
        }
        _disConnectCount = 0;
        _recvData = TypeConvert.BytesToStruct<LcuCommunication.t_lcu_packet_const_size>(bytes);

        byte[] data = new byte[_recvData.nDataSize];

        for (int i = 0; i < _recvData.nDataSize; i++)
        {
            data[i] = bytes[LcuCommon.PACKET_HEADER_LEN + i];
        }

        SetIsWaitAck(_recvData.uCode, false);

        switch (_recvData.uCode)
        {
            case RmCommunication.RM_SVC_SYS_INI_START:
                LcuCommunication.t_lcu_data_sys_init init = TypeConvert.BytesToStruct<LcuCommunication.t_lcu_data_sys_init>(data);
                headerVersion = init.nVersion;
                break;
            case RmCommunication.RM_SVC_SYSTEM_STOP:
                isEmergencyStop = true;
                break;
            case RmCommunication.RM_SVC_ERROR_RESET:
                isEmergencyStop = false;
                break;

            case LcuCommunication.LCU_SVC_CAN_CONNECT:
                if (_recvData.uValue == LcuCommunication.LCU_VAL_CAN_CONNECT) isCanConnect = true;
                else isCanConnect = false;
                break;
            case LcuCommunication.LCU_SVC_LIMIT_ERROR:
                /*if (_recvData.uValue == LcuCommunication.LCU_VAL_LIMIT_ERROR_ENABLE) isOnLimit = true;
                else isOnLimit = false;*/
                break;
            case LcuCommunication.LCU_SVC_DATA_REPLY:
                replyTime = _recvData.uValue;
                break;
            case LcuCommunication.LCU_SVC_KEEP_TIME:
                LcuCommunication.t_lcu_data_keep_time time = TypeConvert.BytesToStruct<LcuCommunication.t_lcu_data_keep_time>(data);
                keepTime = time.nKeepTime;
                break;
            case LcuCommunication.LCU_SVC_TCP:
                if (_recvData.uValue == LcuCommunication.LCU_VAL_TCP_STATE)
                {
                    LcuCommunication.t_lcu_data_tcp tcp = TypeConvert.BytesToStruct<LcuCommunication.t_lcu_data_tcp>(data);
                    tcp.fWorldPos.CopyTo(tcpState, 0);
                }
                break;

            case LcuCommunication.LCU_SVC_GRIP:
                gripState = _recvData.uValue;

                /*LcuCommunication.t_lcu_data_grip_vel grip = TypeConvert.BytesToStruct<LcuCommunication.t_lcu_data_grip_vel>(data);
                gripVelocityLevel = grip.nVelLevel;*/
                Debug.WriteLine("recv/ grip state = " + gripState);
                //UnityEngine.Debug.Log("recv/ grip vel = " + grip.nVelLevel);
                break;
            case LcuCommunication.LCU_SVC_ERROR:
                LcuCommunication.t_lcu_data_error err = TypeConvert.BytesToStruct<LcuCommunication.t_lcu_data_error>(data);
                _ErrorMessageList.Add(ParseErrorToString(err.nError, true));
                _ErrorTextList.Add(ParseErrorToString(err.nError, false));
                break;
            case LcuCommunication.LCU_SVC_CAN_RECEIVE:
                //LcuCommunication.t_lcu_data_can_packet canPacket = TypeConvert.BytesToStruct<LcuCommunication.t_lcu_data_can_packet>(data);
                ////_CanMessageList.Add(ParseCanReceiveToString(canPacket));
                ////Debug.WriteLine("can read!");
                ////Debug.WriteLine(ParseCanReceiveToString(canPacket));
                //_canQueryData = ParseCanReceiveToStringData(canPacket);
                //_isCanQueryDataUpdate = true;
                //LcuCommunication.t_lcu_data_can_udp_rx_packet canRxPacket = TypeConvert.BytesToStruct<LcuCommunication.t_lcu_data_can_udp_rx_packet>(data);
                _canUDPRx = TypeConvert.BytesToStruct<LcuCommunication.t_lcu_data_can_udp_rx_packet>(data);
                //_isCanQueryDataUpdate = true;
                break;
            case LcuCommunication.LCU_SVC_MODE:
                mode = _recvData.uValue;
                break;
            case LcuCommunication.LCU_SVC_MONITORING_DATA:
                this.isConnect = true;
                monitoringData = TypeConvert.BytesToStruct<LcuCommunication.t_lcu_data_monitor>(data);
                if (monitoringData.nEmergency > 0)
                {
                    isEmergencyStop = true;
                }
                else
                {
                    isEmergencyStop = false;
                }
                //Debug.WriteLine(monitoringData.nAutoControl);
                break;
            case LcuCommunication.LCU_SVC_POSDIFF_MONITOR:
                if (_recvData.uValue == LcuCommunication.LCU_VAL_POSDIFF_MONITOR_ENABLE)
                {
                    isPosDiffMonitor = true;
                }
                else
                {
                    isPosDiffMonitor = false;
                }
                break;
            case LcuCommunication.LCU_SVC_SET_CONTROL:
                masterDevice = _recvData.uValue;
                break;
            case LcuCommunication.LCU_SVC_SV_STATE:
                //.svState = _recvData.uData.svState;
                break;
            case LcuCommunication.LCU_SVC_LIMIT_CHANGE:
                if (_recvData.uValue == LcuCommunication.LCU_VAL_LIMIT_SW_STATE
                    || _recvData.uValue == LcuCommunication.LCU_VAL_LIMIT_HW_STATE)
                {
                    _jointLimit = TypeConvert.BytesToStruct<LcuCommunication.t_lcu_data_limit>(data);
                }
                break;
            case LcuCommunication.LCU_SVC_SENDER_CHANGE:
                if (_recvData.uValue == LcuCommunication.LCU_VAL_SENDER_STATE)
                    lcuSender = TypeConvert.BytesToStruct<LcuCommunication.t_lcu_data_sender>(data).nSender;
                break;
        }
    }

    //설명: LCU 에러코드 파싱
    public string ParseErrorToString(int errorCode, bool isAddDate)
    {
        string msg = "No Error";

        switch ((UInt16)errorCode)
        {
            case LcuManipulatorError.ERR_FROM_LCU: msg = "FROM_LCU"; break;
            case LcuManipulatorError.ERR_CRITICAL: msg = "CRITICAL"; break;
            case LcuManipulatorError.ERR_WARNING: msg = "WARNING"; break;

            case LcuManipulatorError.ERR_LCU_COMM_VER_DIFFERENCE: msg = "LCU_COMM_VER_DIFFERENCE"; break;
            case LcuManipulatorError.ERR_LCU_NO_START: msg = "LCU_NO_START"; break;
            case LcuManipulatorError.ERR_LCU_OVER_KEEPTIME: msg = "LCU_OVER_KEEPTIME"; break;
            case LcuManipulatorError.ERR_LCU_OVER_TEMP: msg = "LCU_OVER_TEMP"; break;

            case LcuManipulatorError.ERR_LCU_ANALOG_PORT_UNDEFINED: msg = "LCU_ANALOG_PORT_UNDEFINED"; break;
            case LcuManipulatorError.ERR_LCU_ANALOG_PORT_DENIED: msg = "LCU_ANALOG_PORT_DENIED"; break;
            case LcuManipulatorError.ERR_LCU_ANALOG_CMD_OVERRAGE: msg = "LCU_ANALOG_CMD_OVERRAGE"; break;
            case LcuManipulatorError.ERR_LCU_DIO_PORT_UNDEFINED: msg = "LCU_DIO_PORT_UNDEFINED"; break;
            case LcuManipulatorError.ERR_LCU_DIO_PORT_WRITE_DENIED: msg = "LCU_DIO_PORT_WRITE_DENIED"; break;
            case LcuManipulatorError.ERR_LCU_DIO_WRONG_VALUE: msg = "LCU_DIO_WRONG_VALUE"; break;

            case LcuManipulatorError.ERR_LCU_CAN_ID_EXCEEDED: msg = "LCU_CAN_ID_EXCEEDED"; break;
            case LcuManipulatorError.ERR_LCU_CAN_DLC_EXCEEDED: msg = "LCU_CAN_DLC_EXCEEDED"; break;

            case LcuManipulatorError.ERR_LCU_PACKET_SIZE: msg = "LCU_PACKET_SIZE"; break;
            case LcuManipulatorError.ERR_LCU_CHECKSUM: msg = "LCU_CHECKSUM"; break;
            case LcuManipulatorError.ERR_LCU_SENDER: msg = "LCU_SENDER"; break;
            case LcuManipulatorError.ERR_LCU_RECEIVER: msg = "LCU_RECEIVER"; break;
            case LcuManipulatorError.ERR_LCU_UNKOWN_CODE: msg = "LCU_UNKOWN_CODE"; break;
            case LcuManipulatorError.ERR_LCU_UNDEFINED_VALUE: msg = "LCU_UNDEFINED_VALUE"; break;
            case LcuManipulatorError.ERR_LCU_DATA_SIZE: msg = "LCU_DATA_SIZE"; break;

            case LcuManipulatorError.ERR_LCU_MOTION_NOT_STARTED: msg = "LCU_MOTION_NOT_STARTED"; break;
            case LcuManipulatorError.ERR_LCU_MODE_UNDEFINED: msg = "LCU_MODE_UNDEFINED"; break;
            case LcuManipulatorError.ERR_LCU_MODE_UNMATCHED: msg = "LCU_MODE_UNMATCHED"; break;

            case LcuManipulatorError.ERR_LCU_QLIM_LOW_AXIS: msg = "LCU_QLIM_LOW_AXIS"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_LOW_AXIS1: msg = "LCU_QLIM_LOW_AXIS1"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_LOW_AXIS2: msg = "LCU_QLIM_LOW_AXIS2"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_LOW_AXIS3: msg = "LCU_QLIM_LOW_AXIS3"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_LOW_AXIS4: msg = "LCU_QLIM_LOW_AXIS4"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_LOW_AXIS5: msg = "LCU_QLIM_LOW_AXIS5"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_LOW_AXIS6: msg = "LCU_QLIM_LOW_AXIS6"; break;

            case LcuManipulatorError.ERR_LCU_QLIM_HIGH_AXIS: msg = "LCU_QLIM_HIGH_AXIS"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_HIGH_AXIS1: msg = "LCU_QLIM_HIGH_AXIS1"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_HIGH_AXIS2: msg = "LCU_QLIM_HIGH_AXIS2"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_HIGH_AXIS3: msg = "LCU_QLIM_HIGH_AXIS3"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_HIGH_AXIS4: msg = "LCU_QLIM_HIGH_AXIS4"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_HIGH_AXIS5: msg = "LCU_QLIM_HIGH_AXIS5"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_HIGH_AXIS6: msg = "LCU_QLIM_HIGH_AXIS6"; break;

            case LcuManipulatorError.ERR_LCU_POS_DIFF_AXIS: msg = "LCU_POS_DIFF_AXIS"; break;
            case LcuManipulatorError.ERR_LCU_POS_DIFF_AXIS1: msg = "LCU_POS_DIFF_AXIS1"; break;
            case LcuManipulatorError.ERR_LCU_POS_DIFF_AXIS2: msg = "LCU_POS_DIFF_AXIS2"; break;
            case LcuManipulatorError.ERR_LCU_POS_DIFF_AXIS3: msg = "LCU_POS_DIFF_AXIS3"; break;
            case LcuManipulatorError.ERR_LCU_POS_DIFF_AXIS4: msg = "LCU_POS_DIFF_AXIS4"; break;
            case LcuManipulatorError.ERR_LCU_POS_DIFF_AXIS5: msg = "LCU_POS_DIFF_AXIS5"; break;
            case LcuManipulatorError.ERR_LCU_POS_DIFF_AXIS6: msg = "LCU_POS_DIFF_AXIS6"; break;

            case LcuManipulatorError.ERR_LCU_OVER_TEMP_AXIS: msg = "LCU_OVER_TEMP_AXIS"; break;
            case LcuManipulatorError.ERR_LCU_OVER_TEMP_AXIS1: msg = "LCU_OVER_TEMP_AXIS1"; break;
            case LcuManipulatorError.ERR_LCU_OVER_TEMP_AXIS2: msg = "LCU_OVER_TEMP_AXIS2"; break;
            case LcuManipulatorError.ERR_LCU_OVER_TEMP_AXIS3: msg = "LCU_OVER_TEMP_AXIS3"; break;
            case LcuManipulatorError.ERR_LCU_OVER_TEMP_AXIS4: msg = "LCU_OVER_TEMP_AXIS4"; break;
            case LcuManipulatorError.ERR_LCU_OVER_TEMP_AXIS5: msg = "LCU_OVER_TEMP_AXIS5"; break;
            case LcuManipulatorError.ERR_LCU_OVER_TEMP_AXIS6: msg = "LCU_OVER_TEMP_AXIS6"; break;

            case LcuManipulatorError.ERR_LCU_QLIM_SW_LOW_AXIS: msg = "LCU_QLIM_SW_LOW_AXIS"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_SW_LOW_AXIS1: msg = "LCU_QLIM_SW_LOW_AXIS1"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_SW_LOW_AXIS2: msg = "LCU_QLIM_SW_LOW_AXIS2"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_SW_LOW_AXIS3: msg = "LCU_QLIM_SW_LOW_AXIS3"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_SW_LOW_AXIS4: msg = "LCU_QLIM_SW_LOW_AXIS4"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_SW_LOW_AXIS5: msg = "LCU_QLIM_SW_LOW_AXIS5"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_SW_LOW_AXIS6: msg = "LCU_QLIM_SW_LOW_AXIS6"; break;

            case LcuManipulatorError.ERR_LCU_QLIM_SW_HIGH_AXIS: msg = "LCU_QLIM_SW_HIGH_AXIS"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_SW_HIGH_AXIS1: msg = "LCU_QLIM_SW_HIGH_AXIS1"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_SW_HIGH_AXIS2: msg = "LCU_QLIM_SW_HIGH_AXIS2"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_SW_HIGH_AXIS3: msg = "LCU_QLIM_SW_HIGH_AXIS3"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_SW_HIGH_AXIS4: msg = "LCU_QLIM_SW_HIGH_AXIS4"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_SW_HIGH_AXIS5: msg = "LCU_QLIM_SW_HIGH_AXIS5"; break;
            case LcuManipulatorError.ERR_LCU_QLIM_SW_HIGH_AXIS6: msg = "LCU_QLIM_SW_HIGH_AXIS6"; break;
        }
        if (isAddDate)
            return "Time: " + DateTime.Now.ToString("MM/dd/yy HH:mm:ss") + " Error: " + msg + "(0x" + errorCode.ToString("X") + ")";
        else
            return msg + "(0x" + errorCode.ToString("X") + ")";
    }

    //설명: LCU로 index, type, 데이터 등을 선택하여 CAN데이터를 보냄.(float)
    public void SendCanData(int hacID, byte index, byte subIndex, byte type, float data)
    {
        LcuCommunication.t_lcu_data_can_packet packet = ParseCanSendData(hacID, index, subIndex, type);

        if (type == 0) //Set
        {
            packet.uData[3] = LcuCan.LCU_CAN_SET_FLOAT;
            packet.uData[4] = BitConverter.GetBytes(data)[0];
            packet.uData[5] = BitConverter.GetBytes(data)[1];
            packet.uData[6] = BitConverter.GetBytes(data)[2];
            packet.uData[7] = BitConverter.GetBytes(data)[3];
        }
        else if (type == 1) //Get
        {
            packet.uData[3] = LcuCan.LCU_CAN_GET_FLOAT;
        }

        SendCanPacket(packet);
    }

    //설명: LCU로 index, type, 데이터 등을 선택하여 CAN데이터를 보냄.(short)
    public void SendCanData(int hacID, byte index, byte subIndex, byte type, ushort data)
    {
        LcuCommunication.t_lcu_data_can_packet packet = ParseCanSendData(hacID, index, subIndex, type);

        if (type == 0) //Set
        {
            packet.uData[3] = LcuCan.LCU_CAN_SET_UINT;
            packet.uData[4] = BitConverter.GetBytes(data)[0];
            packet.uData[5] = BitConverter.GetBytes(data)[1];
            packet.uData[6] = 0x00;
            packet.uData[7] = 0x00;
        }
        else if (type == 1) //Get
        {
            packet.uData[3] = LcuCan.LCU_CAN_GET_UINT;
        }

        SendCanPacket(packet);
    }

    //설명: LCU로 index, type, 데이터 등을 입력하여 CAN 데이터를 보냄.
    public void SendCanRawData(int hacID, byte indexLow, byte indexHigh, byte subIndex, byte type, ushort data)
    {
        LcuCommunication.t_lcu_data_can_packet packet;
        packet = new LcuCommunication.t_lcu_data_can_packet();

        packet.nId = (ushort)(LcuCan.LCU_CAN_ID_SEND | (ushort)hacID);
        packet.nDlc = LcuCan.LCU_CAN_DLC;
        packet.uData = new byte[8];

        packet.uData[0] = indexHigh;
        packet.uData[1] = indexLow;
        packet.uData[2] = subIndex;

        if (type == 0) //Set
        {
            packet.uData[3] = LcuCan.LCU_CAN_SET_UINT;
            packet.uData[4] = BitConverter.GetBytes(data)[0];
            packet.uData[5] = BitConverter.GetBytes(data)[1];
            packet.uData[6] = 0x00;
            packet.uData[7] = 0x00;
        }
        else if (type == 1) //Get
        {
            packet.uData[3] = LcuCan.LCU_CAN_GET_UINT;
        }

        SendCanPacket(packet);
    }

    //설명: string 데이터를 char형으로 변환
    private byte ExtractCanSerialToCharMode(string serialCommand)
    {
        return (byte)System.Convert.ToChar(serialCommand[0]);
    }

    //설명: string 데이터를 float으로 변환
    private float ExtractCanSerialToData(string serialCommand)
    {
        string str = serialCommand.Remove(0, 1);
        float value = 0;

        if (!float.TryParse(str, out value))
        {
            value = 0;
        }

        return value;
    }

    //설명: CAN 통신으로 LCU에 연결된 HAC로 Serial 창에 입력하는 명령과 동일한 명령을 보냄(eX) O0.1, O-2) 
    public void SendCanSerialData(int hacID, string serialCommand)
    {
        LcuCommunication.t_lcu_data_can_packet packet;
        packet = new LcuCommunication.t_lcu_data_can_packet();
        byte[] indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_SERIAL_CMD);
        byte charMode = ExtractCanSerialToCharMode(serialCommand);
        float data = ExtractCanSerialToData(serialCommand);

        packet.nId = (ushort)(LcuCan.LCU_CAN_ID_SEND | (ushort)hacID);
        packet.nDlc = LcuCan.LCU_CAN_DLC;
        packet.uData = new byte[8];

        packet.uData[0] = indexBytes[0];
        packet.uData[1] = indexBytes[1];
        packet.uData[2] = charMode;

        //Set Only
        packet.uData[3] = LcuCan.LCU_CAN_SET_FLOAT;
        packet.uData[4] = BitConverter.GetBytes(data)[0];
        packet.uData[5] = BitConverter.GetBytes(data)[1];
        packet.uData[6] = BitConverter.GetBytes(data)[2];
        packet.uData[7] = BitConverter.GetBytes(data)[3];

        Debug.WriteLine("Input Command: " + serialCommand);
        Debug.WriteLine("charMode: " + (char)charMode);
        Debug.WriteLine("data: " + data);

        SendCanPacket(packet);
    }

    //설명: CAN 통신으로 LCU에 연결된 3번축 Force 명령 전달
    public void SendCanSerialDataUDPForce(string serialCommand)
    {
        LcuCommunication.t_lcu_data_can_udp_packet _canUDPpacket;
        _canUDPpacket = new LcuCommunication.t_lcu_data_can_udp_packet();
        byte[] indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_SERIAL_CMD);
        byte charMode = ExtractCanSerialToCharMode(serialCommand);
        float data = ExtractCanSerialToData(serialCommand);

        _canUDPpacket.subindex = (ushort)charMode;
        _canUDPpacket.Data = data;
        //Debug.WriteLine("Input Command: " + serialCommand);
        Debug.WriteLine("charMode: " + charMode);
        Debug.WriteLine("data: " + data);

        SendCanUDPForcePacket(_canUDPpacket);
        //SendCanPacket(packet);
    }


    //설명: CAN 통신으로 LCU에 연결된 HAC로 Serial 창에 입력하는 명령과 동일한 명령을 보냄(eX) O0.1, O-2) 
    public void SendCanSerialDataUDP(int hacID, string serialCommand)
    {
        LcuCommunication.t_lcu_data_can_udp_packet _canUDPpacket;
        _canUDPpacket = new LcuCommunication.t_lcu_data_can_udp_packet();
        byte[] indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_SERIAL_CMD);
        byte charMode = ExtractCanSerialToCharMode(serialCommand);
        float data = ExtractCanSerialToData(serialCommand);

        //_canUDPpacket.Id = (ushort)(LcuCan.LCU_CAN_ID_SEND | (ushort)hacID);
        ////_canUDPpacket.nDlc = LcuCan.LCU_CAN_DLC;
        //_canUDPpacket.uData = new byte[8];
        ////_canUDPpacket.subindex = Encoding.Default.GetChars(charMode);
        //_canUDPpacket.Data = data;

        //_canUDPpacket.uData[0] = indexBytes[0];
        //_canUDPpacket.uData[1] = indexBytes[1];
        //_canUDPpacket.uData[2] = charMode;

        //////Set Only
        //_canUDPpacket.uData[3] = LcuCan.LCU_CAN_SET_FLOAT;
        //_canUDPpacket.uData[4] = BitConverter.GetBytes(data)[0];
        //_canUDPpacket.uData[5] = BitConverter.GetBytes(data)[1];
        //_canUDPpacket.uData[6] = BitConverter.GetBytes(data)[2];
        //_canUDPpacket.uData[7] = BitConverter.GetBytes(data)[3];

        //Debug.WriteLine("Input Command: " + serialCommand);
        //Debug.WriteLine("charMode: " + (char)charMode);
        //Debug.WriteLine("data: " + data);
        _canUDPpacket.Id = (ushort)(LcuCan.LCU_CAN_ID_SEND | (ushort)hacID);
        _canUDPpacket.subindex = (ushort)charMode;
        _canUDPpacket.Data = data;
        Debug.WriteLine("Input Command: " + serialCommand);
        Debug.WriteLine("charMode: " + charMode);
        Debug.WriteLine("data: " + data);

        SendCanUDPPacket(_canUDPpacket);
        //SendCanPacket(packet);
    }

    //설명: CAN 전송 패킷들의 index를 읽어 데이터를 파싱. 
    LcuCommunication.t_lcu_data_can_packet ParseCanSendData(int hacID, byte index, byte subIndex, byte type)
    {
        LcuCommunication.t_lcu_data_can_packet packet;
        packet = new LcuCommunication.t_lcu_data_can_packet();

        packet.nId = (ushort)(LcuCan.LCU_CAN_ID_SEND | (ushort)hacID);
        packet.nDlc = LcuCan.LCU_CAN_DLC;
        packet.uData = new byte[8];

        byte[] indexBytes = new byte[2];
        byte subIndexByte = 0x00;

        switch (subIndex)
        {
            case 0: break;
            case 1: break;
            case 2: break;
        }

        switch (index)
        {
            case 0: //Write Gain
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_WRITE_GAIN);
                break;
            case 1: //Sign of Output(w/r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_OUTPUT_SIGN);
                switch (subIndex)
                {
                    case 0: subIndexByte = ++subIndex; break; //position
                    case 1: subIndexByte = ++subIndex; break; //torque
                }
                break;
            case 2: //Servo v/v out(w/r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_OUTPUT_SV);
                break;
            case 3://Start Mode(w/r)*
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_START_MODE);
                switch (subIndex)
                {
                    case LcuCan.HAC_CONTROL_MODE_OPEN_CAN: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_OPEN_ANALOG: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_POSITIONCONTROL_CAN: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_POSITIONCONTROL_ANALOG: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_FORCECONTROL_CAN: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_FORCECONTROL_ANALOG: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_COMPLIANCECONTROL_CAN: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_COMPLIANCECONTROL_ANALOG: subIndexByte = subIndex; break;
                }
                break;
            case 4: //Position Gain(w/r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_POSITION_GAIN);
                switch (subIndex)
                {
                    case 0: subIndexByte = ++subIndex; break; //proportional
                    case 1: subIndexByte = ++subIndex; break; //integral
                    case 2: subIndexByte = ++subIndex; break; //differential
                }
                break;
            case 5: //Torque Gain(w/r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_TORQUE_GAIN);
                switch (subIndex)
                {
                    case 0: subIndexByte = ++subIndex; break; //proportional
                    case 1: subIndexByte = ++subIndex; break; //integral
                    case 2: subIndexByte = ++subIndex; break; //differential
                }
                break;
            case 6: //Compliance Gain(w/r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_COMPLIANCE_GAIN);
                switch (subIndex)
                {
                    case 0: subIndexByte = ++subIndex; break; //stiffness
                    case 1: subIndexByte = ++subIndex; break; //friction
                    case 2: subIndexByte = ++subIndex; break; //transition deflection
                    case 3: subIndexByte = ++subIndex; break; //offset
                }
                break;
            case 7: //Velocity Gain(w/r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_VELOCITY_GAIN);
                switch (subIndex)
                {
                    case 0: subIndexByte = ++subIndex; break; //proportional
                    case 1: subIndexByte = ++subIndex; break; //integral
                    case 2: subIndexByte = ++subIndex; break; //differential
                }
                break;
            case 8://Error Sum Clear
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_ESUM_CLEAR);
                break;
            case 9://Set Min & Max Position(w/r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_SET_MIN_MAX_POS);
                switch (subIndex)
                {
                    case 0: subIndexByte = ++subIndex; break; //max degree
                    case 1: subIndexByte = ++subIndex; break; //min degree
                }
                break;
            case 10://Temperature of Board(r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_BOARD_TEMPERATURE);
                break;
            case 11://Temperature of Thermocouple(r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_THERMOCOUPLER_TEMPERATURE);
                break;
            case 12: //Analog Read(r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_ANALOG_READ);
                switch (subIndex)
                {
                    case 0: subIndexByte = ++subIndex; break; //port1
                    case 1: subIndexByte = ++subIndex; break; //port2
                    case 2: subIndexByte = ++subIndex; break; //port3
                    case 3: subIndexByte = ++subIndex; break; //port4
                }
                break;
            case 13://Stop
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_STOP);
                break;
            case 14://Control Mode(w/r)*
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_CONTROL_MODE);
                switch (subIndex)
                {
                    case LcuCan.HAC_CONTROL_MODE_OPEN_CAN: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_OPEN_ANALOG: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_POSITIONCONTROL_CAN: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_POSITIONCONTROL_ANALOG: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_FORCECONTROL_CAN: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_FORCECONTROL_ANALOG: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_COMPLIANCECONTROL_CAN: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_COMPLIANCECONTROL_ANALOG: subIndexByte = subIndex; break;

                    case LcuCan.HAC_CONTROL_MODE_ENCODER_ERROR_STOP: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_AUTO_SERVO_VIBR: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_AUTO_CALIBRATION: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_AUTO_LOOP_A: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_AUTO_LOOP_B: subIndexByte = subIndex; break;
                    case LcuCan.HAC_CONTROL_MODE_AUTO_LOOP_C: subIndexByte = subIndex; break;
                }
                break;
            case 15://Target Position(w/r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_TARGET_POSITION);
                break;
            case 16://Actual Position(r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_ACTUAL_POSITION);
                break;
            case 17://Target Velocity(w/r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_TARGET_VELOCITY);
                break;
            case 18://Actual Velocity(r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_ACTUAL_VELOCITY);
                break;
            case 19://Target Torque(w/r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_TARGET_TORQUE);
                break;
            case 20://Actual Torque(r)
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_ACTUAL_TORQUE);
                break;
            case 21://Encoder Raw Angle
                indexBytes = BitConverter.GetBytes(LcuCan.LCU_CAN_INDEX_ENCODER_RAW_ANGLE);
                break;
        }

        packet.uData[0] = indexBytes[0];
        packet.uData[1] = indexBytes[1];
        packet.uData[2] = subIndexByte;

        return packet;
    }

    //설명: CAN 수신 패킷들을 파싱 후 String 으로 리턴.
    string ParseCanReceiveToString(LcuCommunication.t_lcu_data_can_packet canData)
    {
        string textHacId = "";
        string textIndex = "";
        string textSubIndex = "";
        string textData = "";

        UInt16 index = BitConverter.ToUInt16(new byte[] { canData.uData[0], canData.uData[1] }, 0);
        byte subIndex = canData.uData[2];

        int id = canData.nId ^ LcuCan.LCU_CAN_ID_RECV;
        textHacId = id.ToString("0");

        float fData = 0.0f;
        int nData = 0;

        if (canData.uData[3] == LcuCan.LCU_CAN_SET_FLOAT)
        {
            fData = BitConverter.ToSingle(new byte[] { canData.uData[4], canData.uData[5], canData.uData[6], canData.uData[7] }, 0);
            textData = fData.ToString("0.0000");
        }
        else
        {
            nData = BitConverter.ToUInt16(new byte[] { canData.uData[4], canData.uData[5] }, 0);
            textData = nData.ToString("0");
        }

        switch (index)
        {
            case LcuCan.LCU_CAN_INDEX_WRITE_GAIN: //Write Gain
                textIndex = "Write Gain";
                break;
            case LcuCan.LCU_CAN_INDEX_OUTPUT_SIGN: //Sign of Output(w/r)
                textIndex = "Sign of Output";
                switch (subIndex - 1)
                {
                    case 0: textSubIndex = "Position"; break; //position
                    case 1: textSubIndex = "Torque"; break; //torque
                }
                break;
            case LcuCan.LCU_CAN_INDEX_OUTPUT_SV: //Servo v/v out(w/r)
                textIndex = "Servo v/v out";
                break;
            case LcuCan.LCU_CAN_INDEX_START_MODE://Start Mode(w/r)*
                textIndex = "Start Mode";
                switch (subIndex)
                {
                    case LcuCan.HAC_CONTROL_MODE_OPEN_CAN: textSubIndex = "Open can"; break;
                    case LcuCan.HAC_CONTROL_MODE_OPEN_ANALOG: textSubIndex = "Open analog"; break;
                    case LcuCan.HAC_CONTROL_MODE_POSITIONCONTROL_CAN: textSubIndex = "Position can"; break;
                    case LcuCan.HAC_CONTROL_MODE_POSITIONCONTROL_ANALOG: textSubIndex = "Position analog"; break;
                    case LcuCan.HAC_CONTROL_MODE_FORCECONTROL_CAN: textSubIndex = "Force can"; break;
                    case LcuCan.HAC_CONTROL_MODE_FORCECONTROL_ANALOG: textSubIndex = "Force analog"; break;
                    case LcuCan.HAC_CONTROL_MODE_COMPLIANCECONTROL_CAN: textSubIndex = "Compliance can"; break;
                    case LcuCan.HAC_CONTROL_MODE_COMPLIANCECONTROL_ANALOG: textSubIndex = "Compliance analog"; break;
                }
                break;
            case LcuCan.LCU_CAN_INDEX_POSITION_GAIN: //Position Gain(w/r)
                textIndex = "Position Gain";
                switch (subIndex - 1)
                {
                    case 0: textSubIndex = "Proportional"; break; //proportional
                    case 1: textSubIndex = "Integral"; break; //integral
                    case 2: textSubIndex = "Differential"; break; //differential
                }
                break;
            case LcuCan.LCU_CAN_INDEX_TORQUE_GAIN: //Torque Gain(w/r)
                textIndex = "Torque Gain";
                switch (subIndex - 1)
                {
                    case 0: textSubIndex = "Proportional"; break; //proportional
                    case 1: textSubIndex = "Integral"; break; //integral
                    case 2: textSubIndex = "Differential"; break; //differential
                }
                break;
            case LcuCan.LCU_CAN_INDEX_COMPLIANCE_GAIN: //Compliance Gain(w/r)
                textIndex = "Compliance Gain";
                switch (subIndex - 1)
                {
                    case 0: textSubIndex = "Stiffness"; break; //stiffness
                    case 1: textSubIndex = "Friction"; break; //friction
                    case 2: textSubIndex = "Transition deflection"; break; //transition deflection
                    case 3: textSubIndex = "Offset"; break; //offset
                }
                break;
            case LcuCan.LCU_CAN_INDEX_VELOCITY_GAIN: //Velocity Gain(w/r)
                textIndex = "Velocity Gain";
                switch (subIndex - 1)
                {
                    case 0: textSubIndex = "Proportional"; break; //proportional
                    case 1: textSubIndex = "Integral"; break; //integral
                    case 2: textSubIndex = "Differential"; break; //differential
                }
                break;
            case LcuCan.LCU_CAN_INDEX_ESUM_CLEAR://Error Sum Clear
                textIndex = "Error Sum Clear";
                break;
            case LcuCan.LCU_CAN_INDEX_SET_MIN_MAX_POS://Set Min & Max Position(w/r)
                textIndex = "Set Min & Max Position";
                switch (subIndex - 1)
                {
                    case 0: textSubIndex = "Max degree"; break; //max degree
                    case 1: textSubIndex = "Min degree"; break; //min degree
                }
                break;
            case LcuCan.LCU_CAN_INDEX_BOARD_TEMPERATURE://Temperature of Board(r)
                textIndex = "Temperature of Board";
                break;
            case LcuCan.LCU_CAN_INDEX_THERMOCOUPLER_TEMPERATURE://Temperature of Thermocouple(r)
                textIndex = "Temperature of Thermocoupler";
                break;
            case LcuCan.LCU_CAN_INDEX_ANALOG_READ: //Analog Read(r)
                textIndex = "Analog Read";
                switch (subIndex - 1)
                {
                    case 0: textSubIndex = "Port1"; break; //port1
                    case 1: textSubIndex = "Port2"; break; //port2
                    case 2: textSubIndex = "Port3"; break; //port3
                    case 3: textSubIndex = "Port4"; break; //port4
                }
                break;
            case LcuCan.LCU_CAN_INDEX_STOP://Stop
                textIndex = "Stop";
                break;
            case LcuCan.LCU_CAN_INDEX_CONTROL_MODE://Control Mode(w/r)*
                textIndex = "Control Mode";
                switch (nData)
                {
                    case LcuCan.HAC_CONTROL_MODE_OPEN_CAN: textSubIndex = "Open can"; break;
                    case LcuCan.HAC_CONTROL_MODE_OPEN_ANALOG: textSubIndex = "Open analog"; break;
                    case LcuCan.HAC_CONTROL_MODE_POSITIONCONTROL_CAN: textSubIndex = "Position can"; break;
                    case LcuCan.HAC_CONTROL_MODE_POSITIONCONTROL_ANALOG: textSubIndex = "Position analog"; break;
                    case LcuCan.HAC_CONTROL_MODE_FORCECONTROL_CAN: textSubIndex = "Force can"; break;
                    case LcuCan.HAC_CONTROL_MODE_FORCECONTROL_ANALOG: textSubIndex = "Force analog"; break;
                    case LcuCan.HAC_CONTROL_MODE_COMPLIANCECONTROL_CAN: textSubIndex = "Compliance can"; break;
                    case LcuCan.HAC_CONTROL_MODE_COMPLIANCECONTROL_ANALOG: textSubIndex = "Compliance analog"; break;

                    case LcuCan.HAC_CONTROL_MODE_ENCODER_ERROR_STOP: textSubIndex = "Encoder error stop"; break;
                    case LcuCan.HAC_CONTROL_MODE_AUTO_SERVO_VIBR: textSubIndex = "Auto SV vibration"; break;
                    case LcuCan.HAC_CONTROL_MODE_AUTO_CALIBRATION: textSubIndex = "Auto calibration"; break;
                    case LcuCan.HAC_CONTROL_MODE_AUTO_LOOP_A: textSubIndex = "Auto Loop A"; break;
                    case LcuCan.HAC_CONTROL_MODE_AUTO_LOOP_B: textSubIndex = "Auto Loop B"; break;
                    case LcuCan.HAC_CONTROL_MODE_AUTO_LOOP_C: textSubIndex = "Auto Loop C"; break;
                }
                break;
            case LcuCan.LCU_CAN_INDEX_TARGET_POSITION://Target Position(w/r)
                textIndex = "Target Position";
                break;
            case LcuCan.LCU_CAN_INDEX_ACTUAL_POSITION://Actual Position(r)
                textIndex = "Actual Position";
                break;
            case LcuCan.LCU_CAN_INDEX_TARGET_VELOCITY://Target Velocity(w/r)
                textIndex = "Target Velocity";
                break;
            case LcuCan.LCU_CAN_INDEX_ACTUAL_VELOCITY://Actual Velocity(r)
                textIndex = "Actual Velocity";
                break;
            case LcuCan.LCU_CAN_INDEX_TARGET_TORQUE://Target Torque(w/r)
                textIndex = "Target Torque";
                break;
            case LcuCan.LCU_CAN_INDEX_ACTUAL_TORQUE://Actual Torque(r)
                textIndex = "Actual Torque";
                break;
            case LcuCan.LCU_CAN_INDEX_ENCODER_RAW_ANGLE://Actual Torque(r)
                textIndex = "Encoder Raw Angle";
                break;
        }


        textIndex += "(0x" + string.Format("{0:X}", index) + ")";
        textSubIndex += "(0x" + string.Format("{0:X}", subIndex) + ")";

        return "Time: " + DateTime.Now.ToString("MM/dd/yy HH:mm:ss") + "  HAC ID: " + textHacId + " NAME: " + textIndex + " SUB: " + textSubIndex + " DATA: " + textData;
    }

    //설명: CAN 수신 패킷 중 데이터만 string으로 리턴.
    string ParseCanReceiveToStringData(LcuCommunication.t_lcu_data_can_packet canData)
    {
        string textData = "";

        if (canData.uData[3] == LcuCan.LCU_CAN_SET_FLOAT)
        {
            textData = BitConverter.ToSingle(new byte[] { canData.uData[4], canData.uData[5], canData.uData[6], canData.uData[7] }, 0).ToString("0.0000");
        }
        else
        {
            textData = BitConverter.ToUInt16(new byte[] { canData.uData[4], canData.uData[5] }, 0).ToString("0");
        }

        return textData;
    }

    //설명: 마지막으로 수신된 CAN 패킷의 데이터를 리턴. 
    public string GetLastCanDataText()
    {
        _isCanQueryDataUpdate = false;
        return _canQueryData;
    }

    //설명: CAN 패킷의 수신여부를 확인.
    public bool GetIsCanDataUpdate()
    {
        return _isCanQueryDataUpdate;
    }

    //설명: 마지막 에러 코드를 string으로 리턴.
    public string GetLastErrorText()
    {
        if (_ErrorMessageList.Count > 0)
            return _ErrorTextList[_ErrorTextList.Count - 1];
        else
            return "";
    }

    //설명: LCU의 에러의 갯수를 리턴.
    public int GetErrorTextCount()
    {
        return _ErrorTextList.Count;
    }

    //설명: 마지막 에러 전체(시간, 에러명 등 포함)를 string으로 리턴.
    public string GetLastErrorMessage()
    {
        if (_ErrorMessageList.Count > 0)
            return _ErrorMessageList[_ErrorMessageList.Count - 1];
        else
            return "";
    }

    //설명: 기록된 에러 리셋(초기화).
    public void ResetErrorList()
    {
        _ErrorMessageList.Clear();
        _ErrorTextList.Clear();
    }

    //설명: json으로 저장된 IP를 읽어옴
    public string GetTargetSocketIp(int index)
    {
        return _ipTargetSocket.IP[index];
    }

    //설명: json으로 저장된 port로 읽어옴
    public int GetTargetSocketPort(int index)
    {
        return _ipTargetSocket.port[index];
    }

    //설명: ip 및 port를 json으로 저장.
    public void SaveTargetSocket(int index, string ip, int port)
    {
        _ipTargetSocket.IP[index] = ip;
        _ipTargetSocket.port[index] = port;

        JsonFileIO.Save(_ipTargetSocket, GetFileNameWith_deviceNumber(FILE_NAME_TARGET_SOCKET, _deviceNumber));
    }

    //설명: ip 및 port를 초기화하여 json으로 저장.
    private void SaveTargetSocketInit()
    {
        _ipTargetSocket = new JsonTargetSocket();
        _ipTargetSocket.name = new List<string>();
        _ipTargetSocket.IP = new List<string>();
        _ipTargetSocket.port = new List<int>();

        _ipTargetSocket.name.Add("LCU");
        _ipTargetSocket.IP.Add("192.168.000.10" + _deviceNumber.ToString());
        _ipTargetSocket.port.Add(4100 + _deviceNumber);

        _ipTargetSocket.name.Add("RM");
        _ipTargetSocket.IP.Add("192.168.000.100");
        _ipTargetSocket.port.Add(4010);

        _ipTargetSocket.name.Add("PC1");
        _ipTargetSocket.IP.Add("192.168.000.21");
        _ipTargetSocket.port.Add(4021);
        _ipTargetSocket.name.Add("PC2");
        _ipTargetSocket.IP.Add("192.168.000.22");
        _ipTargetSocket.port.Add(4022);

        _ipTargetSocket.name.Add("MULTI");
        _ipTargetSocket.IP.Add("239.255.000." + _deviceNumber.ToString());
        _ipTargetSocket.port.Add(4000 + _deviceNumber);

        JsonFileIO.Save(_ipTargetSocket, GetFileNameWith_deviceNumber(FILE_NAME_TARGET_SOCKET, _deviceNumber));
    }

    ////설명: LCU에 메시지를 전송한 후 호출. 일정시간동안 응답을 대기한 후 성공/실패 Action 함수 실행.
    ////기능: 코루틴
    //public IEnumerator WaitAckAndDoWork(byte code, int timeSec, Action workOk, Action workFail)
    //{
    //    bool rst = false;
    //    if (timeSec < 1) timeSec = 1;
    //    for (int i = 0; i < timeSec * 10; i++)
    //    {
    //        if (!this.GetIsWaitAck(code))
    //        {
    //            Debug.WriteLine("ACK DONE!");
    //            workOk();
    //            rst = true;
    //            break;
    //        }
    //        yield return new WaitForSeconds(0.1f);
    //    }

    //    if (!rst)
    //    {
    //        Debug.WriteLine("ACK FAIL!");
    //        this.SetIsWaitAck(code, false);
    //        workFail();
    //    }
    //}

    //설명: IP 및 Port 정보를 포함하는 json 파일 경로
    private string GetFileNameWith_deviceNumber(string name, int _deviceNumber)
    {
        return FILE_PATH_SETTING + name + "(LCU" + _deviceNumber.ToString() + ")" + FILE_EXTENSION;
    }

    //설명: LCU를 LCU 없이 가상으로 시뮬레이션하는 기능을 켬.
    public void SetJointSimulation(bool isOn)
    {
        _lcuJointSimulation.isOnJointJoystick = isOn;

        if (!isOn)
        {
            for (int i = 0; i < KnrHeader.LcuCommon.ROBOT_AXIS_CNT_MAX; i++)
            {
                monitoringData.fJointPos[i] = 0;
                _lcuJointSimulation.targetMaxVelocity[i] = 0;
            }
        }
    }

    //설명: LCU로부터 읽어온 index로 구분( SW또는 HW )되는제한각도의 min 값을 반환.
    public float GetLimitMin(int index)
    {
        if (index < 0) index = 0;
        else if (index > KnrHeader.LcuCommon.ROBOT_AXIS_CNT_MAX - 1) index = KnrHeader.LcuCommon.ROBOT_AXIS_CNT_MAX - 1;

        return _jointLimit.fLimitMin[index];
    }

    //설명: LCU로부터 읽어온 index로 구분( SW또는 HW )되는 제한각도의 max 값을 반환.
    public float GetLimitMax(int index)
    {
        if (index < 0) index = 0;
        else if (index > KnrHeader.LcuCommon.ROBOT_AXIS_CNT_MAX - 1) index = KnrHeader.LcuCommon.ROBOT_AXIS_CNT_MAX - 1;

        return _jointLimit.fLimitMax[index];
    }



}

