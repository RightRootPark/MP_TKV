using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using ScottPlot;
using ScottPlot.Control;
using ScottPlot.Plottable;
using KnrHeader;
using System.IO;
using System.Net.Sockets;
using System.Net;
using System.Security;
using System.IO.Ports;

namespace MP
{
    public partial class MP : Form
    {
        #region Serializable
        //UPD Socket 정보, 초기화 및 설정 저장
        [Serializable]
        public class JsonSocket
        {
            public List<string> name;
            public List<string> IP;
            public List<int> port;
        }

        [Serializable]
        public class JsonVelocity
        {
            public List<float> joint;
            public List<float> world;

            //public List<int> jointDir;
            //public List<int> worldDir;
        }

        [Serializable]
        public class Position
        {
            public List<float> HOME;
            public List<float> PARKING;
            public List<float> Target1;
            public List<float> Target2;
            public List<float> Target3;
            public List<float> Target4;
            public List<float> Target5;
            public List<float> Target6;
            public List<float> Target7;
            public List<float> Target8;
            public List<float> Target9;
            public List<float> Target10;
            public List<float> Target11;
            public List<float> Target12;
            public List<float> Target13;
            public List<float> Target14;
            public List<float> Target15;
            public List<float> Target16;
            public List<float> Target17;
            public List<float> Target18;
            public List<float> Target19;
            public List<float> Target20;
            public List<float> Target21;

        }

        [Serializable]
        public class Pos_Gain
        {
            public List<float> ConveGain;
            public List<float> Surge;
            public List<float> Sway;
            public List<float> Heave;
            public List<float> Roll;
            public List<float> Pitch;
            public List<float> Yaw;
            //public List<float> th_7;
        }

        ////SVC: LCU_SVC_CAN_SEND, LCU_SVC_CAN_RECEIVE - LCU pro 적용 _ Added by Hmshin _ 20221115
        //[Serializable]
        //public struct Cansend
        //{
        //    public int Id;
        //    public char subindex;
        //    public float Data;
        //}
        #endregion

        #region private
        //#define과 유사한 고정(const) 선언
        //설정 파일 저장 경로 ..dir\setting\
        private const string FILE_PATH_SETTING = @"setting\";
        private const string FILE_NAME_SOCKET = "socketInfo.txt";
        private const string FILE_NAME_VELOCITY = "velocity.txt";
        private const string FILE_NAME_POSITION = "position.txt";
        private const string FILE_NAME_POS_GAIN = "posGain.txt";

        private const string CHART_PATH = "C:/Users/R&D_SW/Desktop/Chart";
        //private const string CHART_NAME = DateTime.Now.ToString("yyyy-MM-dd : HH-mm-ss");

        private const string FILE_PATH_SAVE = @"DATA_SAVE\";

        private byte mode;

// call using Class
        private UDPServer server;
        private TCPServer tcpserver;
        private UDPClientTKV udpclienttkv;
        private TKVplay TkvPlay;
        
        private QsysECPcomunication QsysCom;
        private UdpClient clientMain;
        private UdpClient clientArm;
        private UdpClient clientLCUpro;
        private UdpClient clientArm2;
        private UdpClient clientArm3;
        private UdpClient clientArm4;
        private UdpClient clientSetup;

        private bool solOnOff = false;

        //Thread - UI Update
        private const int Interval = 1;//ms
        private bool targetCurrent = false;
        private Thread monitoring;
        private Thread move;
        private bool moveThread = false;


        //Profile
        private Button selectButton;
        //private OpenFileDialog openFileDialog1;
        private double x_Profile = 0, surge_Profile = 0, sway_Profile = 0, heave_Profile = 0, roll_Profile = 0, pitch_Profile = 0, yaw_Profile = 0;
        private int listcnt = 0;

        //Command
        private float Velocity = 0;
        private float Pos_Cmd_Joint = 0;
        private float Pos_Cmd_World = 0;
        private float _velLevel = 1;
        private float[] jointVelocityCmd = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        private float[] worldVelocityCmd = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        private float[] jointPosCmd = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        private float[] worldPosCmd = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        private float[] forceCmd = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        private float[] inputCmd = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        private float _toolPosCmd = 0;
        private float RMS = 0;

        private int velocityIndex = 0;
        private bool _isGripOn = false;
        private bool _isGasOn = false;
        private double[] _Err = new double[3];

        public float _ConveGain;
        public float _ToolVel;
        public float[] _VelLimit = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        public float[] _AccelLimit = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        public float[] _JerkLimit = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];

        //Auto move
        // _movetoflat : -1 초기값 / 0 - 타겟위치로 이동 중 / 1 - 이동 완료
        // 2 - Profile 시작위치이동 중 / 3 - Profile  시작위치로 이동 완료
        // 4 - Ready 상태 송출
        // 5 - Profile start 입력 / 6 - Pause 입력 / 7 - Profile stop 입력
        private int _movetoflag = -1;
        private float[] targetPos = new float[8];
        private float[] currentPos_Joint = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        private float[] currentPos_World = new float[LcuCommon.ROBOT_AXIS_CNT_MAX];
        private float[] deltaPos = new float[8];
        private float _pi = 3.1415927f;
        private string posList;
        private float[] savePos = new float[LcuCommon.ROBOT_AXIS_CNT_MAX + 2];

        private object _sender;
        private EventArgs _e;
        private MouseEventArgs _me;

        //Data Save
        private StreamWriter data_Des;
        private StreamWriter data_Mea;
        private bool _isSaveOn = false;

        //Arm3
        private bool _isReady = false;
        //private bool _ESD_1 = false;
        private bool _ESD_2 = false;
        private bool _esdReady = false;
        private int _profileFlag = -1;
        private int _ESDFlag = -1;

        private bool _pnDebug = false;
        private bool _SimulatorMode = false;

        // Daincube
        private int serial_open_flag = 0;
        SerialPort Daincube = new SerialPort();

        // sleepmode PW
        private string PW = "abcd"; //기준 패스워드
        private string inputPW = null; // 입력 패스워드
        private int pwCount = 0;


        #endregion

        #region Public
        //소캣 정보의 순서(index)
        public enum socketIndex
        {
            Arm,
            UIPC,
            Arm2,
            Arm3,
            Arm4
            //UIPC,
            //Arm2,
            //Arm3,
            //Arm,
            //LCU_pro
        }

        public LCU_Manager robotManager { get; private set; }
        //public Setup_Manager _setManager { get; private set; }
        public JsonSocket socketInfo { get; private set; }
        public JsonVelocity jsonVelocity { get; private set; }
        public Position _position { get; private set; }

        public Pos_Gain _posGain { get; private set; }
       // public TCPServer tcpdata{ get;private set; }

        //public NiDaqMax _digitalInOut { get; private set; }

        //public Profile _profileManager { get; private set; }
        //Form f2 = new Profile();
        #endregion

        #region CAN field
        private bool canON = false;
        private int canID = -2;
        private int canIndex;
        private char canSubIndex;
        private float canValue;
        private float canValueARM;
        private string _canSerialCmd;
        private string _canSerialCmdArm;
        private string _canData;
        private string _canDataARM;



        #endregion

        #region ChartView 
        //Random rand = new Random();
        //private static int array = 200_000;
        //private int preArray = 200;
        //readonly double[] Surge = new double[array];
        //readonly double[] Sway = new double[array];
        //readonly double[] Heave = new double[array];
        //readonly double[] Roll = new double[array];
        //readonly double[] Pitch = new double[array];
        //readonly double[] Yaw = new double[array];
        //private double[] Joint1 = new double[array];
        //private double[] Joint2 = new double[array];
        //private double[] Joint3 = new double[array];
        //private double[] Joint4 = new double[array];
        //private double[] Joint5 = new double[array];
        //private double[] Joint6 = new double[array];
        ////private double[] _X;
        ////private double[] _Y;
        //DataGen.Electrocardiogram ecg = new DataGen.Electrocardiogram();
        //Stopwatch sw = Stopwatch.StartNew();
        //Stopwatch _runtime = Stopwatch.StartNew();
        //VLine vline;
        //bool _isChartOn = false;
        //bool _isChartOnDesMea = false;

        //SignalPlot _surge, _sway, _heave, _roll, _pitch, _yaw;
        //SignalPlot _surge_Profile, _sway_Profile, _heave_Profile, _roll_Profile, _pitch_Profile, _yaw_Profile;
        //SignalPlot _joint1, _joint2, _joint3, _joint4, _joint5, _joint6;
        ////ScatterPlot _surge, _sway, _heave, _roll, _pitch, _yaw;
        ////ScatterPlot _surge_Profile, _sway_Profile, _heave_Profile, _roll_Profile, _pitch_Profile, _yaw_Profile;

        //private string _saveChartImg;
        //bool _ispauseOn = false;
        #endregion

        #region Profile
        public string _Surge { get; set; }
        public string _Sway { get; set; }
        public string _Heave { get; set; }
        public string _Roll { get; set; }
        public string _Pitch { get; set; }
        public string _Yaw { get; set; }

        readonly double[] Surge_Profile = new double[100_000];
        readonly double[] Sway_Profile = new double[100_000];
        readonly double[] Heave_Profile = new double[100_000];
        readonly double[] Roll_Profile = new double[100_000];
        readonly double[] Pitch_Profile = new double[100_000];
        readonly double[] Yaw_Profile = new double[100_000];

        bool _isProfileOn = false;
        bool _isPlayOn = false;

        int data_index = 0;             // 0 ~ 5 .. Profile의 Column: Surge, Sway, Heave, Roll, Pitch, Yaw
        int data_index_pre = 0;         //  0 ~ 5 .. Graph 출력 - Pause 후 재시작 시 적용
        int nextValueIndex = -1;        // Profile의 Row
        int nextValueIndex_pre = -1;    // Profile의 Row 저장 - Pause 후 재시작 시 적용
        int cnf_Profile = 0;            // 실제 실행한 프로파일 횟 수
        int profileCnt = 1;             // 실행하고자 하는 프로파일 횟 수

        double currentRightEdge = 200;
        double currentLeftEdge = 0;
        #endregion

        public MP()
        {
            robotManager = new LCU_Manager();
            //_setManager = new Setup_Manager();
            //openFileDialog1 = new OpenFileDialog();
            //_digitalInOut = new NiDaqMax();
            //_profileManager = new Profile();


            InitializeComponent();

            Awake();    //LCU pro 통신 설정 초기화
            Start();    //UI 화면 설정 초기화

            Daincube.DataReceived += new SerialDataReceivedEventHandler(Daincube_DataReceived);
        }

        #region _ Daincube
        private void Serial_autoopen()
        {
            string[] port_lists = new string[10];
            //string test;
            int serial_portnum = 0;
            int port_size = 0;
            byte[] serial_send_data = new byte[8];
            string temp = "";
            //string comport_str = "";

            foreach (string port_list in SerialPort.GetPortNames())
            {
                port_lists[serial_portnum] = port_list;
                //IncludeTextMessage(port_list);
                //test = port_list;


                //IncludeTextMessage(port_lists[serial_portnum]);
                serial_portnum = serial_portnum + 1;
            }

            temp = Convert.ToString(serial_portnum);
            //IncludeTextMessage(temp);

            port_size = serial_portnum;

            for (serial_portnum = 0; serial_portnum < port_size; serial_portnum++)
            {
                if (port_lists[serial_portnum] == "COM7")
                {
                    //IncludeTextMessage("Daincube connect success");

                    Daincube.PortName = "COM7";//다인큐브 interface Serial port

                    Daincube.BaudRate = 115200;
                    Daincube.DataBits = 8;
                    Daincube.StopBits = StopBits.One;
                    Daincube.Parity = Parity.None;


                    if (!Daincube.IsOpen)
                    {
                        try
                        {
                            Daincube.Open();
                            //IncludeTextMessage("Serial port open");
                            //comport_str+=""+"is Open!!"

                        }
                        catch
                        {
                            //IncludeTextMessage("no serial port");
                        }
                    }
                }

                else
                {
                    //IncludeTextMessage("serial fail");
                }
            }
            /*
            Daincube.PortName = "COM7";//다인큐브 interface Serial port
            
            Daincube.BaudRate = 115200;
            Daincube.DataBits = 8;
            Daincube.StopBits = StopBits.One;
            Daincube.Parity = Parity.None;

            
            if (!Daincube.IsOpen)
            {
                try
                {
                    Daincube.Open();
                    IncludeTextMessage("Serial port open");
                    //comport_str+=""+"is Open!!"

                }
                catch 
                {
                    IncludeTextMessage("no serial port");
                }
            }
            */
        }
        private void Daincube_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            this.Invoke(new EventHandler(Daincube_Serialparse));
        }
        private void Daincube_Serialparse(object s, EventArgs e)
        {
            string temp = "";

            string SEL_keypad = "";
            string SEL_keypad_UorD = "";
            string SEL_keypad_select = "";

            int i_recv_size = Daincube.BytesToRead;
            int SPEED_temp;

            byte[] b_tmp_buf = new byte[i_recv_size];
            string recv_str = "";

            double SPEED_bar2txt = 0;

            int temp_int = 0;

            Daincube.Read(b_tmp_buf, 0, i_recv_size);
            recv_str = Encoding.Default.GetString(b_tmp_buf);



            SEL_keypad = Convert.ToString(b_tmp_buf[2]);
            SEL_keypad_UorD = Convert.ToString(b_tmp_buf[3]);
            SEL_keypad_select = Convert.ToString(b_tmp_buf[4]);

            if (SEL_keypad == "61")
            {
                if (SEL_keypad_UorD == "49")    // Keypad Down
                {
                    switch (SEL_keypad_select)
                    {
                        case "30": //KEY A
                            J1Down_MouseDown(_sender, _me);
                            J1Down_Click(_sender, _me);
                            break;
                        case "48": //KEY B
                            J1Up_MouseDown(_sender, _me);
                            J1Up_Click(_sender, _me);
                            break;
                        case "46": //KEY C
                            J2Down_MouseDown(_sender, _me);
                            J2Down_Click(_sender, _me);
                            break;
                        case "32": //KEY D
                            J2Up_MouseDown(_sender, _me);
                            J2Up_Click(_sender, _me);
                            break;
                        case "18": //KEY E
                            J3Down_MouseDown(_sender, _me);
                            J3Down_Click(_sender, _me);
                            break;
                        case "33": //KEY F
                            J3Up_MouseDown(_sender, _me);
                            J3Up_Click(_sender, _me);
                            break;
                        case "34": //KEY G
                            J4Down_MouseDown(_sender, _me);
                            J4Down_Click(_sender, _me);
                            break;
                        case "35": //KEY H
                            J4Up_MouseDown(_sender, _me);
                            J4Up_Click(_sender, _me);
                            break;
                        case "23": //KEY I
                            J5Down_MouseDown(_sender, _me);
                            J5Down_Click(_sender, _me);
                            break;
                        case "36": //KEY J
                            J5Up_MouseDown(_sender, _me);
                            J5Up_Click(_sender, _me);
                            break;


                        case "103": //KEY UP
                            //if (velLevel_List.SelectedIndex < 4) velLevel_List.SelectedIndex++;
                            PW_a_Click(_sender, _e);
                            break;
                        case "108": //KEY DOWN
                            //if (velLevel_List.SelectedIndex >= 0) velLevel_List.SelectedIndex--;
                            PW_c_Click(_sender, _e);
                            break;
                        case "105": //KEY LEFT'
                            //_debug.Text = "LEFT";
                            PW_b_Click(_sender, _e);
                            break;
                        case "106": //KEY RIGHT
                            //_debug.Text = "RIGHT";
                            PW_d_Click(_sender, _e);
                            break;

                        case "64": //KEY F6
                            if (mode_List.SelectedIndex >= 5) mode_List.SelectedIndex = 0;
                            else mode_List.SelectedIndex++;
                            break;
                        case "65": //KEY F7
                            //_debug.Text = "F7";
                            break;
                        //case "67": //KEY F9
                        //_debug.Text = "F9";
                        //    break;
                        default:
                            break;
                    }
                }
                else if (SEL_keypad_UorD == "48")   // Keypad Up
                {
                    Velocity_MouseUp(_sender, _me);
                }
            }
            /*
            string temp = "";
            int ReceiveData = Daincube.ReadByte();
            temp = string.Format("{0:X2}", ReceiveData);
            */


            //IncludeTextMessage(temp);
            //this.BeginInvoke(new SetTextCallBack(display_data), new object[] { recv_str });
        }
        #endregion

        #region Socket Open & Initialize  of Json/UI
        void Start()
        {
            //cGain.Text = _posGain.ConveGain[0].ToString();
            //vLimit_1.Text = _posGain.Surge[0].ToString();
            //vLimit_2.Text = _posGain.Sway[0].ToString();
            //vLimit_3.Text = _posGain.Heave[0].ToString();
            //vLimit_4.Text = _posGain.Roll[0].ToString();
            //vLimit_5.Text = _posGain.Pitch[0].ToString();
            //vLimit_6.Text = _posGain.Yaw[0].ToString();
            //vLimit_7.Text = _posGain.th_7[0].ToString();

            mode_List.SelectedIndex = 0;
            velLevel_List.SelectedIndex = 0;

            this.KeyPreview = true; //Keyboard 입력 이벤트

            //monitoringLB_update();

            if (serial_open_flag == 0)
            {
                Serial_autoopen();
                serial_open_flag = 1;
                //IncludeTextMessage("serial_done");
            }
            pn_Monitoring.Enabled = btnReset.Enabled  = setMaster.Enabled = mode_List.Enabled = false;

        }
        void Awake()
        {
            //폴더 생성
            DirectoryInfo dic = new DirectoryInfo(FILE_PATH_SETTING);
            if (!dic.Exists) dic.Create();

            //폴더 생성
            DirectoryInfo dic2 = new DirectoryInfo(FILE_PATH_SAVE);
            if (!dic2.Exists) dic2.Create();

            ////폴더 생성
            //DirectoryInfo dic3 = new DirectoryInfo(CHART_PATH);
            //if (!dic3.Exists) dic3.Create();

            //load Json(세팅 값 불러오기) 
            LoadJsonScoket(FILE_PATH_SETTING + FILE_NAME_SOCKET);
            LoadJsonVelocity(FILE_PATH_SETTING + FILE_NAME_VELOCITY);
            LoadPosition(FILE_PATH_SETTING + FILE_NAME_POSITION);
            //LoadPosGain(FILE_PATH_SETTING + FILE_NAME_POS_GAIN);

            //디바이스 별 각 client UDP 소캣 생성
            clientMain = new UdpClient(); //PC
            CreateArm(); //LCU
            CreateArm2(); //LCU2
            CreateArm3(); //LCU3


            //CreateSetup();

            //UDP 서버 소캣 및 thread 생성
            server = new UDPServer(socketInfo.port[(int)socketIndex.UIPC], Interval, ParseData);
            //server = new UDPServer(socketInfo.port[(int)socketIndex.Arm2], Interval, ParseData);
            udpclienttkv = new UDPClientTKV("127.0.0.1", 50002);

            tcpserver = new TCPServer();

            //클래스초기화
            TkvPlay = new TKVplay();
            QsysCom = new QsysECPcomunication();


            //setMaster_Click(_sender, _e);
            btnSim_CheckedChanged(_sender, _e);

            if (monitoring == null)
            {
                monitoring = new Thread(Run);
                monitoring.IsBackground = true;
                monitoring.Start();
            }
        }
        private void CreateArm()
        {
            //IP 정보를 담기
            IPEndPoint ipEndPoint = new IPEndPoint(IPAddress.Parse(socketInfo.IP[(int)socketIndex.Arm])
                , socketInfo.port[(int)socketIndex.Arm]);
            clientLCUpro = new UdpClient();
            robotManager.SocketUDPCreateArm(clientLCUpro, ipEndPoint);
        }
        private void CreateArm2()
        {
            //IP 정보를 담기
            IPEndPoint ipEndPoint = new IPEndPoint(IPAddress.Parse(socketInfo.IP[(int)socketIndex.Arm2])
                , socketInfo.port[(int)socketIndex.Arm2]);
            clientArm2 = new UdpClient();
            robotManager.SocketUDPCreateArm2(clientArm2, ipEndPoint);
        }
        private void CreateArm3()
        {
            //IP 정보를 담기
            IPEndPoint ipEndPoint = new IPEndPoint(IPAddress.Parse(socketInfo.IP[(int)socketIndex.Arm3])
                , socketInfo.port[(int)socketIndex.Arm3]);
            clientArm3 = new UdpClient();
            robotManager.SocketUDPCreateArm3(clientArm3, ipEndPoint);
        }
        //private void CreateSetup()
        //{
        //    //IP 정보를 담기
        //    IPEndPoint ipEndPoint = new IPEndPoint(IPAddress.Parse(socketInfo.IP[(int)socketIndex.Arm])
        //        , socketInfo.port[(int)socketIndex.Arm]);
        //    clientSetup = new UdpClient();
        //    _setManager.SocketUDPCreateGain(clientSetup, ipEndPoint);
        //}
        //UDP Receive 데이터를 파싱하기 위한 함수.
        //UDPServer 클래스에서 데이터 수신 시 쓰레드가 지속적으로 호출하는 함수
        private void ParseData(string ip, byte[] bytes)
        {
            //아이피를 비교하여 어떤 디바이스인지 매칭
            if (ip.Equals(socketInfo.IP[(int)socketIndex.Arm])) //Arm
            {
                robotManager.arm.ParseReceive(bytes);
            }
        }
        private void btnReset_Click(object sender, EventArgs e)
        {
            //pn_Automove.Enabled = false;
            pos_update_Click(_sender, _e);
            //solOnOff = !solOnOff;
            //if (solOnOff)
            {
                robotManager.arm.SendErrorReset();
                robotManager.arm2.SendErrorReset();

                if (robotManager.arm.isConnect)
                {
                    btn_LCUpro.BackColor = Color.DarkCyan;
                    btnReset.BackColor = Color.DarkCyan;
                    btnEmg.BackColor = Color.FromArgb(58, 65, 73);
                    //btnReset.Text = "SOL OFF";

                    pn_Monitoring.Enabled = mode_List.Enabled = pn_Automove.Enabled = true;
                }
                else
                {
                    btn_LCUpro.BackColor = Color.FromArgb(58, 65, 73);
                    btnEmg.BackColor = Color.DarkCyan;

                    //pn_Command.Enabled = pn_Automove.Enabled = pn_Profile.Enabled = false;
                    movestop_Click(_sender, _e);
                }
            }
            //else
            //{
            //    robotManager.arm.SendSetEmergency();
            //    btnReset.BackColor = Color.FromArgb(58, 65, 73);
            //    //btnReset.Text = "SOL ON";

            //    pn_Command.Enabled = pn_Automove.Enabled = pn_Profile.Enabled = false;
            //    movestop_Click(_sender, _e);
            //    movestop_profile_Click(_sender, _e);
            //}
        }
        private void btnEmg_Click(object sender, EventArgs e)
        {
            pos_update_Click(_sender, _e);
            movestop_Click(_sender, _e);

            if(robotManager.arm.isEmergencyStop == false) robotManager.arm.SendSetEmergency();
            mode_List.SelectedIndex = 0;
            robotManager.arm.SetArmPosCmd(jointPosCmd, 0, 0);
            robotManager.arm.SetArmVelCmd(jointVelocityCmd, 1);
            btn_LCUpro.BackColor = Color.FromArgb(58, 65, 73);
            btnEmg.BackColor = Color.DarkCyan;

            solOnOff = false;
            btnReset.BackColor = Color.FromArgb(58, 65, 73);
            pn_Automove.Enabled = false;
            //btnReset.Text = "SOL ON";

            //btnReset.Enabled = false;
            //pn_Command.Enabled = pn_Automove.Enabled = pn_Profile.Enabled = false;
            //for (int i = 0; i < 10; i++) robotManager.arm.SendSetEmergency();
        }
        private void setMaster_Click(object sender, EventArgs e)
        {
            //setMaster.BackColor = Color.DarkCyan;
            robotManager.arm.SendSetMasterControl();
            robotManager.arm2.SendSetMasterControl();
            btnReset.Enabled = true;
        }
        private void Connect_Click(object sender, EventArgs e)
        {
            CreateArm(); //LCU
            Thread.Sleep(10);
            CreateArm2();//LCU2
            Thread.Sleep(10);
            CreateArm3();//LCU2

            mode_List.SelectedIndex = 0;
            robotManager.ChangeControlMode(0);
            robotManager.ClickControlStart();
            //pos_update.Enabled = true;
            pos_update_Click(_sender, _e);
            //for (int i = 0; i < 15; i++) robotManager.arm.SendInitStart();

            //if ((Convert.ToSingle(E1.Text)) < 0.5 && (Convert.ToSingle(E1.Text)) > -0.5 && Convert.ToSingle(W1.Text) != 0
            //    && (Convert.ToSingle(E2.Text)) < 0.5 && (Convert.ToSingle(E2.Text)) > -0.5 && Convert.ToSingle(W2.Text) != 0
            //    && (Convert.ToSingle(E3.Text)) < 0.5 && (Convert.ToSingle(E3.Text)) > -0.5 && Convert.ToSingle(W3.Text) != 0
            //    && (Convert.ToSingle(E4.Text)) < 0.5 && (Convert.ToSingle(E4.Text)) > -0.5 && Convert.ToSingle(W4.Text) != 0
            //    && (Convert.ToSingle(E5.Text)) < 0.5 && (Convert.ToSingle(E5.Text)) > -0.5 && Convert.ToSingle(W5.Text) != 0
            //    //&& (Convert.ToSingle(E6.Text)) < 0.5 && (Convert.ToSingle(E6.Text)) > -0.5 && Convert.ToSingle(W6.Text) != 0
            //    )
            Thread.Sleep(10);
            {
                setMaster.Enabled = true;
                pn_Automove.Enabled = false;
                btnReset.BackColor = Color.FromArgb(50, 54, 58);
                btnEmg.BackColor = Color.DarkCyan;
            }
            //else { btnReset.Enabled = false; btnEmg.BackColor = Color.DarkRed; btnReset.BackColor = Color.FromArgb(50, 54, 58); }

            //if (formsPlot1.Plot != null) formsPlot1.Plot.Clear();
            //UpdateValues();
            //timerRender.Enabled = true;
            //timerUpdateData.Enabled = true;

            if (monitoring == null)
            {
                monitoring = new Thread(Run);
                monitoring.IsBackground = true;
                monitoring.Start();
            }
            
        }
       
        #endregion

        #region Command_event
        private void MP_KeyDown(object sender, KeyEventArgs e)
        {
            switch (e.KeyCode)
            {
                case Keys.Escape:
                    btnEmg_Click(sender, e);
                    break;
                case Keys.Enter:
                    if (canON && (CAN_Serail.Text != "" && CAN_Serail.Text != " "))
                    {
                        CAN_Serail.Text = _canSerialCmd + CAN_Serail.Text;
                        //robotManager.arm.SendCanSerialData(canID, CAN_Serail.Text);
                        robotManager.arm.SendCanSerialDataUDP(canID, CAN_Serail.Text);
                        CAN_Serail.Text = "";
                        CAN_Serail.Select();
                    }
                    //if (!canON && ((cGain.Text != "" && cGain.Text != " ") || (vLimit_1.Text != "" && vLimit_1.Text != " ")))
                    //{
                    //    SetPosGain();
                    //}
                    break;
                case Keys.F1:
                    break;
                case Keys.F2:                    
                    break;
            }
        }

        private void btnSend_Click(object sender, EventArgs e)
        {
            SetPosGain();
        }
        private void SetPosGain()
        {
            _ConveGain = Convert.ToSingle(cGain.Text);
            _ToolVel = Convert.ToSingle(vTool.Text);

            _VelLimit[0] = Convert.ToSingle(vLimit_1.Text);
            _VelLimit[1] = Convert.ToSingle(vLimit_2.Text);
            _VelLimit[2] = Convert.ToSingle(vLimit_3.Text);
            _VelLimit[3] = Convert.ToSingle(vLimit_4.Text);
            _VelLimit[4] = Convert.ToSingle(vLimit_5.Text);
            _VelLimit[5] = Convert.ToSingle(vLimit_6.Text);

            _AccelLimit[0] = Convert.ToSingle(aLimit_1.Text);
            _AccelLimit[1] = Convert.ToSingle(aLimit_2.Text);
            _AccelLimit[2] = Convert.ToSingle(aLimit_3.Text);
            _AccelLimit[3] = 0;
            _AccelLimit[4] = 0;
            _AccelLimit[5] = 0;

            _JerkLimit[0] = Convert.ToSingle(jLimit_1.Text);
            _JerkLimit[1] = Convert.ToSingle(jLimit_2.Text);
            _JerkLimit[2] = Convert.ToSingle(jLimit_3.Text);
            _JerkLimit[3] = 0;
            _JerkLimit[4] = 0;
            _JerkLimit[5] = 0;

           robotManager.arm.SendSetPosGain(_ConveGain, _ToolVel, _VelLimit, _AccelLimit, _JerkLimit);

            //if (File.Exists(FILE_PATH_SETTING + FILE_NAME_POS_GAIN))
            //{
            //    _posGain.ConveGain.Clear();
            //    _posGain.Surge.Clear();
            //    _posGain.Sway.Clear();
            //    _posGain.Heave.Clear();
            //    _posGain.Roll.Clear();
            //    _posGain.Pitch.Clear();
            //    _posGain.Yaw.Clear();

            //    _posGain.ConveGain.Add(_ConveGain);
            //    _posGain.Surge.Add(_VelLimit[0]);
            //    _posGain.Sway.Add(_VelLimit[1]);
            //    _posGain.Heave.Add(_VelLimit[2]);
            //    _posGain.Roll.Add(_VelLimit[3]);
            //    _posGain.Pitch.Add(_VelLimit[4]);
            //    _posGain.Yaw.Add(_VelLimit[5]);

            //    JsonFileIO.Save(_posGain, FILE_PATH_SETTING + FILE_NAME_POS_GAIN);
            //}
        }
        private void MP_KeyUp(object sender, KeyEventArgs e)
        {
            switch (e.KeyCode)
            {
                case Keys.Escape:
                    btnEmg_Click(sender, e);
                    break;
            }
        }
       
        private void velLevel_List_SelectedIndexChanged(object sender, EventArgs e)
        {
            int index = velLevel_List.SelectedIndex;

            switch (index)
            {
                case 0: 
                    _velLevel = 1.0f;
                    break;
                case 1: 
                    _velLevel = 3.0f;
                    break;
                case 2: 
                    _velLevel = 5.0f;
                    break;
                case 3: 
                    _velLevel = 10.0f;
                    break;
                case 4: 
                    _velLevel = 20.0f;
                    break;
            }
        }
       
        private void btnDebug_Click(object sender, EventArgs e)
        {
            _pnDebug = !_pnDebug;
            if (_pnDebug)
            {
                PN_NotUse.Visible = true;
            }
            else
            {
                PN_NotUse.Visible = false;
            }
            robotManager.arm._sleepCount = 0;
        }
        private void btnSim_CheckedChanged(object sender, EventArgs e)
        {
            //_SimulatorMode = !_SimulatorMode;            
            if (btnSim.Checked)
            {
                robotManager.arm.SendSetSimulator_ON();
                robotManager.arm2.SendSetSimulator_ON();
                robotManager.arm3.SendSetSimulator_ON();
                //btn_Init_WorldPos.Visible = cmd_send2.Visible = true;
            }
            else
            {
                robotManager.arm.SendSetSimulator_OFF();
                robotManager.arm2.SendSetSimulator_OFF();
                robotManager.arm3.SendSetSimulator_OFF();
                //btn_Init_WorldPos.Visible = cmd_send2.Visible = false;
            }
        }
        #endregion

        #region Monitoring
        private void Run()
        {
            while (true)
            {
                Monitoring_Update();
                Thread.Sleep(10);
            }
        }

        private void Monitoring_Update()
        {
            if (this.InvokeRequired)
            {
                this.BeginInvoke(new Action(() =>
                {
                    #region Simulation TKV file sinc test-- Simulation sinc test-- Simulation sinc test-- Simulation sinc test-- 
                    if (checkBoxSimulMod.Checked)
                    {                        
                        labelSimulcomand.Text = "Simul Acitive! " +server.SimulAngle[23]+", " + server.SimulAngle[24] + ", " + server.SimulAngle[25] + ", " + server.SimulAngle[4] + ", " + server.SimulAngle[5] + ", " + server.SimulAngle[6] + ", " + server.SimulAngle[21] + ", " + server.SimulAngle[22] + ", " + server.SimulAngle[23] + ", " + server.SimulAngle[24] + ", " + server.SimulAngle[25] + ", " + server.SimulAngle[26] + ", " + server.SimulAngle[27] + ", " + server.SimulAngle[28] + ", " + server.SimulAngle[14] + ", " + "-234fr-";
                        JT1.Text = server.SimulAngle[23].ToString();
                        JT2.Text = server.SimulAngle[24].ToString();
                        JT3.Text = server.SimulAngle[25].ToString();
                        JT4.Text = server.SimulAngle[4].ToString();
                        JT5.Text = server.SimulAngle[5].ToString();
                        JT6.Text = server.SimulAngle[6].ToString();

                        //JT1.Text = tcpserver.DataString();
                        //JT1.Text = TkvPlay.dataArray[23];
                        //JT2.Text = TkvPlay.dataArray[24];
                        //JT3.Text = TkvPlay.dataArray[24];
                        //JT4.Text = TkvPlay.dataArray[25];
                        //JT5.Text = TkvPlay.dataArray[26];
                        //JT6.Text = TkvPlay.dataArray[27];

                        if (robotManager.arm.mode == 5) //Joint_pos Mode 가 잘 되었을때
                        {
                            jointPosCmd[0] = inputCmd[0] = Convert.ToSingle(JT1.Text);
                            JT1.Text = jointPosCmd[0].ToString();

                            jointPosCmd[1] = inputCmd[1] = Convert.ToSingle(JT2.Text);
                            JT2.Text = jointPosCmd[1].ToString();

                            jointPosCmd[2] = inputCmd[2] = Convert.ToSingle(JT3.Text);
                            JT3.Text = jointPosCmd[2].ToString();

                            jointPosCmd[3] = inputCmd[3] = Convert.ToSingle(JT4.Text);
                            JT4.Text = jointPosCmd[3].ToString();

                            jointPosCmd[4] = inputCmd[4] = Convert.ToSingle(JT5.Text);
                            JT5.Text = jointPosCmd[4].ToString();

                            jointPosCmd[5] = inputCmd[5] = Convert.ToSingle(JT6.Text);
                            JT6.Text = jointPosCmd[5].ToString();


                            if (inputCmd[0] >= 130) jointPosCmd[0] = 129.9f;
                            else if (inputCmd[0] <= -10) jointPosCmd[0] = -9.9f;
                            else jointPosCmd[0] = inputCmd[0];

                            for (byte i = 1; i > 5; i++) //just test
                            {
                                if (inputCmd[i] >= 130) jointPosCmd[i] = 129.9f;
                                else if (inputCmd[i] <= -10) jointPosCmd[i] = -9.9f;
                                else jointPosCmd[i] = inputCmd[i];
                            }                            
                            robotManager.arm.SetArmPosCmd(jointPosCmd, 0, 0);

                        }
                    }
                    else labelSimulcomand.Text = "Simul is not active" + tcpserver.DataString();

                    //udp angle send test
                    for (int i = 0; i < 29; i++)
                    {
                        udpclienttkv.JointAngleArray[i]=i;
                    }
                for (int i = 0; i < 6; i++)
                {
                    udpclienttkv.JointAngleArray[i] = robotManager.arm.monitoringData.fJointPosCur[i] * Convert.ToSingle(180 / Math.PI);
                }
                
                    udpclienttkv.UDPwCurrentSend(udpclienttkv.JointAngleArray); //시뮬레이터에 데이터 보내기
                    

                    if (!(TkvPlay.State_num == 0)) TKVPlayBar.Value = TkvPlay.lineIndex;//TKV 진행바 업데이트

                    #endregion Simulation sinc test-- Simulation sinc test-- Simulation sinc test-- Simulation sinc test-- 
                    //_digitalInOut.UpdateDIState();
                    //_digitalInOut._DO();

                    #region Monitoring data from LCU

                    //for (int i = 0; i < LcuCommon.ROBOT_AXIS_CNT_MAX; i++) currentPos_Joint[i] = robotManager.arm.monitoringData.fJointPosCur[i];
                    //for (int i = 0; i < LcuCommon.ROBOT_AXIS_CNT_MAX; i++) currentPos_World[i] = robotManager.arm.monitoringData.fWorldPosCur[i];

                    J1.Text = (robotManager.arm.monitoringData.fJointPos[0] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    J2.Text = (robotManager.arm.monitoringData.fJointPos[1] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    J3.Text = (robotManager.arm.monitoringData.fJointPos[2] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    J4.Text = (robotManager.arm.monitoringData.fJointPos[3] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    J5.Text = (robotManager.arm.monitoringData.fJointPos[4] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    J6.Text = (robotManager.arm.monitoringData.fJointPos[5] * Convert.ToSingle(180 / Math.PI)).ToString("f2");

                    JC1.Text = (robotManager.arm.monitoringData.fJointPosCur[0] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    JC2.Text = (robotManager.arm.monitoringData.fJointPosCur[1] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    JC3.Text = (robotManager.arm.monitoringData.fJointPosCur[2] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    JC4.Text = (robotManager.arm.monitoringData.fJointPosCur[3] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    JC5.Text = (robotManager.arm.monitoringData.fJointPosCur[4] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    JC6.Text = (robotManager.arm.monitoringData.fJointPosCur[5] * Convert.ToSingle(180 / Math.PI)).ToString("f2");

                    W1.Text = robotManager.arm.monitoringData.fWorldPos[0].ToString("f2");
                    W2.Text = robotManager.arm.monitoringData.fWorldPos[1].ToString("f2");
                    W3.Text = robotManager.arm.monitoringData.fWorldPos[2].ToString("f2");
                    W4.Text = (robotManager.arm.monitoringData.fWorldPos[3] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    W5.Text = (robotManager.arm.monitoringData.fWorldPos[4] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    W6.Text = (robotManager.arm.monitoringData.fWorldPos[5] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    W7.Text = (robotManager.arm.monitoringData.fJointPos[4] * Convert.ToSingle(180 / Math.PI)).ToString("f2");

                    WC1.Text = robotManager.arm.monitoringData.fWorldPosCur[0].ToString("f2");
                    WC2.Text = (robotManager.arm.monitoringData.fWorldPosCur[1]).ToString("f2");
                    WC3.Text = (robotManager.arm.monitoringData.fWorldPosCur[2]).ToString("f2");
                    WC4.Text = (robotManager.arm.monitoringData.fWorldPosCur[3] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    WC5.Text = (robotManager.arm.monitoringData.fWorldPosCur[4] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    WC6.Text = (robotManager.arm.monitoringData.fWorldPosCur[5] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                    WC7.Text = (robotManager.arm.monitoringData.fJointPosCur[4] * Convert.ToSingle(180 / Math.PI)).ToString("f2");

                    //_Err[0] = Convert.ToDouble(robotManager.arm.monitoringData.fWorldPos[0] - robotManager.arm.monitoringData.fWorldPosCur[0]);
                    //_Err[1] = Convert.ToDouble(robotManager.arm.monitoringData.fWorldPos[1] - robotManager.arm.monitoringData.fWorldPosCur[1]);
                    //_Err[2] = Convert.ToDouble(robotManager.arm.monitoringData.fWorldPos[2] - robotManager.arm.monitoringData.fWorldPosCur[2]);
                    RMS_Error.Text = robotManager.arm.monitoringData.fPosError.ToString("f2"); //RMS_Error.Text = Math.Sqrt(Math.Pow(_Err[0],2)+ Math.Pow(_Err[1], 2)+ Math.Pow(_Err[2], 2)).ToString("f2");
                    RMS_Check.Text = robotManager.arm.monitoringData.fRmsCheck.ToString("f2");
                    ACCESS_Check.Text = robotManager.arm.monitoringData.nAutoControl.ToString("f2");
                    PW_Check.Text = inputPW;


                    if (robotManager.arm.mode == 2 || robotManager.arm.mode == 3)
                    {
                        E1.Text = robotManager.arm._jointVelCmd.fJointCmd[0].ToString("f2");
                        E2.Text = robotManager.arm._jointVelCmd.fJointCmd[1].ToString("f2");
                        E3.Text = robotManager.arm._jointVelCmd.fJointCmd[2].ToString("f2");
                        E4.Text = robotManager.arm._jointVelCmd.fJointCmd[3].ToString("f2");
                        E5.Text = robotManager.arm._jointVelCmd.fJointCmd[4].ToString("f2");
                        E6.Text = robotManager.arm._jointVelCmd.fJointCmd[5].ToString("f2");

                        Tar_E1.Text = robotManager.arm._worldVelCmd.fWorldCmd[0].ToString("f2");
                        Tar_E2.Text = robotManager.arm._worldVelCmd.fWorldCmd[1].ToString("f2");
                        Tar_E3.Text = robotManager.arm._worldVelCmd.fWorldCmd[2].ToString("f2");
                        Tar_E4.Text = robotManager.arm._worldVelCmd.fWorldCmd[3].ToString("f2");
                        Tar_E5.Text = robotManager.arm._worldVelCmd.fWorldCmd[4].ToString("f2");
                        Tar_E6.Text = robotManager.arm._worldVelCmd.fWorldCmd[5].ToString("f2");
                    }
                    else
                    {
                        E1.Text = robotManager.arm._jointPosCmd.fJointCmd[0].ToString("f2");
                        E2.Text = robotManager.arm._jointPosCmd.fJointCmd[1].ToString("f2");
                        E3.Text = robotManager.arm._jointPosCmd.fJointCmd[2].ToString("f2");
                        E4.Text = robotManager.arm._jointPosCmd.fJointCmd[3].ToString("f2");
                        E5.Text = robotManager.arm._jointPosCmd.fJointCmd[4].ToString("f2");
                        E6.Text = robotManager.arm._jointPosCmd.fJointCmd[5].ToString("f2");

                        Tar_E1.Text = robotManager.arm._worldPosCmd_M.fWorldCmd[0].ToString("f2");
                        Tar_E2.Text = robotManager.arm._worldPosCmd_M.fWorldCmd[1].ToString("f2");
                        Tar_E3.Text = robotManager.arm._worldPosCmd_M.fWorldCmd[2].ToString("f2");
                        Tar_E4.Text = robotManager.arm._worldPosCmd_M.fWorldCmd[3].ToString("f2");
                        Tar_E5.Text = robotManager.arm._worldPosCmd_M.fWorldCmd[4].ToString("f2");
                        Tar_E6.Text = robotManager.arm._worldPosCmd_M.fWorldCmd[5].ToString("f2");
                    }
                    #endregion

                    //if (robotManager.arm.masterDevice == LcuCommon.HEADER_RM) setMaster.BackColor = Color.DarkCyan;
                    if (robotManager.arm.masterDevice == LcuCommon.HEADER_PC1) setMaster.BackColor = Color.DarkCyan;
                    else
                    {
                        setMaster.BackColor = Color.FromArgb(58, 65, 73);

                        //if (moveThread)
                        //{
                        //    movestop_Click(_sender, _e);
                        //    movestop_profile_Click(_sender, _e);
                        //}
                    }

                    if (robotManager.arm.isConnect)
                    {
                        if (robotManager.arm.isEmergencyStop)
                        {
                            btn_LCUpro.BackColor = btnEmg.BackColor = Color.DarkRed;
                            btnEmg_Click(_sender, _e);
                            pn_Monitoring.Enabled = btnReset.Enabled = setMaster.Enabled = mode_List.Enabled = false;
                            Debug.WriteLine("EmergencyStop");

                        }
                        else btn_LCUpro.BackColor = Color.DarkCyan;
                    }
                    else
                    {
                        btn_LCUpro.BackColor = Color.FromArgb(58, 65, 73);
                        pn_Monitoring.Enabled = btnReset.Enabled = setMaster.Enabled = mode_List.Enabled = false;
                        Debug.WriteLine("Connect failed");
                    }
                    if (robotManager.arm._sleepFlag)
                    {
                        pn_Monitoring.Enabled = btnReset.Enabled = setMaster.Enabled = mode_List.Enabled = false;
                        pn_PW.Visible = true;
                        Debug.WriteLine("Sleep Mode");
                    }

                    if (pwCount==4 && inputPW == PW)
                    {
                        Debug.WriteLine("wakeup");
                        robotManager.arm._sleepCount = 0;
                        robotManager.arm._sleepFlag = false;
                        pn_PW.Visible = false;
                        pn_Monitoring.Enabled = btnReset.Enabled = setMaster.Enabled = mode_List.Enabled = true;
                        inputPW = null;
                        pwCount = 0;
                    }
                    else if(pwCount>=4 && inputPW != PW)
                    {
                        inputPW = null;
                        pwCount = 0;
                    }
                }
                ));
            }
            else
            {

            }
        }
        #endregion

        #region - Move to target position
        private void readTXT_Click(object sender, EventArgs e)
        {
            LoadPosition(FILE_PATH_SETTING + FILE_NAME_POSITION);
            LoadJsonVelocity(FILE_PATH_SETTING + FILE_NAME_VELOCITY);
        }
        private void save_curPos_Click(object sender, EventArgs e)
        {
            update_cur_pos();

            switch (posList)
            {
                case "HOME":
                    _position.HOME.Clear();
                    for (int i = 0; i < 8; i++) _position.HOME.Insert(i, savePos[i]);
                    break;
                case "PARKING":
                    _position.PARKING.Clear();
                    for (int i = 0; i < 8; i++) _position.PARKING.Insert(i, savePos[i]);
                    break;
                case "Target1":
                    _position.Target1.Clear();
                    for (int i = 0; i < 8; i++) _position.Target1.Insert(i, savePos[i]);
                    break;
                case "Target2":
                    _position.Target2.Clear();
                    for (int i = 0; i < 8; i++) _position.Target2.Insert(i, savePos[i]);
                    break;
                case "Target3":
                    _position.Target3.Clear();
                    for (int i = 0; i < 8; i++) _position.Target3.Insert(i, savePos[i]);
                    break;
                case "Target4":
                    _position.Target4.Clear();
                    for (int i = 0; i < 8; i++) _position.Target4.Insert(i, savePos[i]);
                    break;
                case "Target5":
                    _position.Target5.Clear();
                    for (int i = 0; i < 8; i++) _position.Target5.Insert(i, savePos[i]);
                    break;
                case "Target6":
                    _position.Target6.Clear();
                    for (int i = 0; i < 8; i++) _position.Target6.Insert(i, savePos[i]);
                    break;
                case "Target7":
                    _position.Target7.Clear();
                    for (int i = 0; i < 8; i++) _position.Target7.Insert(i, savePos[i]);
                    break;
                case "Target8":
                    _position.Target8.Clear();
                    for (int i = 0; i < 8; i++) _position.Target8.Insert(i, savePos[i]);
                    break;
                case "Target9":
                    _position.Target9.Clear();
                    for (int i = 0; i < 8; i++) _position.Target9.Insert(i, savePos[i]);
                    break;
                case "Target10":
                    _position.Target10.Clear();
                    for (int i = 0; i < 8; i++) _position.Target10.Insert(i, savePos[i]);
                    break;
                case "Target11":
                    _position.Target11.Clear();
                    for (int i = 0; i < 8; i++) _position.Target11.Insert(i, savePos[i]);
                    break;
                case "Target12":
                    _position.Target12.Clear();
                    for (int i = 0; i < 8; i++) _position.Target12.Insert(i, savePos[i]);
                    break;
                case "Target13":
                    _position.Target13.Clear();
                    for (int i = 0; i < 8; i++) _position.Target13.Insert(i, savePos[i]);
                    break;
                case "Target14":
                    _position.Target14.Clear();
                    for (int i = 0; i < 8; i++) _position.Target14.Insert(i, savePos[i]);
                    break;
                case "Target15":
                    _position.Target15.Clear();
                    for (int i = 0; i < 8; i++) _position.Target15.Insert(i, savePos[i]);
                    break;
                case "Target16":
                    _position.Target16.Clear();
                    for (int i = 0; i < 8; i++) _position.Target16.Insert(i, savePos[i]);
                    break;
                case "Target17":
                    _position.Target17.Clear();
                    for (int i = 0; i < 8; i++) _position.Target17.Insert(i, savePos[i]);
                    break;
                case "Target18":
                    _position.Target18.Clear();
                    for (int i = 0; i < 8; i++) _position.Target18.Insert(i, savePos[i]);
                    break;
                case "Target19":
                    _position.Target19.Clear();
                    for (int i = 0; i < 8; i++) _position.Target19.Insert(i, savePos[i]);
                    break;
                case "Target20":
                    _position.Target20.Clear();
                    for (int i = 0; i < 8; i++) _position.Target20.Insert(i, savePos[i]);
                    break;
                case "Target21":
                    _position.Target21.Clear();
                    for (int i = 0; i < 8; i++) _position.Target21.Insert(i, savePos[i]);
                    break;
            }

            JsonFileIO.Save(_position, FILE_PATH_SETTING + FILE_NAME_POSITION);
            Debug.WriteLine("Json save");
            readTXT_Click(_sender, _e);
        }
        private void update_cur_pos()
        {
            posList = PosList.SelectedItem.ToString();

            savePos[0] = Convert.ToSingle(C1.Text);
            savePos[1] = Convert.ToSingle(C2.Text);
            savePos[2] = Convert.ToSingle(C3.Text);
            savePos[3] = Convert.ToSingle(C4.Text);
            savePos[4] = Convert.ToSingle(C5.Text);
            savePos[5] = Convert.ToSingle(C6.Text);
            savePos[6] = Convert.ToSingle(C7.Text);
            savePos[7] = Convert.ToSingle(C8.Text);
        }
        private void PosList_SelectedIndexChanged(object sender, EventArgs e)
        {
            update_cur_pos();

            switch (posList)
            {
                case "HOME":
                    C1.Text = _position.HOME[0].ToString();
                    C2.Text = _position.HOME[1].ToString();
                    C3.Text = _position.HOME[2].ToString();
                    C4.Text = _position.HOME[3].ToString();
                    C5.Text = _position.HOME[4].ToString();
                    C6.Text = _position.HOME[5].ToString();
                    C7.Text = _position.HOME[6].ToString();
                    C8.Text = _position.HOME[7].ToString();
                    break;
                case "PARKING":
                    C1.Text = _position.PARKING[0].ToString();
                    C2.Text = _position.PARKING[1].ToString();
                    C3.Text = _position.PARKING[2].ToString();
                    C4.Text = _position.PARKING[3].ToString();
                    C5.Text = _position.PARKING[4].ToString();
                    C6.Text = _position.PARKING[5].ToString();
                    C7.Text = _position.PARKING[6].ToString();
                    C8.Text = _position.PARKING[7].ToString();
                    break;
                case "Target1":    //Target 1
                    C1.Text = _position.Target1[0].ToString();
                    C2.Text = _position.Target1[1].ToString();
                    C3.Text = _position.Target1[2].ToString();
                    C4.Text = _position.Target1[3].ToString();
                    C5.Text = _position.Target1[4].ToString();
                    C6.Text = _position.Target1[5].ToString();
                    C7.Text = _position.Target1[6].ToString();
                    C8.Text = _position.Target1[7].ToString();
                    break;
                case "Target2": //Target 2
                    C1.Text = _position.Target2[0].ToString();
                    C2.Text = _position.Target2[1].ToString();
                    C3.Text = _position.Target2[2].ToString();
                    C4.Text = _position.Target2[3].ToString();
                    C5.Text = _position.Target2[4].ToString();
                    C6.Text = _position.Target2[5].ToString();
                    C7.Text = _position.Target2[6].ToString();
                    C8.Text = _position.Target2[7].ToString();
                    break;
                case "Target3": //Target 3
                    C1.Text = _position.Target3[0].ToString();
                    C2.Text = _position.Target3[1].ToString();
                    C3.Text = _position.Target3[2].ToString();
                    C4.Text = _position.Target3[3].ToString();
                    C5.Text = _position.Target3[4].ToString();
                    C6.Text = _position.Target3[5].ToString();
                    C7.Text = _position.Target3[6].ToString();
                    C8.Text = _position.Target3[7].ToString();
                    break;
                case "Target4": //Target 4
                    C1.Text = _position.Target4[0].ToString();
                    C2.Text = _position.Target4[1].ToString();
                    C3.Text = _position.Target4[2].ToString();
                    C4.Text = _position.Target4[3].ToString();
                    C5.Text = _position.Target4[4].ToString();
                    C6.Text = _position.Target4[5].ToString();
                    C7.Text = _position.Target4[6].ToString();
                    C8.Text = _position.Target4[7].ToString();
                    break;
                case "Target5": //Target 5
                    C1.Text = _position.Target5[0].ToString();
                    C2.Text = _position.Target5[1].ToString();
                    C3.Text = _position.Target5[2].ToString();
                    C4.Text = _position.Target5[3].ToString();
                    C5.Text = _position.Target5[4].ToString();
                    C6.Text = _position.Target5[5].ToString();
                    C7.Text = _position.Target5[6].ToString();
                    C8.Text = _position.Target5[7].ToString();
                    break;
                case "Target6": //Target 6
                    C1.Text = _position.Target6[0].ToString();
                    C2.Text = _position.Target6[1].ToString();
                    C3.Text = _position.Target6[2].ToString();
                    C4.Text = _position.Target6[3].ToString();
                    C5.Text = _position.Target6[4].ToString();
                    C6.Text = _position.Target6[5].ToString();
                    C7.Text = _position.Target6[6].ToString();
                    C8.Text = _position.Target6[7].ToString();
                    break;
                case "Target7": //Target 7
                    C1.Text = _position.Target7[0].ToString();
                    C2.Text = _position.Target7[1].ToString();
                    C3.Text = _position.Target7[2].ToString();
                    C4.Text = _position.Target7[3].ToString();
                    C5.Text = _position.Target7[4].ToString();
                    C6.Text = _position.Target7[5].ToString();
                    C7.Text = _position.Target7[6].ToString();
                    C8.Text = _position.Target7[7].ToString();
                    break;
                case "Target8": //Target 8
                    C1.Text = _position.Target8[0].ToString();
                    C2.Text = _position.Target8[1].ToString();
                    C3.Text = _position.Target8[2].ToString();
                    C4.Text = _position.Target8[3].ToString();
                    C5.Text = _position.Target8[4].ToString();
                    C6.Text = _position.Target8[5].ToString();
                    C7.Text = _position.Target8[6].ToString();
                    C8.Text = _position.Target8[7].ToString();
                    break;
                case "Target9":    //Target 1
                    C1.Text = _position.Target9[0].ToString();
                    C2.Text = _position.Target9[1].ToString();
                    C3.Text = _position.Target9[2].ToString();
                    C4.Text = _position.Target9[3].ToString();
                    C5.Text = _position.Target9[4].ToString();
                    C6.Text = _position.Target9[5].ToString();
                    C7.Text = _position.Target9[6].ToString();
                    C8.Text = _position.Target9[7].ToString();
                    break;
                case "Target10": //Target 2
                    C1.Text = _position.Target10[0].ToString();
                    C2.Text = _position.Target10[1].ToString();
                    C3.Text = _position.Target10[2].ToString();
                    C4.Text = _position.Target10[3].ToString();
                    C5.Text = _position.Target10[4].ToString();
                    C6.Text = _position.Target10[5].ToString();
                    C7.Text = _position.Target10[6].ToString();
                    C8.Text = _position.Target10[7].ToString();
                    break;
                case "Target11": //Target 3
                    C1.Text = _position.Target11[0].ToString();
                    C2.Text = _position.Target11[1].ToString();
                    C3.Text = _position.Target11[2].ToString();
                    C4.Text = _position.Target11[3].ToString();
                    C5.Text = _position.Target11[4].ToString();
                    C6.Text = _position.Target11[5].ToString();
                    C7.Text = _position.Target11[6].ToString();
                    C8.Text = _position.Target11[7].ToString();
                    break;
                case "Target12": //Target 4
                    C1.Text = _position.Target12[0].ToString();
                    C2.Text = _position.Target12[1].ToString();
                    C3.Text = _position.Target12[2].ToString();
                    C4.Text = _position.Target12[3].ToString();
                    C5.Text = _position.Target12[4].ToString();
                    C6.Text = _position.Target12[5].ToString();
                    C7.Text = _position.Target12[6].ToString();
                    C8.Text = _position.Target12[7].ToString();
                    break;
                case "Target13": //Target 5
                    C1.Text = _position.Target13[0].ToString();
                    C2.Text = _position.Target13[1].ToString();
                    C3.Text = _position.Target13[2].ToString();
                    C4.Text = _position.Target13[3].ToString();
                    C5.Text = _position.Target13[4].ToString();
                    C6.Text = _position.Target13[5].ToString();
                    C7.Text = _position.Target13[6].ToString();
                    C8.Text = _position.Target13[7].ToString();
                    break;
                case "Target14": //Target 6
                    C1.Text = _position.Target14[0].ToString();
                    C2.Text = _position.Target14[1].ToString();
                    C3.Text = _position.Target14[2].ToString();
                    C4.Text = _position.Target14[3].ToString();
                    C5.Text = _position.Target14[4].ToString();
                    C6.Text = _position.Target14[5].ToString();
                    C7.Text = _position.Target14[6].ToString();
                    C8.Text = _position.Target14[7].ToString();
                    break;
                case "Target15": //Target 4
                    C1.Text = _position.Target15[0].ToString();
                    C2.Text = _position.Target15[1].ToString();
                    C3.Text = _position.Target15[2].ToString();
                    C4.Text = _position.Target15[3].ToString();
                    C5.Text = _position.Target15[4].ToString();
                    C6.Text = _position.Target15[5].ToString();
                    C7.Text = _position.Target15[6].ToString();
                    C8.Text = _position.Target15[7].ToString();
                    break;
                case "Target16": //Target 5
                    C1.Text = _position.Target16[0].ToString();
                    C2.Text = _position.Target16[1].ToString();
                    C3.Text = _position.Target16[2].ToString();
                    C4.Text = _position.Target16[3].ToString();
                    C5.Text = _position.Target16[4].ToString();
                    C6.Text = _position.Target16[5].ToString();
                    C7.Text = _position.Target16[6].ToString();
                    C8.Text = _position.Target16[7].ToString();
                    break;
                case "Target17": //Target 6
                    C1.Text = _position.Target17[0].ToString();
                    C2.Text = _position.Target17[1].ToString();
                    C3.Text = _position.Target17[2].ToString();
                    C4.Text = _position.Target17[3].ToString();
                    C5.Text = _position.Target17[4].ToString();
                    C6.Text = _position.Target17[5].ToString();
                    C7.Text = _position.Target17[6].ToString();
                    C8.Text = _position.Target17[7].ToString();
                    break;

                case "Target18": //Target 6
                    C1.Text = _position.Target18[0].ToString();
                    C2.Text = _position.Target18[1].ToString();
                    C3.Text = _position.Target18[2].ToString();
                    C4.Text = _position.Target18[3].ToString();
                    C5.Text = _position.Target18[4].ToString();
                    C6.Text = _position.Target18[5].ToString();
                    C7.Text = _position.Target18[6].ToString();
                    C8.Text = _position.Target18[7].ToString();
                    break;
                case "Target19": //Target 4
                    C1.Text = _position.Target19[0].ToString();
                    C2.Text = _position.Target19[1].ToString();
                    C3.Text = _position.Target19[2].ToString();
                    C4.Text = _position.Target19[3].ToString();
                    C5.Text = _position.Target19[4].ToString();
                    C6.Text = _position.Target19[5].ToString();
                    C7.Text = _position.Target19[6].ToString();
                    C8.Text = _position.Target19[7].ToString();
                    break;
                case "Target20": //Target 5
                    C1.Text = _position.Target20[0].ToString();
                    C2.Text = _position.Target20[1].ToString();
                    C3.Text = _position.Target20[2].ToString();
                    C4.Text = _position.Target20[3].ToString();
                    C5.Text = _position.Target20[4].ToString();
                    C6.Text = _position.Target20[5].ToString();
                    C7.Text = _position.Target20[6].ToString();
                    C8.Text = _position.Target20[7].ToString();
                    break;
                case "Target21": //Target 6
                    C1.Text = _position.Target21[0].ToString();
                    C2.Text = _position.Target21[1].ToString();
                    C3.Text = _position.Target21[2].ToString();
                    C4.Text = _position.Target21[3].ToString();
                    C5.Text = _position.Target21[4].ToString();
                    C6.Text = _position.Target21[5].ToString();
                    C7.Text = _position.Target21[6].ToString();
                    C8.Text = _position.Target21[7].ToString();
                    break;

            }
        }
        private void MoveToTarget()
        {
            if (robotManager.arm.mode == 2) //Joint - Velocity
            {
                for (int i = 0; i < 6; i++)
                {
                    deltaPos[i] = Math.Abs(currentPos_Joint[i] - targetPos[i]);

                    if (deltaPos[i] > 0.05)
                    {
                        if (currentPos_Joint[i] > targetPos[i]) Velocity = deltaPos[i] * jsonVelocity.joint[i] * _pi / 180 * -1;
                        else if (currentPos_Joint[i] < targetPos[i]) Velocity = deltaPos[i] * jsonVelocity.joint[i] * _pi / 180;
                    }
                    else
                    {
                        Velocity = 0;
                    }

                    jointVelocityCmd[i] = Velocity;
                }
                robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);

                //Debug.WriteLine(deltaPos[0] + " " + deltaPos[1] + " " + deltaPos[2]);
                if (deltaPos[0] < 0.5 && deltaPos[1] < 0.5 && deltaPos[2] < 0.5 && deltaPos[3] < 0.5 && deltaPos[4] < 0.5 && deltaPos[5] < 0.5)
                {
                    movestop_Click(_sender, _e);
                }
            }
            else if (robotManager.arm.mode == 3) //World - Velocity
            {
                for (int i = 0; i < 6; i++)
                {
                    deltaPos[i] = Math.Abs(currentPos_World[i] - targetPos[i]);

                    if (deltaPos[i] > 0.05)
                    {
                        if (currentPos_World[i] > targetPos[i]) Velocity = targetPos[i] * jsonVelocity.world[i] * _pi / 180 * -1;
                        else if (currentPos_World[i] < targetPos[i]) Velocity = targetPos[i] * jsonVelocity.world[i] * _pi / 180;
                    }
                    else Velocity = 0;

                    jointVelocityCmd[i] = Velocity;
                }
                robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);

                if (deltaPos[0] < 1 && deltaPos[1] < 1 && deltaPos[2] < 1 && deltaPos[3] < 0.5 && deltaPos[4] < 0.5 && deltaPos[5] < 0.5)
                {
                    movestop_Click(_sender, _e);
                }
            }
        }
        private void MoveTo()
        {
            while (moveThread)
            {
                MoveToTarget();

                Thread.Sleep(100);
            }
        }

        private void movestop_Click(object sender, EventArgs e)
        {
            //pnError.Visible = false;
            for (int i = 0; i < 6; i++)
            {
                Velocity = 0;
                jointVelocityCmd[i] = Velocity;
            }
            robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);
            robotManager.arm2.SetArmVelCmd(jointVelocityCmd, 0);


            //if (_dstart.Checked)
            //{
            //_playflag = 2;
            //_movetoflag = 1;
            //}
            //else
            //{
            _movetoflag = 1;
            //}

            //if (move != null)
            if (moveThread)
            {
                //btnHome.BackColor = Color.FromArgb(58, 65, 73);
                //btnLift.BackColor = Color.FromArgb(58, 65, 73);
                //btnParking.BackColor = Color.FromArgb(58, 65, 73);

                move.Abort();
                move.Join();

                Debug.WriteLine("move stop");
                Debug.WriteLine(move);
                moveThread = false;
            }

            //btnHome.Enabled = true; btnTar1.Enabled = true; btnParking.Enabled = true;
            btnHome.BackColor = btnTar1.BackColor = btnParking.BackColor = Color.FromArgb(58, 65, 73);

            //pn_Command.Enabled = pn_Profile.Enabled = true;
            //// 간헐적으로 에러 발생
            //C1.Text = "0.00";
            //C2.Text = "0.00";
            //C3.Text = "0.00";
            //C4.Text = "0.00";
            //C5.Text = "0.00";
            //C6.Text = "0.00";
        }

        private void move_to_pos()
        {
            #region new code _ move to target _ 20221108

            targetPos[0] = inputCmd[0] = Convert.ToSingle(C1.Text);
            targetPos[1] = inputCmd[1] = Convert.ToSingle(C2.Text);
            targetPos[2] = inputCmd[2] = Convert.ToSingle(C3.Text);
            targetPos[3] = inputCmd[3] = Convert.ToSingle(C4.Text);
            targetPos[4] = inputCmd[4] = Convert.ToSingle(C5.Text);
            targetPos[5] = inputCmd[5] = Convert.ToSingle(C6.Text);

            if (moveThread == false)
            {
                moveThread = true;

                move = new Thread(MoveTo);
                move.IsBackground = true;
                move.Start();
            }

            //stRun.BackColor = Color.DarkCyan;
            #endregion
        }
        private void move_to_target_Click(object sender, EventArgs e)
        {
            _movetoflag = 0;
        }

        #endregion

        #region - Current Update & Command click
        // Manipulator control mode
        private void mode_List_SelectedIndexChanged(object sender, EventArgs e)
        {
            int index = mode_List.SelectedIndex;

            switch (index)
            {
                case 0: //Joint - Velocity control
                    //pn_Automove.Enabled = false;
                    velLevel_List.Enabled = false;
                    //FT.Enabled = forceTx.Enabled = forceRelease.Enabled = false;
                    robotManager.arm.LCU_MODECHECK_VEL = true;
                    for (int i = 0; i < 6; i++) jointVelocityCmd[i] = 0;
                    robotManager.arm.SetArmVelCmd(jointVelocityCmd, 1);
                    robotManager.ChangeControlMode(0);

                    C1.Text = JT1.Text = "0.00";
                    C2.Text = JT2.Text = "0.00";
                    C3.Text = JT3.Text = "0.00";
                    C4.Text = JT4.Text = "0.00";
                    C5.Text = JT5.Text = "0.00";
                    C6.Text = JT6.Text = "0.00";

                    monitoringLB_update(0);
                    break;
                case 1: //World - Velocity control
                    //pn_Automove.Enabled = false;
                    velLevel_List.Enabled = true;
                    //FT.Enabled = forceTx.Enabled = forceRelease.Enabled = false;
                    robotManager.arm.LCU_MODECHECK_VEL = true;
                    for (int i = 0; i < 6; i++) worldVelocityCmd[i] = 0;
                    robotManager.arm.SetArmVelCmd(worldVelocityCmd, 1);
                    robotManager.ChangeControlMode(1); 

                    C1.Text = WT1.Text = "0.00";
                    C2.Text = WT2.Text = "0.00";
                    C3.Text = WT3.Text = "0.00";
                    C4.Text = WT4.Text = "0.00";
                    C5.Text = WT5.Text = "0.00";
                    C6.Text = WT6.Text = "0.00";

                    monitoringLB_update(1);
                    break;
                case 2: //Joint - Position control
                    robotManager.arm.LCU_MODECHECK_VEL = false;
                    robotManager.ChangeControlMode(2);

                    C1.Text = JT1.Text = JC1.Text;
                    C2.Text = JT2.Text = JC2.Text;
                    C3.Text = JT3.Text = JC3.Text;
                    C4.Text = JT4.Text = JC4.Text;
                    C5.Text = JT5.Text = JC5.Text;
                    C6.Text = JT6.Text = JC6.Text;

                    //cmd_send_Click(_sender, _e);

                    monitoringLB_update(2);
                    break;
                case 3: //World - Position control - Move from Measured to Target position 
                    //robotManager.arm.LCU_MODECHECK_VEL = false;
                    //robotManager.ChangeControlMode(3);

                    //C1.Text = WT1.Text = WC1.Text;
                    //C2.Text = WT2.Text = WC2.Text;
                    //C3.Text = WT3.Text = WC3.Text;
                    //C4.Text = WT4.Text = WC4.Text;
                    //C5.Text = WT5.Text = WC5.Text;
                    //C6.Text = WT6.Text = WC6.Text;
                    //C7.Text = WT7.Text = WC7.Text;
                    //C8.Text = WT8.Text = WC8.Text;

                    ////cmd_send_Click(_sender, _e);

                    //monitoringLB_update(3);
                    break;
                case 4: //World - Position control - Move from Previous Desired to Target position
                    pn_Automove.Enabled = true; 
                    velLevel_List.Enabled = true;
                    //FT.Enabled = forceTx.Enabled = forceRelease.Enabled = false;
                    robotManager.arm.LCU_MODECHECK_VEL = false;
                    robotManager.ChangeControlMode(4);

                    C1.Text = WT1.Text = WC1.Text;
                    C2.Text = WT2.Text = WC2.Text;
                    C3.Text = WT3.Text = WC3.Text;
                    C4.Text = WT4.Text = WC4.Text;
                    C5.Text = WT5.Text = WC5.Text;
                    C6.Text = WT6.Text = WC6.Text;
                    C7.Text = WT7.Text = WC7.Text;
                    C8.Text = WT8.Text = WC8.Text;

                    //cmd_send_Click(_sender, _e);

                    monitoringLB_update(4);
                    break;
                case 5: //Force control
                    //pn_Automove.Enabled = false;
                    velLevel_List.Enabled = false;
                    FT.Enabled = forceTx.Enabled = forceRelease.Enabled = true;
                    robotManager.arm.LCU_MODECHECK_VEL = false;
                    robotManager.ChangeControlMode(5);

                    ////pos_update_Click(_sender, _e);
                    //monitoringLB_update(5);
                    //Debug.WriteLine("Force control");
                    break;
            }
            cmd_send_Click(_sender, _e);
            //mode = robotManager.arm.mode;
            //Debug.WriteLine(mode);
        }
        private void pos_update_Click(object sender, EventArgs e)
        {
            if (robotManager.arm.mode == 2)// || robotManager.arm.mode == 3)
            {
                robotManager.arm.SetArmVelCmd(jointVelocityCmd, 1);

                C1.Text = JT1.Text = "0.00";
                C2.Text = JT2.Text = "0.00";
                C3.Text = JT3.Text = "0.00";
                C4.Text = JT4.Text = "0.00";
                C5.Text = JT5.Text = "0.00";
                C6.Text = JT6.Text = "0.00";

                //for (int i = 0; i < 6; i++) jointVelocityCmd[i] = 0;
                //robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);
            }
            else if (robotManager.arm.mode == 3)
            {
                robotManager.arm.SetArmVelCmd(worldVelocityCmd, 1);

                C1.Text = WT1.Text = "0.00";
                C2.Text = WT2.Text = "0.00";
                C3.Text = WT3.Text = "0.00";
                C4.Text = WT4.Text = "0.00";
                C5.Text = WT5.Text = "0.00";
                C6.Text = WT6.Text = "0.00";

                //for (int i = 0; i < 6; i++) worldVelocityCmd[i] = 0;
                //robotManager.arm.SetArmVelCmd(worldVelocityCmd, 0);
            }
            else if (robotManager.arm.mode == 5)
            {
                C1.Text = JT1.Text = JC1.Text;// (robotManager.arm.monitoringData.fJointPosCur[0] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                C2.Text = JT2.Text = JC2.Text;// (robotManager.arm.monitoringData.fJointPosCur[1] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                JT3.Text = JC3.Text;//(robotManager.arm.monitoringData.fJointPosCur[2] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                JT4.Text = JC4.Text;//(robotManager.arm.monitoringData.fJointPosCur[3] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                JT5.Text = JC5.Text;//(robotManager.arm.monitoringData.fJointPosCur[4] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                JT6.Text = JC6.Text;//(robotManager.arm.monitoringData.fJointPosCur[5] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
            }
            else if (robotManager.arm.mode == 8 || robotManager.arm.mode == 9)
            {
                C1.Text = WT1.Text = WC1.Text;// robotManager.arm.monitoringData.fWorldPosCur[0].ToString("f2");
                C2.Text = WT2.Text = WC2.Text;//robotManager.arm.monitoringData.fWorldPosCur[1].ToString("f2");
                C3.Text = WT3.Text = WC3.Text;//robotManager.arm.monitoringData.fWorldPosCur[2].ToString("f2");
                C4.Text = WT4.Text = WC4.Text;//(robotManager.arm.monitoringData.fWorldPosCur[3] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                C5.Text = WT5.Text = WC5.Text;//(robotManager.arm.monitoringData.fWorldPosCur[4] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                C6.Text = WT6.Text = WC6.Text;//(robotManager.arm.monitoringData.fWorldPosCur[5] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                C7.Text = WT7.Text = WC7.Text;//(robotManager.arm.monitoringData.fWorldPosCur[4] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
                WT8.Text = "1.0"; // WC8.Text;//(robotManager.arm.monitoringData.fWorldPosCur[5] * Convert.ToSingle(180 / Math.PI)).ToString("f2");
            }
            //pn_Command.Enabled = true;
        }
        private void btn_Init_WorldPos_Click(object sender, EventArgs e)
        {
            //JT1.Text = "-30";
            //JT2.Text = "90";
            //JT3.Text = "-7.5";
            //JT4.Text = "7.5";
            //JT5.Text = "-45";
            //JT6.Text = "0";

            WT1.Text = "3876.09";
            WT2.Text = "-259.08";
            WT4.Text = "-243.17";
            WT3.Text = "0";
            WT5.Text = "177.79";// "50";
            WT6.Text = "0";// "-35";
            WT7.Text = "0.5";// "50";
            WT8.Text = "0.5";// "-35";
        }
        private void cmd_send_Click(object sender, EventArgs e)
        {
            #region old code _ move to target
            if (robotManager.arm.mode == 5)
            {
                Debug.WriteLine("Joint Pos - ");
                jointPosCmd[0] = inputCmd[0] = Convert.ToSingle(JT1.Text);
                JT1.Text = jointPosCmd[0].ToString();

                jointPosCmd[1] = inputCmd[1] = Convert.ToSingle(JT2.Text);
                JT2.Text = jointPosCmd[1].ToString();

                jointPosCmd[2] = inputCmd[2] = Convert.ToSingle(JT3.Text);
                JT3.Text = jointPosCmd[2].ToString();

                jointPosCmd[3] = inputCmd[3] = Convert.ToSingle(JT4.Text);
                JT4.Text = jointPosCmd[3].ToString();

                jointPosCmd[4] = inputCmd[4] = Convert.ToSingle(JT5.Text);
                JT5.Text = jointPosCmd[4].ToString();

                //inputCmd[5] = Convert.ToSingle(JT6.Text);
                jointPosCmd[5] = inputCmd[5] = Convert.ToSingle(JT6.Text);
                JT6.Text = jointPosCmd[5].ToString();

                //for (int i = 0; i < 6; i++)
                //{
                //    //Pos_Cmd_Joint = inputCmd[i];
                //    //jointPosCmd[i] = Pos_Cmd_Joint;
                //}
                robotManager.arm.SetArmPosCmd(jointPosCmd, 0, 0);
            }
            else if (robotManager.arm.mode == 8 || robotManager.arm.mode == 9)
            {
                Debug.WriteLine("World Pos - ");
                //inputCmd[0] = Convert.ToSingle(WT1.Text);
                worldPosCmd[0] = inputCmd[0] = Convert.ToSingle(WT1.Text);
                WT1.Text = worldPosCmd[0].ToString();

                inputCmd[1] = Convert.ToSingle(WT2.Text);
                worldPosCmd[1] = inputCmd[1];
                WT2.Text = worldPosCmd[1].ToString();

                inputCmd[2] = Convert.ToSingle(WT3.Text);
                worldPosCmd[2] = inputCmd[2];
                WT3.Text = worldPosCmd[2].ToString();

                inputCmd[3] = Convert.ToSingle(WT4.Text);
                worldPosCmd[3] = inputCmd[3];
                WT4.Text = worldPosCmd[3].ToString();

                inputCmd[4] = Convert.ToSingle(WT5.Text);
                worldPosCmd[4] = inputCmd[4];
                WT5.Text = worldPosCmd[4].ToString();

                inputCmd[5] = Convert.ToSingle(WT6.Text);
                worldPosCmd[5] = inputCmd[5];
                WT6.Text = worldPosCmd[5].ToString();

                _toolPosCmd = Convert.ToSingle(WT7.Text);
                //worldPosCmd[6] = inputCmd[6];
                WT7.Text = _toolPosCmd.ToString();

                RMS = Convert.ToSingle(WT8.Text);
                //worldPosCmd[7] = inputCmd[7];
                WT8.Text = RMS.ToString();

                robotManager.arm.SetArmPosCmd(worldPosCmd, _toolPosCmd, RMS);
            }
            
            //else if (robotManager.arm.mode == 2 || robotManager.arm.mode == 3)//(dr_mode.Text == "Joint" || dr_mode.Text == "World")
            //{
            //    //inputCmdVel[0] = 0;// Convert.ToSingle(C1.Text);
            //    //inputCmdVel[1] = 0;// Convert.ToSingle(C2.Text);
            //    //inputCmdVel[2] = 0;// Convert.ToSingle(C3.Text);
            //    //inputCmdVel[3] = 0;// Convert.ToSingle(C4.Text);
            //    //inputCmdVel[4] = 0;// Convert.ToSingle(C5.Text);
            //    //inputCmdVel[5] = 0;// Convert.ToSingle(C6.Text);

            //    for (int i = 0; i < 6; i++)
            //    {
            //        Velocity = 0;// jsonVelocity.joint[velocityIndex] * inputCmdVel[i];
            //        jointVelocityCmd[i] = Velocity;
            //    }

            //    robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);
            //}
            #endregion
        }

        private void btnHome_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.HOME[0].ToString();
                WT2.Text = _position.HOME[1].ToString();
                WT3.Text = _position.HOME[2].ToString();
                WT4.Text = _position.HOME[3].ToString();
                WT5.Text = _position.HOME[4].ToString();
                WT6.Text = _position.HOME[5].ToString();
                WT7.Text = _position.HOME[6].ToString();
                WT8.Text = _position.HOME[7].ToString();

                cmd_send_Click(_sender, _e);
            }
        }
        private void btnTar1_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            //robotManager.arm.mode = 9;
            {
                WT1.Text = _position.Target1[0].ToString();
                WT2.Text = _position.Target1[1].ToString();
                WT3.Text = _position.Target1[2].ToString();
                WT4.Text = _position.Target1[3].ToString();
                WT5.Text = _position.Target1[4].ToString();
                WT6.Text = _position.Target1[5].ToString();
                WT7.Text = _position.Target1[6].ToString();
                WT8.Text = _position.Target1[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }
        private void btnTar2_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            //robotManager.arm.mode = 9;
            {
                WT1.Text = _position.Target2[0].ToString();
                WT2.Text = _position.Target2[1].ToString();
                WT3.Text = _position.Target2[2].ToString();
                WT4.Text = _position.Target2[3].ToString();
                WT5.Text = _position.Target2[4].ToString();
                WT6.Text = _position.Target2[5].ToString();
                WT7.Text = _position.Target2[6].ToString();
                WT8.Text = _position.Target2[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }
        private void btnTar3_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
                //robotManager.arm.mode = 9;
            {
                WT1.Text = _position.Target3[0].ToString();
                WT2.Text = _position.Target3[1].ToString();
                WT3.Text = _position.Target3[2].ToString();
                WT4.Text = _position.Target3[3].ToString();
                WT5.Text = _position.Target3[4].ToString();
                WT6.Text = _position.Target3[5].ToString();
                WT7.Text = _position.Target3[6].ToString();
                WT8.Text = _position.Target3[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }
        private void btnTar4_Click(object sender, EventArgs e)  //Stop
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target4[0].ToString();
                WT2.Text = _position.Target4[1].ToString();
                WT3.Text = _position.Target4[2].ToString();
                WT4.Text = _position.Target4[3].ToString();
                WT5.Text = _position.Target4[4].ToString();
                WT6.Text = _position.Target4[5].ToString();
                WT7.Text = _position.Target4[6].ToString();
                WT8.Text = _position.Target4[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar5_Click(object sender, EventArgs e) 
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target5[0].ToString();
                WT2.Text = _position.Target5[1].ToString();
                WT3.Text = _position.Target5[2].ToString();
                WT4.Text = _position.Target5[3].ToString();
                WT5.Text = _position.Target5[4].ToString();
                WT6.Text = _position.Target5[5].ToString();
                WT7.Text = _position.Target5[6].ToString();
                WT8.Text = _position.Target5[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar6_Click(object sender, EventArgs e)  
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target6[0].ToString();
                WT2.Text = _position.Target6[1].ToString();
                WT3.Text = _position.Target6[2].ToString();
                WT4.Text = _position.Target6[3].ToString();
                WT5.Text = _position.Target6[4].ToString();
                WT6.Text = _position.Target6[5].ToString();
                WT7.Text = _position.Target6[6].ToString();
                WT8.Text = _position.Target6[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }
        private void btnTar7_Click(object sender, EventArgs e)  
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target7[0].ToString();
                WT2.Text = _position.Target7[1].ToString();
                WT3.Text = _position.Target7[2].ToString();
                WT4.Text = _position.Target7[3].ToString();
                WT5.Text = _position.Target7[4].ToString();
                WT6.Text = _position.Target7[5].ToString();
                WT7.Text = _position.Target7[6].ToString();
                WT8.Text = _position.Target7[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }
        private void btnTar8_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target8[0].ToString();
                WT2.Text = _position.Target8[1].ToString();
                WT3.Text = _position.Target8[2].ToString();
                WT4.Text = _position.Target8[3].ToString();
                WT5.Text = _position.Target8[4].ToString();
                WT6.Text = _position.Target8[5].ToString();
                WT7.Text = _position.Target8[6].ToString();
                WT8.Text = _position.Target8[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }


        private void btnTar9_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target9[0].ToString();
                WT2.Text = _position.Target9[1].ToString();
                WT3.Text = _position.Target9[2].ToString();
                WT4.Text = _position.Target9[3].ToString();
                WT5.Text = _position.Target9[4].ToString();
                WT6.Text = _position.Target9[5].ToString();
                WT7.Text = _position.Target9[6].ToString();
                WT8.Text = _position.Target9[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar10_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target10[0].ToString();
                WT2.Text = _position.Target10[1].ToString();
                WT3.Text = _position.Target10[2].ToString();
                WT4.Text = _position.Target10[3].ToString();
                WT5.Text = _position.Target10[4].ToString();
                WT6.Text = _position.Target10[5].ToString();
                WT7.Text = _position.Target10[6].ToString();
                WT8.Text = _position.Target10[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar11_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target11[0].ToString();
                WT2.Text = _position.Target11[1].ToString();
                WT3.Text = _position.Target11[2].ToString();
                WT4.Text = _position.Target11[3].ToString();
                WT5.Text = _position.Target11[4].ToString();
                WT6.Text = _position.Target11[5].ToString();
                WT7.Text = _position.Target11[6].ToString();
                WT8.Text = _position.Target11[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar12_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target12[0].ToString();
                WT2.Text = _position.Target12[1].ToString();
                WT3.Text = _position.Target12[2].ToString();
                WT4.Text = _position.Target12[3].ToString();
                WT5.Text = _position.Target12[4].ToString();
                WT6.Text = _position.Target12[5].ToString();
                WT7.Text = _position.Target12[6].ToString();
                WT8.Text = _position.Target12[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar13_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target13[0].ToString();
                WT2.Text = _position.Target13[1].ToString();
                WT3.Text = _position.Target13[2].ToString();
                WT4.Text = _position.Target13[3].ToString();
                WT5.Text = _position.Target13[4].ToString();
                WT6.Text = _position.Target13[5].ToString();
                WT7.Text = _position.Target13[6].ToString();
                WT8.Text = _position.Target13[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar14_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target14[0].ToString();
                WT2.Text = _position.Target14[1].ToString();
                WT3.Text = _position.Target14[2].ToString();
                WT4.Text = _position.Target14[3].ToString();
                WT5.Text = _position.Target14[4].ToString();
                WT6.Text = _position.Target14[5].ToString();
                WT7.Text = _position.Target14[6].ToString();
                WT8.Text = _position.Target14[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar15_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target15[0].ToString();
                WT2.Text = _position.Target15[1].ToString();
                WT3.Text = _position.Target15[2].ToString();
                WT4.Text = _position.Target15[3].ToString();
                WT5.Text = _position.Target15[4].ToString();
                WT6.Text = _position.Target15[5].ToString();
                WT7.Text = _position.Target15[6].ToString();
                WT8.Text = _position.Target15[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar16_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target16[0].ToString();
                WT2.Text = _position.Target16[1].ToString();
                WT3.Text = _position.Target16[2].ToString();
                WT4.Text = _position.Target16[3].ToString();
                WT5.Text = _position.Target16[4].ToString();
                WT6.Text = _position.Target16[5].ToString();
                WT7.Text = _position.Target16[6].ToString();
                WT8.Text = _position.Target16[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar17_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target17[0].ToString();
                WT2.Text = _position.Target17[1].ToString();
                WT3.Text = _position.Target17[2].ToString();
                WT4.Text = _position.Target17[3].ToString();
                WT5.Text = _position.Target17[4].ToString();
                WT6.Text = _position.Target17[5].ToString();
                WT7.Text = _position.Target17[6].ToString();
                WT8.Text = _position.Target17[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar18_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target18[0].ToString();
                WT2.Text = _position.Target18[1].ToString();
                WT3.Text = _position.Target18[2].ToString();
                WT4.Text = _position.Target18[3].ToString();
                WT5.Text = _position.Target18[4].ToString();
                WT6.Text = _position.Target18[5].ToString();
                WT7.Text = _position.Target18[6].ToString();
                WT8.Text = _position.Target18[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar19_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target19[0].ToString();
                WT2.Text = _position.Target19[1].ToString();
                WT3.Text = _position.Target19[2].ToString();
                WT4.Text = _position.Target19[3].ToString();
                WT5.Text = _position.Target19[4].ToString();
                WT6.Text = _position.Target19[5].ToString();
                WT7.Text = _position.Target19[6].ToString();
                WT8.Text = _position.Target19[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar20_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target20[0].ToString();
                WT2.Text = _position.Target20[1].ToString();
                WT3.Text = _position.Target20[2].ToString();
                WT4.Text = _position.Target20[3].ToString();
                WT5.Text = _position.Target20[4].ToString();
                WT6.Text = _position.Target20[5].ToString();
                WT7.Text = _position.Target20[6].ToString();
                WT8.Text = _position.Target20[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnTar21_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.Target21[0].ToString();
                WT2.Text = _position.Target21[1].ToString();
                WT3.Text = _position.Target21[2].ToString();
                WT4.Text = _position.Target21[3].ToString();
                WT5.Text = _position.Target21[4].ToString();
                WT6.Text = _position.Target21[5].ToString();
                WT7.Text = _position.Target21[6].ToString();
                WT8.Text = _position.Target21[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }

        private void btnParking_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;
            
            if (robotManager.arm.mode == 9)
            {
                WT1.Text = _position.PARKING[0].ToString();
                WT2.Text = _position.PARKING[1].ToString();
                WT3.Text = _position.PARKING[2].ToString();
                WT4.Text = _position.PARKING[3].ToString();
                WT5.Text = _position.PARKING[4].ToString();
                WT6.Text = _position.PARKING[5].ToString();
                WT7.Text = _position.PARKING[6].ToString();
                WT8.Text = _position.PARKING[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }
        #endregion

        private void btnSTOP_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 4;

            if (robotManager.arm.mode == 9)
            {
                WT1.Text = WC1.Text;// _position.Target4[0].ToString();
                WT2.Text = WC2.Text;// _position.Target4[1].ToString();
                WT3.Text = WC3.Text;// _position.Target4[2].ToString();
                WT4.Text = WC4.Text;// _position.Target4[3].ToString();
                WT5.Text = WC5.Text;// _position.Target4[4].ToString();
                WT6.Text = WC6.Text;// _position.Target4[5].ToString();
                WT7.Text = WC7.Text;// _position.HOME[6].ToString();
                WT8.Text = WC8.Text;// _position.HOME[7].ToString();
                cmd_send_Click(_sender, _e);
            }
        }
        private void forceTx_Click(object sender, EventArgs e)
        {
            if (robotManager.arm.mode == 2 || robotManager.arm.mode == 3)
            {
                WT1.Text = WC1.Text;// _position.Target4[0].ToString();
                WT2.Text = WC2.Text;// _position.Target4[1].ToString();
                WT3.Text = WC3.Text;// _position.Target4[2].ToString();
                WT4.Text = WC4.Text;// _position.Target4[3].ToString();
                WT5.Text = WC5.Text;// _position.Target4[4].ToString();
                WT6.Text = WC6.Text;// _position.Target4[5].ToString();
                WT7.Text = WC7.Text;// _position.HOME[6].ToString();
                WT8.Text = WC8.Text;
            }
            mode_List.SelectedIndex = 5;
            robotManager.ChangeControlMode(5);
            if (robotManager.arm.mode == 10)
            {
                forceCmd[0] = forceCmd[1] = forceCmd[3] = forceCmd[4] = 0;// forceCmd[6] = forceCmd[7] = 0;
                forceCmd[2] = Convert.ToSingle(FT.Text); //3번축 forcecontrol값 입력
                forceCmd[5] = 1; //force명령 flag. on
                robotManager.arm.SetArmForceCmd(forceCmd);

                //robotManager.ChangeControlMode(5);
                //forceCmd.Text = 'c' + forceCmd.Text;
                //robotManager.arm.SendCanSerialDataUDP(3, forceCmd.Text);
                //forceCmd.Text = "";
                //Debug.WriteLine(forceCmd[2] + " " + forceCmd[5]);
                //txtCanData.Text = robotManager.arm._canUDPRx.rxData.ToString();
            }
        }

        private void forceRelease_Click(object sender, EventArgs e)
        {
            mode_List.SelectedIndex = 5;
            if (robotManager.arm.mode == 10)
            {
                forceCmd[5] = 0; //force명령 flag. off
                robotManager.arm.SetArmForceCmd(forceCmd);
                //txtCanData.Text = robotManager.arm._canUDPRx.rxData.ToString();
                Debug.WriteLine("force release");
                //mode_List.SelectedIndex = 4;
                robotManager.ChangeControlMode(4);
                cmd_send_Click(_sender, _e);
            }
        }

        #region Joint_Velocity-button
        private void Velocity_MouseUp(object sender, MouseEventArgs e)
        {
            J1Up.BackColor = J1Down.BackColor = J2Up.BackColor = J2Down.BackColor = Color.FromArgb(58, 65, 73);
            J3Up.BackColor = J3Down.BackColor = J4Up.BackColor = J4Down.BackColor = Color.FromArgb(58, 65, 73);
            J5Up.BackColor = J5Down.BackColor = Color.FromArgb(58, 65, 73);

            //if (armMode.Text == "Joint_Vel" || armMode.Text == "World_Vel")
            if (robotManager.arm.mode == 2)// || robotManager.arm.mode == 3)
            {
                //C1.Text = "0.00";
                //C2.Text = "0.00";
                //C3.Text = "0.00";
                //C4.Text = "0.00";
                //C5.Text = "0.00";
                //C6.Text = "0.00";

                for (int i = 0; i < 6; i++) jointVelocityCmd[i] = 0;
                robotManager.arm.SetArmVelCmd(jointVelocityCmd, 1);
            }
            else
            {
                for (int i = 0; i < 6; i++) worldVelocityCmd[i] = 0;
                robotManager.arm.SetArmVelCmd(worldVelocityCmd, 1);
            }
            robotManager.arm._sleepCount = 0;
            Debug.WriteLine("MouseUP");
        }
        private void J1Up_MouseDown(object sender, MouseEventArgs e)
        {
            J1Up.BackColor = Color.DarkSlateGray;
            if (robotManager.arm.mode == 2)
            {
                velocityIndex = 0;

                Velocity = jsonVelocity.joint[velocityIndex] *  1; //_velLevel *
                jointVelocityCmd[0] = Velocity;
                C1.Text = jointVelocityCmd[0].ToString();

                robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);
            }
            else if (robotManager.arm.mode == 3)
            {
                velocityIndex = 0;

                Velocity = jsonVelocity.world[velocityIndex] * _velLevel * -1;
                worldVelocityCmd[0] = Velocity;
                C1.Text = worldVelocityCmd[0].ToString();

                robotManager.arm.SetArmVelCmd(worldVelocityCmd, 0);
            }
            robotManager.arm._sleepCount = 0;
            Debug.WriteLine("J1UP");
        }
        private void J1Down_MouseDown(object sender, MouseEventArgs e)
        {
            J1Down.BackColor = Color.DarkSlateGray;
            if (robotManager.arm.mode == 2)
            {
                velocityIndex = 0;

                Velocity = jsonVelocity.joint[velocityIndex] * -1;
                jointVelocityCmd[0] = Velocity;
                C1.Text = jointVelocityCmd[0].ToString();

                robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);
            }
            else if (robotManager.arm.mode == 3)
            {
                velocityIndex = 0;

                Velocity = jsonVelocity.world[velocityIndex] * _velLevel * -1;
                worldVelocityCmd[0] = Velocity;
                C1.Text = worldVelocityCmd[0].ToString();

                robotManager.arm.SetArmVelCmd(worldVelocityCmd, 0);
            }
            robotManager.arm._sleepCount = 0;
        }
        private void J2Up_MouseDown(object sender, MouseEventArgs e)
        {
            J2Up.BackColor = Color.DarkSlateGray;
            if (robotManager.arm.mode == 2)
            {
                velocityIndex = 1;

                Velocity = jsonVelocity.joint[velocityIndex] * 1;
                jointVelocityCmd[1] = Velocity;
                C2.Text = jointVelocityCmd[1].ToString();

                robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);
            }
            else if (robotManager.arm.mode == 3)
            {
                velocityIndex = 1;

                Velocity = jsonVelocity.world[velocityIndex] * _velLevel * 1;
                worldVelocityCmd[1] = Velocity;
                C2.Text = worldVelocityCmd[1].ToString();

                robotManager.arm.SetArmVelCmd(worldVelocityCmd, 0);
            }
            robotManager.arm._sleepCount = 0;
        }
        private void J2Down_MouseDown(object sender, MouseEventArgs e)
        {
            J2Down.BackColor = Color.DarkSlateGray;
            if (robotManager.arm.mode == 2)
            {
                velocityIndex = 1;

                Velocity = jsonVelocity.joint[velocityIndex] * -1;
                jointVelocityCmd[1] = Velocity;
                C2.Text = jointVelocityCmd[1].ToString();

                robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);
            }
            else if (robotManager.arm.mode == 3)
            {
                velocityIndex = 1;

                Velocity = jsonVelocity.world[velocityIndex] * _velLevel * -1;
                worldVelocityCmd[1] = Velocity;
                C2.Text = worldVelocityCmd[1].ToString();

                robotManager.arm.SetArmVelCmd(worldVelocityCmd, 0);
            }
            robotManager.arm._sleepCount = 0;
        }
        private void J3Up_MouseDown(object sender, MouseEventArgs e)
        {
            J3Up.BackColor = Color.DarkSlateGray;
            if (robotManager.arm.mode == 2)
            {
                velocityIndex = 2;

                Velocity = jsonVelocity.joint[velocityIndex] * 1;
                jointVelocityCmd[2] = Velocity;
                C3.Text = jointVelocityCmd[2].ToString();

                robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);
            }
            else if (robotManager.arm.mode == 3)
            {
                velocityIndex = 2;

                Velocity = jsonVelocity.world[velocityIndex] * _velLevel * 1;
                worldVelocityCmd[2] = Velocity;
                C3.Text = worldVelocityCmd[2].ToString();

                robotManager.arm.SetArmVelCmd(worldVelocityCmd, 0);
            }
            robotManager.arm._sleepCount = 0;
        }
        private void J3Down_MouseDown(object sender, MouseEventArgs e)
        {
            J3Down.BackColor = Color.DarkSlateGray;
            if (robotManager.arm.mode == 2)
            {
                velocityIndex = 2;

                Velocity = jsonVelocity.joint[velocityIndex] * -1;
                jointVelocityCmd[2] = Velocity;
                C3.Text = jointVelocityCmd[2].ToString();

                robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);
            }
            else if (robotManager.arm.mode == 3)
            {
                velocityIndex = 2;

                Velocity = jsonVelocity.world[velocityIndex] * _velLevel * -1;
                worldVelocityCmd[2] = Velocity;
                C3.Text = worldVelocityCmd[2].ToString();

                robotManager.arm.SetArmVelCmd(worldVelocityCmd, 0);
            }
            robotManager.arm._sleepCount = 0;
        }
        private void J4Up_MouseDown(object sender, MouseEventArgs e)
        {
            ////J4Up.BackColor = Color.DarkSlateGray;
            //if (robotManager.arm.mode == 2) //Link 4_Not Used
            //{
            //    //velocityIndex = 3;

            //    //Velocity = jsonVelocity.joint[velocityIndex] * _velLevel * 1;
            //    //jointVelocityCmd[3] = Velocity;
            //    //C4.Text = jointVelocityCmd[3].ToString();

            //    //robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);
            //}
            //else if (robotManager.arm.mode == 3)    //Roll
            //{
            //    velocityIndex = 3;

            //    Velocity = jsonVelocity.world[velocityIndex] * _velLevel * 1;
            //    worldVelocityCmd[4] = Velocity;
            //    C4.Text = worldVelocityCmd[4].ToString();

            //    robotManager.arm.SetArmVelCmd(worldVelocityCmd, 0);
            //}
        }
        private void J4Down_MouseDown(object sender, MouseEventArgs e)
        {
            //J4Down.BackColor = Color.DarkSlateGray;
            //if (robotManager.arm.mode == 2) //Link 4 _ Not used
            //{
            //    //velocityIndex = 3;

            //    //Velocity = jsonVelocity.joint[velocityIndex] * _velLevel * -1;
            //    //jointVelocityCmd[3] = Velocity;
            //    ////jointVelocityCmd[3] = 0;    // POSCO ICT
            //    //C4.Text = jointVelocityCmd[3].ToString();

            //    //robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);
            //}
            //else if (robotManager.arm.mode == 3)    //Roll
            //{
            //    velocityIndex = 3;

            //    Velocity = jsonVelocity.world[velocityIndex] * _velLevel * -1;
            //    worldVelocityCmd[4] = Velocity;
            //    C4.Text = worldVelocityCmd[4].ToString();

            //    robotManager.arm.SetArmVelCmd(worldVelocityCmd, 0);
            //}
        }
        private void J5Up_MouseDown(object sender, MouseEventArgs e)
        {
            J5Up.BackColor = Color.DarkSlateGray;
            //if (robotManager.arm.mode == 2)
            {
                velocityIndex = 4;

                Velocity = jsonVelocity.joint[velocityIndex] * 1;
                jointVelocityCmd[4] = Velocity;
                C5.Text = jointVelocityCmd[4].ToString();

                robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);
            }
            //else if (robotManager.arm.mode == 3)
            //{
            //    velocityIndex = 4;

            //    Velocity = jsonVelocity.world[velocityIndex] * _velLevel * 1;
            //    worldVelocityCmd[4] = Velocity;
            //    C5.Text = worldVelocityCmd[4].ToString();

            //    robotManager.arm.SetArmVelCmd(worldVelocityCmd, 0);
            //}
            robotManager.arm._sleepCount = 0;
        }
        private void J5Down_MouseDown(object sender, MouseEventArgs e)
        {
            J5Down.BackColor = Color.DarkSlateGray;
            //if (robotManager.arm.mode == 2)
            {
                velocityIndex = 4;

                Velocity = jsonVelocity.joint[velocityIndex] * -1;
                jointVelocityCmd[4] = Velocity;
                C5.Text = jointVelocityCmd[4].ToString();

                robotManager.arm.SetArmVelCmd(jointVelocityCmd, 0);
            }
            //else if (robotManager.arm.mode == 3)
            //{
            //    velocityIndex = 4;

            //    Velocity = jsonVelocity.world[velocityIndex] * _velLevel * -1;
            //    worldVelocityCmd[4] = Velocity;
            //    C5.Text = worldVelocityCmd[4].ToString();

            //    robotManager.arm.SetArmVelCmd(worldVelocityCmd, 0);
            //}
            robotManager.arm._sleepCount = 0;
        }    
        #endregion

        #region Position control - button
        private void J1Up_Click(object sender, EventArgs e)
        {
            if (robotManager.arm.mode == 5)
            {
                inputCmd[0] = Convert.ToSingle(JT1.Text);
                Pos_Cmd_Joint = inputCmd[0];
                jointPosCmd[0] = Pos_Cmd_Joint + 1;

                if (jointPosCmd[0] >= 130) jointPosCmd[0] = 129.9f;
                else if (jointPosCmd[0] <= -10) jointPosCmd[0] = -9.9f;
                JT1.Text = jointPosCmd[0].ToString();

                Pos_Cmd_Joint = inputCmd[0];
                jointPosCmd[0] = Pos_Cmd_Joint;
                robotManager.arm.SetArmPosCmd(jointPosCmd, jointPosCmd[5], 0);
            }
            else if (robotManager.arm.mode == 8 || robotManager.arm.mode == 9)
            {
                inputCmd[0] = Convert.ToSingle(WT1.Text);
                Pos_Cmd_World = inputCmd[0];
                worldPosCmd[0] = Pos_Cmd_World + _velLevel * 1;

                WT1.Text = worldPosCmd[0].ToString();

                Pos_Cmd_World = inputCmd[0];
                worldPosCmd[0] = Pos_Cmd_World;
                robotManager.arm.SetArmPosCmd(worldPosCmd, jointPosCmd[5], 0);
            }
            //cmd_send_Click(_sender, _e);
        }
        private void J1Down_Click(object sender, EventArgs e)
        {
            if (robotManager.arm.mode == 5)
            {
                inputCmd[0] = Convert.ToSingle(JT1.Text);
                Pos_Cmd_Joint = inputCmd[0];
                jointPosCmd[0] = Pos_Cmd_Joint - 1;

                if (jointPosCmd[0] >= 130.0) jointPosCmd[0] = 129.9f;
                else if (jointPosCmd[0] <= -10) jointPosCmd[0] = -9.9f;
                JT1.Text = jointPosCmd[0].ToString();

                Pos_Cmd_Joint = inputCmd[0];
                jointPosCmd[0] = Pos_Cmd_Joint;

                robotManager.arm.SetArmPosCmd(jointPosCmd, jointPosCmd[5], 0);
                robotManager.arm2.SetArmPosCmd(jointPosCmd, jointPosCmd[5], 0);
                robotManager.arm3.SetArmPosCmd(jointPosCmd, jointPosCmd[5], 0);

            }
            else if (robotManager.arm.mode == 8 || robotManager.arm.mode == 9)
            {
                inputCmd[0] = Convert.ToSingle(WT1.Text);
                Pos_Cmd_World = inputCmd[0];
                worldPosCmd[0] = Pos_Cmd_World - _velLevel * 1;

                WT1.Text = worldPosCmd[0].ToString();

                Pos_Cmd_World = inputCmd[0];
                worldPosCmd[0] = Pos_Cmd_World;
                robotManager.arm.SetArmPosCmd(worldPosCmd, jointPosCmd[5], 0);
                robotManager.arm2.SetArmPosCmd(worldPosCmd, jointPosCmd[5], 0);
            }
            //cmd_send_Click(_sender, _e);
        }
        private void J2Up_Click(object sender, EventArgs e)
        {
            if (robotManager.arm.mode == 5)
            {
                inputCmd[1] = Convert.ToSingle(JT2.Text);
                Pos_Cmd_Joint = inputCmd[1];
                jointPosCmd[1] = Pos_Cmd_Joint + 1;

                if (jointPosCmd[1] >= 144.9) jointPosCmd[1] = 145.0f;
                else if (jointPosCmd[1] <= -35) jointPosCmd[1] = -35f;
                JT2.Text = jointPosCmd[1].ToString();

                Pos_Cmd_Joint = inputCmd[1];
                jointPosCmd[1] = Pos_Cmd_Joint;

                robotManager.arm.SetArmPosCmd(jointPosCmd, jointPosCmd[5], 0);
            }
            else if (robotManager.arm.mode == 8 || robotManager.arm.mode == 9)
            {
                inputCmd[1] = Convert.ToSingle(WT2.Text);
                Pos_Cmd_World = inputCmd[1];
                worldPosCmd[1] = Pos_Cmd_World + _velLevel * 1;

                WT2.Text = worldPosCmd[1].ToString();

                Pos_Cmd_World = inputCmd[1];
                worldPosCmd[1] = Pos_Cmd_World;
                robotManager.arm.SetArmPosCmd(worldPosCmd, jointPosCmd[5], 0);
            }
            //cmd_send_Click(_sender, _e);
        }
        private void J2Down_Click(object sender, EventArgs e)
        {
            if (robotManager.arm.mode == 5)
            {
                inputCmd[1] = Convert.ToSingle(JT2.Text);
                Pos_Cmd_Joint = inputCmd[1];
                jointPosCmd[1] = Pos_Cmd_Joint - 1;

                if (jointPosCmd[1] >= 144.9) jointPosCmd[1] = 145.0f;
                else if (jointPosCmd[1] <= -35) jointPosCmd[1] = -35f;
                JT2.Text = jointPosCmd[1].ToString();

                Pos_Cmd_Joint = inputCmd[1];
                jointPosCmd[1] = Pos_Cmd_Joint;

                robotManager.arm.SetArmPosCmd(jointPosCmd, jointPosCmd[5], 0);
            }
            else if (robotManager.arm.mode == 8 || robotManager.arm.mode == 9)
            {
                inputCmd[1] = Convert.ToSingle(WT2.Text);
                Pos_Cmd_World = inputCmd[1];
                worldPosCmd[1] = Pos_Cmd_World - _velLevel * 1;

                WT2.Text = worldPosCmd[1].ToString();

                Pos_Cmd_World = inputCmd[1];
                worldPosCmd[1] = Pos_Cmd_World;
                robotManager.arm.SetArmPosCmd(worldPosCmd, jointPosCmd[5], 0);
            }
            //cmd_send_Click(_sender, _e);
        }
        private void J3Up_Click(object sender, EventArgs e)
        {
            if (robotManager.arm.mode == 5)
            {
                inputCmd[2] = Convert.ToSingle(JT3.Text);
                Pos_Cmd_Joint = inputCmd[2];
                jointPosCmd[2] = Pos_Cmd_Joint + 1;

                if (jointPosCmd[2] >= 29.9) jointPosCmd[2] = 30.0f;
                else if (jointPosCmd[2] <= -44.9) jointPosCmd[2] = -45f;
                JT3.Text = jointPosCmd[2].ToString();

                Pos_Cmd_Joint = inputCmd[2];
                jointPosCmd[2] = Pos_Cmd_Joint;

                robotManager.arm.SetArmPosCmd(jointPosCmd, jointPosCmd[5], 0);
            }
            else if (robotManager.arm.mode == 8 || robotManager.arm.mode == 9)
            {
                inputCmd[2] = Convert.ToSingle(WT3.Text);
                Pos_Cmd_World = inputCmd[2];
                worldPosCmd[2] = Pos_Cmd_World + _velLevel * 1;

                WT3.Text = worldPosCmd[2].ToString();

                Pos_Cmd_World = inputCmd[2];
                worldPosCmd[2] = Pos_Cmd_World;
                robotManager.arm.SetArmPosCmd(worldPosCmd, jointPosCmd[5], 0);
            }
            //cmd_send_Click(_sender, _e);
        }
        private void J3Down_Click(object sender, EventArgs e)
        {
            if (robotManager.arm.mode == 5)
            {
                inputCmd[2] = Convert.ToSingle(JT3.Text);
                Pos_Cmd_Joint = inputCmd[2];
                jointPosCmd[2] = Pos_Cmd_Joint - 1;

                if (jointPosCmd[2] >= 29.9) jointPosCmd[2] = 30.0f;
                else if (jointPosCmd[2] <= -44.9) jointPosCmd[2] = -45f;
                JT3.Text = jointPosCmd[2].ToString();

                Pos_Cmd_Joint = inputCmd[2];
                jointPosCmd[2] = Pos_Cmd_Joint;

                robotManager.arm.SetArmPosCmd(jointPosCmd, jointPosCmd[5], 0);
            }
            else if (robotManager.arm.mode == 8 || robotManager.arm.mode == 9)
            {
                inputCmd[2] = Convert.ToSingle(WT3.Text);
                Pos_Cmd_World = inputCmd[2];
                worldPosCmd[2] = Pos_Cmd_World - _velLevel * 1;

                WT3.Text = worldPosCmd[2].ToString();

                Pos_Cmd_World = inputCmd[2];
                worldPosCmd[2] = Pos_Cmd_World;
                robotManager.arm.SetArmPosCmd(worldPosCmd, jointPosCmd[5], 0);
            }
            //cmd_send_Click(_sender, _e);
        }


        private void J4Up_Click(object sender, EventArgs e)
        {
            if (robotManager.arm.mode == 5)
            {
                inputCmd[3] = Convert.ToSingle(JT4.Text);
                Pos_Cmd_Joint = inputCmd[3];
                jointPosCmd[3] = Pos_Cmd_Joint + 1;

                //if (jointPosCmd[3] >= 44.9) jointPosCmd[3] = 45.0f;
                //else if (jointPosCmd[3] <= -29.9) jointPosCmd[3] = -30.0f;
                JT4.Text = jointPosCmd[3].ToString();

                Pos_Cmd_Joint = inputCmd[3];
                jointPosCmd[3] = Pos_Cmd_Joint;

                robotManager.arm.SetArmPosCmd(jointPosCmd, jointPosCmd[5], 0);
            }
            else if (robotManager.arm.mode == 8 || robotManager.arm.mode == 9)
            {
                inputCmd[3] = Convert.ToSingle(WT4.Text);
                Pos_Cmd_World = inputCmd[3];
                worldPosCmd[3] = Pos_Cmd_World + 1;

                WT4.Text = worldPosCmd[3].ToString();

                Pos_Cmd_World = inputCmd[3];
                worldPosCmd[3] = Pos_Cmd_World;
                robotManager.arm.SetArmPosCmd(worldPosCmd, jointPosCmd[5], 0);
            }
            cmd_send_Click(_sender, _e);
        }
        private void J4Down_Click(object sender, EventArgs e)
        {
            if (robotManager.arm.mode == 5)
            {
                inputCmd[3] = Convert.ToSingle(JT4.Text);
                Pos_Cmd_Joint = inputCmd[3];
                jointPosCmd[3] = Pos_Cmd_Joint - 1;

                //if (jointPosCmd[3] >= 44.9) jointPosCmd[3] = 45.0f;
                //else if (jointPosCmd[3] <= -29.9) jointPosCmd[3] = -30.0f;
                JT4.Text = jointPosCmd[3].ToString();

                Pos_Cmd_Joint = inputCmd[3];
                jointPosCmd[3] = Pos_Cmd_Joint;

                robotManager.arm.SetArmPosCmd(jointPosCmd, jointPosCmd[5], 0);
            }
            else if (robotManager.arm.mode == 8 || robotManager.arm.mode == 9)
            {
                inputCmd[3] = Convert.ToSingle(WT4.Text);
                Pos_Cmd_World = inputCmd[3];
                worldPosCmd[3] = Pos_Cmd_World - 1;

                WT4.Text = worldPosCmd[3].ToString();

                Pos_Cmd_World = inputCmd[3];
                worldPosCmd[3] = Pos_Cmd_World;
                robotManager.arm.SetArmPosCmd(worldPosCmd, jointPosCmd[5], 0);
            }
            cmd_send_Click(_sender, _e);
        }
        private void J5Up_Click(object sender, EventArgs e)
        {
            if (robotManager.arm.mode == 5)
            {
                inputCmd[4] = Convert.ToSingle(JT5.Text);
                Pos_Cmd_Joint = inputCmd[4];
                jointPosCmd[4] = Pos_Cmd_Joint + 1;

                JT5.Text = jointPosCmd[4].ToString();

                Pos_Cmd_Joint = inputCmd[4];
                jointPosCmd[4] = Pos_Cmd_Joint;

                robotManager.arm.SetArmPosCmd(jointPosCmd, jointPosCmd[5], 0);
            }
            else if (robotManager.arm.mode == 8 || robotManager.arm.mode == 9)
            {
                inputCmd[4] = Convert.ToSingle(WT5.Text);
                Pos_Cmd_World = inputCmd[4];
                worldPosCmd[4] = Pos_Cmd_World + 1;

                WT5.Text = worldPosCmd[4].ToString();

                Pos_Cmd_World = inputCmd[4];
                worldPosCmd[4] = Pos_Cmd_World;
                robotManager.arm.SetArmPosCmd(worldPosCmd, jointPosCmd[5], 0);
            }
            //cmd_send_Click(_sender, _e);
        }
        private void J5Down_Click(object sender, EventArgs e)
        {
            if (robotManager.arm.mode == 5)
            {
                inputCmd[4] = Convert.ToSingle(JT5.Text);
                Pos_Cmd_Joint = inputCmd[4];
                jointPosCmd[4] = Pos_Cmd_Joint - 1;

                JT5.Text = jointPosCmd[4].ToString();

                Pos_Cmd_Joint = inputCmd[4];
                jointPosCmd[4] = Pos_Cmd_Joint;

                robotManager.arm.SetArmPosCmd(worldPosCmd, jointPosCmd[5], 0);
            }
            else if (robotManager.arm.mode == 8 || robotManager.arm.mode == 9)
            {
                inputCmd[4] = Convert.ToSingle(WT5.Text);
                Pos_Cmd_World = inputCmd[4];
                worldPosCmd[4] = Pos_Cmd_World - 1;

                WT5.Text = worldPosCmd[4].ToString();

                Pos_Cmd_World = inputCmd[4];
                worldPosCmd[4] = Pos_Cmd_World;
                robotManager.arm.SetArmPosCmd(worldPosCmd, jointPosCmd[5], 0);
            }
            //cmd_send_Click(_sender, _e);
        }

        private void PW_a_Click(object sender, EventArgs e)
        {
            inputPW += 'a';
            pwCount++;
            Debug.WriteLine(inputPW);
        }
        private void PW_b_Click(object sender, EventArgs e)
        {
            inputPW += 'b';
            pwCount++;
            Debug.WriteLine(inputPW); 
        }
        private void PW_c_Click(object sender, EventArgs e)
        {
            inputPW += 'c';
            pwCount++;
            Debug.WriteLine(inputPW);
        }
        private void PW_d_Click(object sender, EventArgs e)
        {
            inputPW += 'd';
            pwCount++;
            Debug.WriteLine(inputPW);
        }
        #endregion

        #region _ Label Update
        void monitoringLB_update(int _index)
        {
            if (_index == 0 || _index == 2 || _index == 5)
            {
                lbJ1.Text =  "Axis 1";
                lbJ2.Text = "Axis 2";
                lbJ3.Text = "Axis 3";
                lbJ4.Text = "Axis 4";
                lbJ5.Text = "Axis 5";
                //lbJ6.Text = "Axis 6";

                lb_E1_cur.Text = lb_E2_cur.Text = lb_E3_cur.Text = "deg";
            }
            else if (_index == 1 || _index == 3 || _index == 4)
            {
                lbJ1.Text = "X";
                lbJ2.Text = "Y";
                lbJ3.Text = "Z";
                //lbJ4.Text = /*lb_Roll_target.Text =*/ chartLbRoll.Text = "Roll";
                lbJ5.Text = "TOOL";
                //lbJ6.Text = /*lb_Yaw_target.Text =*/ chartLbYaw.Text = "Yaw";

                //lb_X_Tar_unit.Text = lb_Y_Tar_unit.Text = lb_Z_Tar_unit.Text = "mm";
                //lb_X_Cur_unit.Text = lb_Y_Cur_unit.Text = lb_Z_Cur_unit.Text = "mm";
                lb_E1_cur.Text = lb_E2_cur.Text = lb_E3_cur.Text = "mm";
                //lb_E1_pro.Text = lb_E2_pro.Text = lb_E3_pro.Text = "mm";
            }
        }
        #endregion

        #region CAN Field
        private void btnCAN_Click(object sender, EventArgs e)
        {
            canON = !canON;
            if (canON)
            {
                //gr_CAN.Visible = true;
                robotManager.arm.SendSetCanConnect(true);
                btnCAN.BackColor = Color.DarkCyan;
            }
            else
            {
                //gr_CAN.Visible = false;
                robotManager.arm.SendSetCanConnect(false);
                btnCAN.BackColor = Color.FromArgb(50, 54, 58);
            }
        }
        private void CANtx_Click(object sender, EventArgs e)
        {
            CAN_Serail.Text = _canSerialCmd + CAN_Serail.Text;
            robotManager.arm.SendCanSerialDataUDP(canID, CAN_Serail.Text);
            txtCanData.Text = robotManager.arm._canUDPRx.rxData.ToString();
        }
        private void CANrx_Click(object sender, EventArgs e)
        {
            txtCanData.Text = robotManager.arm._canUDPRx.rxData.ToString();
        }
        private void HAClist_SelectedIndexChanged(object sender, EventArgs e)
        {
            string val = HAClist.SelectedItem.ToString();

            switch (val)
            {
                case "Device 1":
                    canID = 1;
                    robotManager.arm._canUDP.Id = 1;
                    break;
                case "Device 2":
                    canID = 2;
                    robotManager.arm._canUDP.Id = 2;
                    break;
                case "Device 3":
                    canID = 3;
                    robotManager.arm._canUDP.Id = 3;
                    break;
                case "Device 4":
                    canID = 4;
                    robotManager.arm._canUDP.Id = 4;
                    break;
                case "Device 5":
                    canID = 5;
                    robotManager.arm._canUDP.Id = 5;
                    break;
                case "Device 6":
                    canID = 6;
                    robotManager.arm._canUDP.Id = 6;
                    break;
                case "Device 7":
                    canID = 7;
                    robotManager.arm._canUDP.Id = 7;
                    break;
            }
        }

        private void checkBoxSimulMod_CheckedChanged(object sender, EventArgs e)
        {
            if (robotManager.arm.mode != 5)//Joint_pos Mode 가 적용 되어있지 않을때
            {
                mode_List.SelectedIndex = 2;// Joint_pos Mode
                robotManager.arm.mode = 5;
            }
        }

        private void BtnTKVopen_Click(object sender, EventArgs e) //TKV File loading
        {
            TkvPlay.Upload();
            TKVPlayBar.Value = 0;
            TKVPlayBar.Maximum = TkvPlay.totalLines;
        }

        private void BtnPlay_Click(object sender, EventArgs e)//TKV File Play, pause
        {
            TkvPlay.playrate = (int)(1000 / NumUDPlayspeed.Value);
            switch (TkvPlay.State_num)
            {
                case 0:
                    TkvPlay.Play();
                    BtnPlay.Text = "⏸️";
                    break;
                case 1:
                    TkvPlay.Pause();
                    BtnPlay.Text = "▶";
                    break;
                case 2:
                    TkvPlay.Play();
                    BtnPlay.Text = "⏸️";
                    break;
                default:
                    TkvPlay.Pause();
                    BtnPlay.Text = "▶";
                    break;
            }
        }

        private void BtnStopPlay_Click(object sender, EventArgs e)//TKV File stop
        {
            TKVPlayBar.Value = 0;
            TkvPlay.Stop();   
            BtnPlay.Text = "▶";
        }

        private void MP_Load(object sender, EventArgs e)
        {

        }

        private void hacIndex_SelectedIndexChanged(object sender, EventArgs e)
        {
            subIndex.Items.Clear();

            string val = hacIndex.SelectedItem.ToString();

            switch (val)
            {
                case " ":
                    subIndex.Enabled = false;
                    CAN_Serail.Text = "";
                    _canSerialCmd = "";
                    break;
                case "Write Gain":
                    canIndex = 0;
                    _canSerialCmd = "W";
                    subIndex.Enabled = false;
                    break;
                case "Sign of Output":
                    canIndex = 1;
                    subIndex.Enabled = true;
                    subIndex.Items.Add("Position");
                    subIndex.Items.Add("Torque");
                    break;
                case "Servo v/v Out":
                    canIndex = 2;
                    _canSerialCmd = "S";
                    subIndex.Enabled = false;
                    break;
                case "Start Mode":
                    canIndex = 3;
                    break;
                case "Position Gain":
                    canIndex = 4;
                    subIndex.Enabled = true;
                    subIndex.Items.Add("Proportional");
                    subIndex.Items.Add("Integral");
                    subIndex.Items.Add("Differential");
                    break;
                case "Torque Gain":
                    canIndex = 5;
                    subIndex.Enabled = true;
                    subIndex.Items.Add("Proportional");
                    subIndex.Items.Add("Integral");
                    subIndex.Items.Add("Differential");
                    break;
                case "Compliance Gain":
                    canIndex = 6;
                    subIndex.Enabled = true;
                    subIndex.Items.Add("Stiffness");
                    subIndex.Items.Add("Friction");
                    subIndex.Items.Add("Transition deflection");
                    subIndex.Items.Add("Offset");
                    break;
                case "Velocity Gain":
                    canIndex = 7;
                    subIndex.Enabled = true;
                    subIndex.Items.Add("Proportional");
                    subIndex.Items.Add("Integral");
                    subIndex.Items.Add("Differential");
                    break;
                case "Error Sum Clear":
                    canIndex = 8;
                    subIndex.Enabled = false;
                    break;
                case "Set Min/Max Position":
                    canIndex = 9;
                    subIndex.Enabled = true;
                    subIndex.Items.Add("PosMax");
                    subIndex.Items.Add("PosMin");
                    break;
                case "Control Mode":
                    canIndex = 14;
                    break;
                case "Target Position":
                    canIndex = 15;
                    subIndex.Enabled = false;
                    break;
                case "Actual Position":
                    canIndex = 16;
                    subIndex.Enabled = false;
                    break;
                case "Target Velocity":
                    canIndex = 17;
                    subIndex.Enabled = false;
                    break;
                case "Actual Velocity":
                    canIndex = 18;
                    subIndex.Enabled = false;
                    break;
                case "Raw Joint Angle":
                    canIndex = 21;
                    subIndex.Enabled = false;
                    break;
            }
        }
        private void subIndex_SelectedIndexChanged(object sender, EventArgs e)
        {
            string val = subIndex.SelectedItem.ToString();

            switch (val)
            {
                case "Position":
                    //canSubIndex = 0;
                    _canSerialCmd = "P";
                    break;
                case "Torque":
                    //canSubIndex = 1;
                    _canSerialCmd = "T";
                    break;
                case "Proportional":
                    //canSubIndex = 0;
                    if (hacIndex.Text == "Position Gain") _canSerialCmd = "P";
                    else if (hacIndex.Text == "Torque Gain") _canSerialCmd = "p";
                    break;
                case "Integral":
                    //canSubIndex = 1;
                    if (hacIndex.Text == "Position Gain") _canSerialCmd = "I";
                    else if (hacIndex.Text == "Torque Gain") _canSerialCmd = "i";
                    break;
                case "Differential":
                    //canSubIndex = 2;
                    if (hacIndex.Text == "Position Gain") _canSerialCmd = "D";
                    else if (hacIndex.Text == "Torque Gain") _canSerialCmd = "d";
                    break;
                case "Stiffness":
                    //canSubIndex = 0;
                    _canSerialCmd = "X";
                    break;
                case "Friction":
                    //canSubIndex = 1;
                    _canSerialCmd = "M";
                    break;
                case "Transition deflection":
                    //canSubIndex = 2;
                    _canSerialCmd = "m";
                    break;
                case "Offset":
                    //canSubIndex = 3;
                    _canSerialCmd = "o";
                    break;
                case "PosMax":
                    //canSubIndex = 0;
                    _canSerialCmd = "N";
                    break;
                case "PosMin":
                    //canSubIndex = 1;
                    _canSerialCmd = "n";
                    break;
            }
        }
        #endregion

        /// <summary>
        /// Json 정보 저장
        /// </summary>
        #region Json
        //설명: 소캣 정보를 Json 형식으로 저장 및 불러오기
        private void LoadJsonScoket(string fileName)
        {
            //파일 확인 및 불러오기
            if (File.Exists(fileName))
                socketInfo = JsonFileIO.Load<JsonSocket>(fileName);

            //파일이 존재하지 않는 경우 아래 설정 값으로 설정 및 파일 생성.
            if (socketInfo == null)
            {
                socketInfo = new JsonSocket();

                socketInfo.name = new List<string>();
                socketInfo.IP = new List<string>();
                socketInfo.port = new List<int>();

                socketInfo.name.Add("Arm");
                socketInfo.name.Add("UIPC");
                socketInfo.name.Add("Arm2");
                socketInfo.name.Add("Arm3");
                socketInfo.name.Add("Arm4");

                socketInfo.IP.Add("192.168.0.101");
                socketInfo.IP.Add("192.168.0.100");
                socketInfo.IP.Add("192.168.0.102");
                socketInfo.IP.Add("192.168.0.103");
                socketInfo.IP.Add("192.168.0.104");

                socketInfo.port.Add(4101); //(int)index.Arm
                socketInfo.port.Add(4010); //(int)index.UIPC
                socketInfo.port.Add(4102); //(int)index.Arm2
                socketInfo.port.Add(4103); //(int)index.Arm3
                socketInfo.port.Add(4104); //(int)index.Arm4

                //파일로 저장.
                JsonFileIO.Save(socketInfo, fileName);
            }
        }
        private void LoadJsonVelocity(string fileName)
        {
            if (File.Exists(fileName))
                jsonVelocity = JsonFileIO.Load<JsonVelocity>(fileName);

            //파일이 존재하지 않는 경우 아래 설정 값으로 설정 및 파일 생성.
            if (jsonVelocity == null)
            {
                jsonVelocity = new JsonVelocity();
                jsonVelocity.joint = new List<float>();
                jsonVelocity.world = new List<float>();

                //jsonVelocity.jointDir = new List<int>();
                //jsonVelocity.mobileDir = new List<int>();

                //jsonVelocity.joint.Add(5);
                //jsonVelocity.joint.Add(10);
                //jsonVelocity.joint.Add(20);
                //jsonVelocity.joint.Add(30);
                //jsonVelocity.joint.Add(0);

                //jsonVelocity.world.Add(20);
                //jsonVelocity.world.Add(50);
                //jsonVelocity.world.Add(100);
                //jsonVelocity.world.Add(200);
                //jsonVelocity.world.Add(0);
                jsonVelocity.joint.Add(0.1f);
                jsonVelocity.joint.Add(0.1f);
                jsonVelocity.joint.Add(0.1f);
                jsonVelocity.joint.Add(0.1f);
                jsonVelocity.joint.Add(0.1f);
                jsonVelocity.joint.Add(0.1f);

                jsonVelocity.world.Add(0.1f);
                jsonVelocity.world.Add(0.1f);
                jsonVelocity.world.Add(0.1f);
                jsonVelocity.world.Add(0.1f);
                jsonVelocity.world.Add(0.1f);
                jsonVelocity.world.Add(0.1f);
                //파일로 저장.
                JsonFileIO.Save(jsonVelocity, fileName);
            }
        }
        private void LoadPosition(string fileName)
        {
            if (File.Exists(fileName))
                _position = JsonFileIO.Load<Position>(fileName);

            //파일이 존재하지 않는 경우 아래 설정 값으로 설정 및 파일 생성.
            if (_position == null)
            {
                _position = new Position();

                _position.HOME = new List<float>();
                _position.PARKING = new List<float>();
                _position.Target1 = new List<float>();
                _position.Target2 = new List<float>();
                _position.Target3 = new List<float>();
                _position.Target4 = new List<float>();
                _position.Target5 = new List<float>();
                _position.Target6 = new List<float>();
                _position.Target7 = new List<float>();
                _position.Target8 = new List<float>();
                _position.Target9 = new List<float>();
                _position.Target10 = new List<float>();
                _position.Target11 = new List<float>();
                _position.Target12 = new List<float>();
                _position.Target13 = new List<float>();
                _position.Target14 = new List<float>();
                _position.Target15 = new List<float>();
                _position.Target16 = new List<float>();
                _position.Target17 = new List<float>();
                _position.Target18 = new List<float>();
                _position.Target19 = new List<float>();
                _position.Target20 = new List<float>();
                _position.Target21 = new List<float>();
               
                _position.HOME.Add(0.01f);
                _position.HOME.Add(49.23f);
                _position.HOME.Add(98.46f);
                _position.HOME.Add(40.77f);
                _position.HOME.Add(0.01f);
                _position.HOME.Add(0.01f);
                _position.HOME.Add(0.5f);
                _position.HOME.Add(0.5f);

                _position.PARKING.Add(0.01f);
                _position.PARKING.Add(29.6f);
                _position.PARKING.Add(118.75f);
                _position.PARKING.Add(0.65f);
                _position.PARKING.Add(0.01f);
                _position.PARKING.Add(0.01f);
                _position.PARKING.Add(0.5f);
                _position.PARKING.Add(0.5f);

                _position.Target1.Add(36.36f);
                _position.Target1.Add(35.81f);
                _position.Target1.Add(71.63f);
                _position.Target1.Add(54.19f);
                _position.Target1.Add(36.36f);
                _position.Target1.Add(0.01f);
                _position.Target1.Add(0.5f);
                _position.Target1.Add(0.5f);

                _position.Target2.Add(-36.36f);
                _position.Target2.Add(35.81f);
                _position.Target2.Add(71.63f);
                _position.Target2.Add(54.19f);
                _position.Target2.Add(-36.36f);
                _position.Target2.Add(0.01f);
                _position.Target2.Add(0.5f);
                _position.Target2.Add(0.5f);

                _position.Target3.Add(0.01f);
                _position.Target3.Add(60.48f);
                _position.Target3.Add(121.0f);
                _position.Target3.Add(29.52f);
                _position.Target3.Add(0.01f);
                _position.Target3.Add(0.01f);
                _position.Target3.Add(0.5f);
                _position.Target3.Add(0.5f);

                _position.Target4.Add(0.01f);
                _position.Target4.Add(35.58f);
                _position.Target4.Add(71.16f);
                _position.Target4.Add(54.42f);
                _position.Target4.Add(0.01f);
                _position.Target4.Add(0.01f);
                _position.Target4.Add(0.5f);
                _position.Target4.Add(0.5f);

                _position.Target5.Add(0.01f);
                _position.Target5.Add(63.97f);
                _position.Target5.Add(93.83f);
                _position.Target5.Add(60.14f);
                _position.Target5.Add(0.01f);
                _position.Target5.Add(0.01f);
                _position.Target5.Add(0.5f);
                _position.Target5.Add(0.5f);

                _position.Target6.Add(0.01f);
                _position.Target6.Add(35.58f);
                _position.Target6.Add(71.16f);
                _position.Target6.Add(54.42f);
                _position.Target6.Add(0.01f);
                _position.Target6.Add(0.01f);
                _position.Target6.Add(0.5f);
                _position.Target6.Add(0.5f);

                _position.Target7.Add(0.01f);
                _position.Target7.Add(63.97f);
                _position.Target7.Add(93.83f);
                _position.Target7.Add(60.14f);
                _position.Target7.Add(0.01f);
                _position.Target7.Add(0.01f);
                _position.Target7.Add(0.5f);
                _position.Target7.Add(0.5f);

                _position.Target8.Add(0.01f);
                _position.Target8.Add(63.97f);
                _position.Target8.Add(93.83f);
                _position.Target8.Add(60.14f);
                _position.Target8.Add(0.01f);
                _position.Target8.Add(0.01f);
                _position.Target8.Add(0.5f);
                _position.Target8.Add(0.5f);

                _position.Target9.Add(0.01f);
                _position.Target9.Add(63.97f);
                _position.Target9.Add(93.83f);
                _position.Target9.Add(60.14f);
                _position.Target9.Add(0.01f);
                _position.Target9.Add(0.01f);
                _position.Target9.Add(0.5f);
                _position.Target9.Add(0.5f);

                _position.Target10.Add(0.01f);
                _position.Target10.Add(63.97f);
                _position.Target10.Add(93.83f);
                _position.Target10.Add(60.14f);
                _position.Target10.Add(0.01f);
                _position.Target10.Add(0.01f);
                _position.Target10.Add(0.5f);
                _position.Target10.Add(0.5f);

                _position.Target11.Add(0.01f);
                _position.Target11.Add(63.97f);
                _position.Target11.Add(93.83f);
                _position.Target11.Add(60.14f);
                _position.Target11.Add(0.01f);
                _position.Target11.Add(0.01f);
                _position.Target11.Add(0.5f);
                _position.Target11.Add(0.5f);

                _position.Target12.Add(0.01f);
                _position.Target12.Add(63.97f);
                _position.Target12.Add(93.83f);
                _position.Target12.Add(60.14f);
                _position.Target12.Add(0.01f);
                _position.Target12.Add(0.01f);
                _position.Target12.Add(0.5f);
                _position.Target12.Add(0.5f);

                _position.Target13.Add(0.01f);
                _position.Target13.Add(63.97f);
                _position.Target13.Add(93.83f);
                _position.Target13.Add(60.14f);
                _position.Target13.Add(0.01f);
                _position.Target13.Add(0.01f);
                _position.Target13.Add(0.5f);
                _position.Target13.Add(0.5f);

                _position.Target14.Add(0.01f);
                _position.Target14.Add(63.97f);
                _position.Target14.Add(93.83f);
                _position.Target14.Add(60.14f);
                _position.Target14.Add(0.01f);
                _position.Target14.Add(0.01f);
                _position.Target14.Add(0.5f);
                _position.Target14.Add(0.5f);

                _position.Target15.Add(0.01f);
                _position.Target15.Add(63.97f);
                _position.Target15.Add(93.83f);
                _position.Target15.Add(60.14f);
                _position.Target15.Add(0.01f);
                _position.Target15.Add(0.01f);
                _position.Target15.Add(0.5f);
                _position.Target15.Add(0.5f);

                _position.Target16.Add(0.01f);
                _position.Target16.Add(63.97f);
                _position.Target16.Add(93.83f);
                _position.Target16.Add(60.14f);
                _position.Target16.Add(0.01f);
                _position.Target16.Add(0.01f);
                _position.Target16.Add(0.5f);
                _position.Target16.Add(0.5f);

                _position.Target17.Add(0.01f);
                _position.Target17.Add(63.97f);
                _position.Target17.Add(93.83f);
                _position.Target17.Add(60.14f);
                _position.Target17.Add(0.01f);
                _position.Target17.Add(0.01f);
                _position.Target17.Add(0.5f);
                _position.Target17.Add(0.5f);

                _position.Target18.Add(0.01f);
                _position.Target18.Add(63.97f);
                _position.Target18.Add(93.83f);
                _position.Target18.Add(60.14f);
                _position.Target18.Add(0.01f);
                _position.Target18.Add(0.01f);
                _position.Target18.Add(0.5f);
                _position.Target18.Add(0.5f);

                _position.Target19.Add(0.01f);
                _position.Target19.Add(63.97f);
                _position.Target19.Add(93.83f);
                _position.Target19.Add(60.14f);
                _position.Target19.Add(0.01f);
                _position.Target19.Add(0.01f);
                _position.Target19.Add(0.5f);
                _position.Target19.Add(0.5f);

                _position.Target20.Add(0.01f);
                _position.Target20.Add(63.97f);
                _position.Target20.Add(93.83f);
                _position.Target20.Add(60.14f);
                _position.Target20.Add(0.01f);
                _position.Target20.Add(0.01f);
                _position.Target20.Add(0.5f);
                _position.Target20.Add(0.5f);

                _position.Target21.Add(0.01f);
                _position.Target21.Add(63.97f);
                _position.Target21.Add(93.83f);
                _position.Target21.Add(60.14f);
                _position.Target21.Add(0.01f);
                _position.Target21.Add(0.01f);
                _position.Target21.Add(0.5f);
                _position.Target21.Add(0.5f);

                //파일로 저장.
                JsonFileIO.Save(_position, fileName);
            }
        }
        //private void LoadPosGain(string fileName)
        //{
        //    if (File.Exists(fileName))
        //        _posGain = JsonFileIO.Load<Pos_Gain>(fileName);

        //    //파일이 존재하지 않는 경우 아래 설정 값으로 설정 및 파일 생성.
        //    if (_posGain == null)
        //    {
        //        _posGain = new Pos_Gain();

        //        _posGain.ConveGain = new List<float>();
        //        _posGain.Surge = new List<float>();
        //        _posGain.Sway = new List<float>();
        //        _posGain.Heave = new List<float>();
        //        _posGain.Roll = new List<float>();
        //        _posGain.Pitch = new List<float>();
        //        _posGain.Yaw = new List<float>();
        //        //_posGain.th_7 = new List<float>();

        //        _posGain.ConveGain.Add(0.7f);
        //        _posGain.Surge.Add(20.0f);
        //        _posGain.Sway.Add(20.0f);
        //        _posGain.Heave.Add(20.0f);
        //        _posGain.Roll.Add(0.157f);
        //        _posGain.Pitch.Add(0.157f);
        //        _posGain.Yaw.Add(0.157f);
        //        //_posGain.th_7.Add(0.157f);

        //        //파일로 저장.
        //        JsonFileIO.Save(_posGain, fileName);
        //    }
        //}
        #endregion

        private void btnExit_Click(object sender, EventArgs e)
        {
            udpclienttkv.UDPwControlClose();
            server.Close();
            Environment.Exit(0);
            Process.GetCurrentProcess().Kill();
            this.Close();
        }
    }
}
