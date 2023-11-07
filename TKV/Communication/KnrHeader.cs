
#define KNR_LCU_COMMON_H
#define KNR_LCU_COMMUNICATION_H
#define KNR_RM_COMMUNICATION_H
#define MANIPULATORERROR_H
#define KNR_LCU_CAN_H

using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.InteropServices;

//�߿䵵: �ſ����
//����: LCU�� ����ϱ� ���� �ʿ��� ��Ŷ ���
namespace KnrHeader
{
    /////////////////////////////////////////////////////////////////////////////
    //
    //PACKET TYPE DECLARATION
    using Bool_t = UInt16;
    using uint32_t = UInt32;
    using uint16_t = UInt16;
    using uint8_t = Byte;
    using int32_t = Int32;
    using int16_t = Int16;
    using byte_t = Char;

    //����: KNR_LCU_COMMON_H ���� ���� �� Checksum ó�� ���.
    //#if XXX ~ #endif: ���� #define XXX �� ����Ǿ� �ִٸ� ���θ� ���. ����Ǿ� �����ʴٸ� ���θ� �ּ�ó��.
#if KNR_LCU_COMMON_H
    public class LcuCommon
    {
        /////////////////////////////////////////////////////////////////////////////
        //
        // *EDITABLE Define
        // SELECT THIS DEVICE NAME
        //public const string THIS_DEVICE = "HEADER_LCU1"; 
        //public const string THIS_DEVICE = "HEADER_LCU2";
        //public const string THIS_DEVICE = "HEADER_LCU3";
        //public const string THIS_DEVICE = "HEADER_LCU4";
        public const string THIS_DEVICE = "HEADER_RM"; //����̽� ����: LCU���� �ش� ����� ����ϱ� ���� ���.
        //public const string THIS_DEVICE = "HEADER_TE";
        //public const string THIS_DEVICE = "HEADER_PC1";
        //public const string THIS_DEVICE = "HEADER_PC2";

        public const int ROBOT_AXIS_CNT_CURR = 6;

        /////////////////////////////////////////////////////////////////////////////
        //
        // Common Define 
        public const int PACKET_HEADER_LEN = 8; //������ ����: checksum(1)/sender(1)/receiver(1)/ack(1)/code(1)/value(1)/datasize(2)
        public const int PACKET_HEADER_EXCEPT_CHECKSUM_LEN = 7; // header size except 'checksum(1)'
        public const int PACKET_DATA_LEN = 1016; //�ִ� ������ ����
        public const int PACKET_NULL_LEN = 1; //������ �������� NULL ����=0�� ����.  
        public const int PACKET_LEN = PACKET_HEADER_LEN + PACKET_DATA_LEN; //�� ��� + ������ ����

        public const int HEADER_LCU1 = 1; //Bin..0000  0001
        public const int HEADER_LCU2 = 2; //Bin..0000  0010
        public const int HEADER_LCU3 = 4; //Bin..0000  0100
        public const int HEADER_LCU4 = 8; //Bin..0000  1000
        public const int HEADER_LCU5 = 9; //Bin..0000  1001


        public const int HEADER_RM = 16; //Bin  0001  0000 //PLC
        public const int HEADER_TE = 32; //Bin  0010  0000
        public const int HEADER_PC1 = 64; //Bin  0001  0000 //팬던트
        public const int HEADER_PC2 = 128; //Bin  0010  0000

        public const int HEADER_MULTI = 255; //Bin  1111  1111

        public const int NONE = 0; //������
        public const int ACK = 1;  //����

        public const int TURN_OFF = 0;
        public const int TURN_ON = 1;

        public const int LCU_ANALOG_PORT_CNT_MAX = 8; // Num of analog port
        public const int LCU_DIGITAL_PORT_CNT_MAX = 6; // Num of digital port
        public const int ROBOT_AXIS_CNT_MAX = 6; // Manipulator _ Num of actuator  //MAX_ROBOT_AXIS_CNT

        public const int MOBILE_ENCODER_CNT_MAX = 8; //����� ���ڴ� ����
        public const int MOBILE_SERVO_CNT_MAX = 8; //����� ������������ �޴� ����̽� ����
        public const int MOBILE_PRES_CNT_MAX = 8; //����� �з� ���� ����
        public const int MOBILE_DO_CNT_MAX = 1; //����� Digital output ����

        public const int LCU_ANALOG_IO_MIN = 0;
        public const int LCU_ANALOG_IO_MAX = 5;

        public const int JOB_NAME_SIZE = 512; //JOB

        //����: ��Ŷ�� ��ȿ�� Ȯ���ϱ� ����. ��Ŷ�� �����ŭ ��� ����Ʈ�� �� ����Ʈ ������ ��� ����.
        public static byte CalcChecksum(LcuCommunication.t_lcu_packet_const_size constSizePacket, int size)
        {
            byte checksum = 0;

            try
            {
                byte[] bytes = TypeConvert.StructToBytes(constSizePacket, size);

                for (int i = 0; i < size; i++)
                {
                    checksum += bytes[i];
                }
            }
            catch
            {
                Debug.WriteLine("StructToBytes Error!");
            }

            Debug.WriteLine("Check sum = " + checksum);
            return checksum;
        }

        //@Overrride
        //����: ��Ŷ�� ��ȿ�� Ȯ���ϱ� ����. Bytes�Է��� ���.
        public static byte CalcChecksum(byte[] bytes)
        {
            byte checksum = 0;

            try
            {
                for (int i = 0; i < bytes.Length; i++)
                {
                    checksum += bytes[i];
                }
            }
            catch
            {
                Debug.WriteLine("StructToBytes Error!");
            }

            return checksum;
        }

        //����: Checksum�� ������ ��� ������ ũ��(Header�� ũ�⿡ ������ ũ�⸦ ����).
        public static UInt16 GetPacketCheckSumSize(UInt16 dataSize)
        {
            return (UInt16)(PACKET_HEADER_EXCEPT_CHECKSUM_LEN + dataSize);
        }
    }
#endif

    //����: KNR_LCU_COMMUNICATION_H LCU���� ���� �� �۽��Ҷ� ����ϴ� ��Ŷ.
#if KNR_LCU_COMMUNICATION_H
    public class LcuCommunication
    {
        public const int nLCU_VERSION = 230;
        //public const string LCU_VERSION = "2.3.0"; //major(LCU ver).minor(Header ver).patch
        //public const string LCU_BUILD = "2017.12.27";
        public const string LCU_VERSION = "pro 1.0.0";
        public const string LCU_BUILD = "2022.07.08";

        /////////////////////////////////////////////////////////////////////////////
        //
        //LCU SERVICE LIST (100 ~ )
        public const int LCU_SVC_ERROR = 101;
        public const int LCU_SVC_AI_STATE = 102;
        public const int LCU_SVC_AO_STATE = 103;
        public const int LCU_SVC_AOUT_CMD = 104;
        public const int LCU_SVC_DIO_DIR = 105;
        public const int LCU_SVC_DIO_STATE = 106;
        public const int LCU_SVC_DIO_SET = 107;
        public const int LCU_SVC_TEMP = 108; //LCU �µ�(optional)
        public const int LCU_SVC_CAN_CONNECT = 109; //CAN ����
        public const int LCU_SVC_CAN_RECEIVE = 110; //CAN �ޱ�(���� SW��)
        public const int LCU_SVC_CAN_SEND = 111; //CAN ������(���� SW����)
        public const int LCU_SVC_DATA_REPLY = 112; //LCU�� �ֱ������� �����ϴ� ����͸� ������ �ֱ�(�ð�) ����
        public const int LCU_SVC_KEEP_TIME = 121; //LCU�� �Էµ� ������ �ӵ������� �����ϴ� �ð�
        public const int LCU_SVC_MOTION_START = 122; //outdated MP3 code only used(������)
        public const int LCU_SVC_MODE = 123; //�κ��� ���� ��� ����
        public const int LCU_SVC_SV_STATE = 124; //������� ��������(���������� ����)
        public const int LCU_SVC_VEL_COMMAND = 125; //�ӵ� ����
        public const int LCU_SVC_MONITORING_DATA = 126; //����͸� ������(Joint/World/Tool)
        public const int LCU_SVC_CLEAR_ERRSUM = 127; //PID ������ ����(I) ������ Ŭ����. 
        public const int LCU_SVC_GRIP = 128; //�κ����� �׸��� ����
        public const int LCU_SVC_TCP = 129; //Tool Center Point ����(���� ���̳� ���⿡ ���� ���� �ⱸ�� ����)
        public const int LCU_SVC_COMPLIANCE = 130; //���ö��̾�(���������� ����)
        public const int LCU_SVC_LIMIT_ERROR = 131; //���� ����Ʈ(����) ������ �߻� ��Ű�� ���� ����
        public const int LCU_SVC_TEMPERATURE_ERROR = 132; //������
        public const int LCU_SVC_POSDIFF_MONITOR = 133; //actual position(���ڴ� ��)�� ����͸��ϰų� �� �� ����.
        public const int LCU_SVC_SET_CONTROL = 134; //�ٸ� PC�� ���� LCU�� ������� ���� �ö� ���.

        public const int LCU_SVC_POS_COMMAND = 135; //ver.2.2.1 //LCU ���ο��� �ڵ� ��ġ ��� �ϴ� ��� ���.
        public const int LCU_SVC_IP_CHANGE = 136; //ver.2.3.0 //LCU�� ��� PC(RM)�� IP�� �����ϴ� ��� ���.
        public const int LCU_SVC_LIMIT_CHANGE = 137; // ver.2.3.1 //LCU�� SW ����Ʈ�� �����ϴ� ��� ���.
        public const int LCU_SVC_SENDER_CHANGE = 138; // ver.2.3.1 //LCU���� ������ ��Ŷ�� ����� Sender�� �����ϴ� ��� ���.

        public const int LCU_SVC_SET_SIMULATOR_ON = 140; // LCU pro - Simulator mode ON ����
        public const int LCU_SVC_SET_SIMULATOR_OFF = 141; // LCU pro - Simulator mode OFF ����

        public const int LCU_SVC_POS_GAIN = 145;        // LCU pro - Position control ���� �ִ� ���żӵ� �� �ִ� �ӵ����� ����
        public const int LCU_SVC_GAS_CON = 146; //���Ұ��� Ŀ���� ����

        public const int LCU_SVC_MOBILE_COMMAND = 201; //Painting robot_Mobile Command ����� LCU ���� ����
        public const int LCU_SVC_MOBILE_MONITORING_DATA = 202; //Painting robot_Mobile Monitoring ����� LCU ����͸� ������

        /////////////////////////////////////////////////////////////////////////////
        //
        //LCU VALUE
        public const int LCU_VAL_AO_PORT1 = 0;
        public const int LCU_VAL_AO_PORT2 = 1;
        public const int LCU_VAL_AO_PORT3 = 2;
        public const int LCU_VAL_AO_PORT4 = 3;
        public const int LCU_VAL_AO_PORT5 = 4;
        public const int LCU_VAL_AO_PORT6 = 5;
        public const int LCU_VAL_AO_PORT7 = 6;
        public const int LCU_VAL_AO_PORT8 = 7;

        public const int LCU_VAL_DIO_DIR_IN = 0;
        public const int LCU_VAL_DIO_DIR_OUT = 1;

        public const int LCU_VAL_DIO_PORT1 = 0;
        public const int LCU_VAL_DIO_PORT2 = 1;
        public const int LCU_VAL_DIO_PORT3 = 2;
        public const int LCU_VAL_DIO_PORT4 = 3;
        public const int LCU_VAL_DIO_PORT5 = 4;
        public const int LCU_VAL_DIO_PORT6 = 5;

        public const int LCU_VAL_CAN_DISCONNECT = 0;
        public const int LCU_VAL_CAN_CONNECT = 1;

        public const int LCU_VAL_MODE_SV_CMD = 1; //�̻��
        public const int LCU_VAL_MODE_JOINT_VEL = 2;
        public const int LCU_VAL_MODE_WORLD_VEL = 3;
        public const int LCU_VAL_MODE_TOOL_VEL = 4;
        public const int LCU_VAL_MODE_JOINT_POS = 5; //LCU���� ���� ��ġ�� �� �ٷ� �̵�(����).
        public const int LCU_VAL_MODE_JOINT_POS_PROF = 6; //LCU���� ���� Joint ��ġ�� ���� �ð� ���� �̵�(�ϼ�).
        public const int LCU_VAL_MODE_WORLD_POS_PROF = 7; //LCU���� ���� World ��ġ�� ���� �ð� ���� �̵�(���ߴܰ�).
        public const int LCU_VAL_MODE_WORLD_POS_M = 8; //Move from Measured to Target position _hmshin_20230127
        public const int LCU_VAL_MODE_WORLD_POS_D = 9; //Move from Previous Desired to Target position _hmshin_20230206
        public const int LCU_VAL_MODE_FORCE = 10; //Force control _hmshin_20230208

        public const int LCU_VAL_GRIP_OFF = 0; //Gripper Close - POSCO ICT
        public const int LCU_VAL_GRIP_ON_CLOSE = 1; //������� ���� ����
        public const int LCU_VAL_GRIP_ON_OPEN = 2; //Gripper Open - POSCO ICT

        public const int LCU_VAL_GAS_CON_OFF = 0; //Gas Connector Close - POSCO ICT
        public const int LCU_VAL_GAS_CON_ON = 1; //Gas Connector Open - POSCO ICT

        public const int LCU_VAL_TCP_STATE = 0; //��û
        public const int LCU_VAL_TCP_SET = 1; //����

        public const int LCU_VAL_COMPLIANCE_OFF = 0;
        public const int LCU_VAL_COMPLIANCE_ON = 1;

        public const int LCU_VAL_LIMIT_ERROR_DISABLE = 0; //����Ʈ ����(���� ������ ��������)�� ������� �ʴ� ���.
        public const int LCU_VAL_LIMIT_ERROR_ENABLE = 1;

        public const int LCU_VAL_TEMPERATURE_ERROR_DISABLE = 0;
        public const int LCU_VAL_TEMPERATURE_ERROR_ENABLE = 1;

        public const int LCU_VAL_POSDIFF_MONITOR_DISABLE = 0;
        public const int LCU_VAL_POSDIFF_MONITOR_ENABLE = 1;

        //ver.2.3.0
        public const int LCU_VAL_IP_CHANGE_LCU = 1;
        public const int LCU_VAL_IP_CHANGE_RM = 2;
        public const int LCU_VAL_IP_CHANGE_PC1 = 3;
        public const int LCU_VAL_IP_CHANGE_PC2 = 4;
        public const int LCU_VAL_IP_CHANGE_MULTI = 5;

        //ver.2.3.1
        public const int LCU_VAL_LIMIT_SW_STATE = 0;//������ SW����Ʈ �� ��û
        public const int LCU_VAL_LIMIT_SW_SET = 1;//SW����Ʈ �� ����
        public const int LCU_VAL_LIMIT_HW_STATE = 2;//������ HW����Ʈ �� ��û(���߿����� ���� ����)
        public const int LCU_VAL_LIMIT_HW_SET = 3;//HW����Ʈ �� ����(���߿����� ���� ����)

        //ver.2.3.1
        public const int LCU_VAL_SENDER_STATE = 0;
        public const int LCU_VAL_SENDER_SET = 1;

        /////////////////////////////////////////////////////////////////////////////
        //
        // Service Name: LCU_SVC_DATA

        //����: �ʱ�ȭ ��Ŷ. 
        //SVC: RM_SVC_SYS_INI_START
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_sys_init
        {
            public uint16_t nVersion; //���� ����
        }

        //SVC: LCU_SVC_KEEP_TIME 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_keep_time
        {
            public uint16_t nKeepTime; //�ð� msec
        }

        //SVC: LCU_SVC_GRIP  
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_grip_vel
        {
            public uint16_t nVelLevel; //�׸��� �ӵ�(0~10)
        }

        //SVC; LCU_SVC_ERROR 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_error
        {
            public uint16_t nError;
        }


        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_ai_state
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.LCU_ANALOG_PORT_CNT_MAX)]
            public float[] fAiState;
        }

        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_ao_state
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.LCU_ANALOG_PORT_CNT_MAX)]
            public float[] fAoState;
        }

        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_ao_cmd
        {
            float fAoCmd;//[LCU_ANALOG_PORT_CNT_MAX];
        }

        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_dio_dir
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.LCU_DIGITAL_PORT_CNT_MAX)]
            public uint16_t[] nDioDir;
        }

        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_dio_state
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.LCU_DIGITAL_PORT_CNT_MAX)]
            public uint16_t[] nDioState;
        }

        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_dio_set
        {
            public uint16_t nDioSet;//[LCU_DIGITAL_PORT_CNT_MAX];
        }

        //SVC: LCU_SVC_TEMP 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_temp
        {
            public float ftempLcu;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.LCU_ANALOG_PORT_CNT_MAX)]
            public float[] ftempHac;
        }

        //SVC: LCU_SVC_IP_CHANGE 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_ip_packet
        {
            public uint16_t nPort;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public uint16_t[] nIp;
        }

        //SVC: LCU_SVC_TCP 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_tcp
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldPos;
        }

        //SVC: LCU_SVC_CAN_SEND, LCU_SVC_CAN_RECEIVE
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_can_packet
        {
            public uint16_t nId;
            public uint16_t nDlc;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public uint8_t[] uData;
        }

        //SVC: LCU_SVC_CAN_SEND, LCU_SVC_CAN_RECEIVE - LCU pro ���� _ Added by Hmshin _ 20221115
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_can_udp_packet
        {
            public uint16_t Id;
            public uint16_t subindex;
            public float Data;
        }

        //SVC: LCU_SVC_CAN_SEND, LCU_SVC_CAN_RECEIVE - LCU pro ���� _ Added by Hmshin _ 20221115
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_can_udp_rx_packet
        {
            public float rxData;
        }

        //NOT USED, VC: LCU_SVC_SV_STATE 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_SV_state
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.LCU_ANALOG_PORT_CNT_MAX)]
            public uint16_t[] nSVOn;
        }

        //SVC: LCU_SVC_VEL_COMMAND 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_joint_cmd
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJointCmd;
            public uint16_t nEmergency;
        }

        //SVC: LCU_SVC_VEL_COMMAND 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_world_cmd
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldCmd;
            //public float th_7;
            public uint16_t nEmergency;
        }

        //SVC: LCU_SVC_POS_GAIN 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_pos_gain
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fVelLimit;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fAccelLimit;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJerkLimit;
            public float fConveGain;
            public float fToolVel;
        }

        //SVC: LCU_SVC_MONITORING_DATA  
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_monitor
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJointPos;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldPos;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJointPosCur;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldPosCur;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fDiffPressure;
            public float fPosError;
            public float fForceZ;
            public uint8_t fRmsCheck;
            public uint8_t fGripperState;
            public uint8_t fGasconState;
            public uint8_t nAutoControl;
            public uint8_t nEmergency;
        }
        public struct t_lcu_data_monitor2
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJointPos;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldPos;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJointPosCur;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldPosCur;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fDiffPressure;
            public float fPosError;
            public float fForceZ;
            public uint8_t fRmsCheck;
            public uint8_t fGripperState;
            public uint8_t fGasconState;
            public uint8_t nAutoControl;
            public uint8_t nEmergency;
        }
        public struct t_lcu_data_monitor3
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJointPos;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldPos;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJointPosCur;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldPosCur;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fDiffPressure;
            public float fPosError;
            public float fForceZ;
            public uint8_t fRmsCheck;
            public uint8_t fGripperState;
            public uint8_t fGasconState;
            public uint8_t nAutoControl;
            public uint8_t nEmergency;
        }
        public struct t_lcu_data_monitor4
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJointPos;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldPos;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJointPosCur;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldPosCur;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fDiffPressure;
            public float fPosError;
            public float fForceZ;
            public uint8_t fRmsCheck;
            public uint8_t fGripperState;
            public uint8_t fGasconState;
            public uint8_t nAutoControl;
            public uint8_t nEmergency;
        }
        public struct t_lcu_data_monitor5
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJointPos;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldPos;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJointPosCur;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldPosCur;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fDiffPressure;
            public float fPosError;
            public float fForceZ;
            public uint8_t fRmsCheck;
            public uint8_t fGripperState;
            public uint8_t fGasconState;
            public uint8_t nAutoControl;
            public uint8_t nEmergency;
        }

        //SVC : LCU_SVC_POS_COMMAND - Joint
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_joint_cmd_pos
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJointCmd;
            public float toolCmd;
            public float RMS;
        }

        //SVC : LCU_SVC_POS_COMMAND - World - Move from Measured to Target position 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_world_cmd_pos_M
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldCmd;
            public float toolCmd;
            public float RMS;
        }
        //SVC : LCU_SVC_POS_COMMAND - World - Move from Previous Desired to Target position 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_world_cmd_pos_D
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldCmd;
            public float toolCmd;
            public float RMS;
        }

        //SVC : LCU_SVC_POS_COMMAND - Force 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_force
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fForceCmd;
        }

        //SVC : LCU_SVC_POS_COMMAND 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_joint_cmd_pos_prof
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fJointCmd;
            public uint16_t nMoveTimeMs;
        }

        //SVC : LCU_SVC_POS_COMMAND 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_world_cmd_pos_prof
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fWorldCmd;
            public uint16_t nMoveTimeMs;
        }

        //SVC : LCU_SVC_LIMIT_ERROR  
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_limit
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fLimitMin;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            public float[] fLimitMax;
        }

        //SVC: LCU_SVC_SENDER_CHANGE 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_sender
        {
            public uint8_t nSender;
        }

        //MOBILE
        //SVC: LCU_SVC_MOBILE_COMMAND 
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_mobile_cmd
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.MOBILE_ENCODER_CNT_MAX)]
            public float[] fEncoderCmd;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.MOBILE_SERVO_CNT_MAX)]
            public float[] fServoCmd;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.MOBILE_DO_CNT_MAX)]
            public uint16_t[] nDigitalCmd;
            public uint16_t nEmergency;
        }

        //MOBILE
        //SVC: LCU_SVC_MOBILE_MONITORING_DATA  
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_data_mobile_monitor
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.MOBILE_ENCODER_CNT_MAX)]
            public float[] fEncoderCnt;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.MOBILE_SERVO_CNT_MAX)]
            public float[] fServoState;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.MOBILE_PRES_CNT_MAX)]
            public float[] fPresState;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.MOBILE_DO_CNT_MAX)]
            public uint16_t[] nDigitalState;
            public uint16_t nEmergency;
        }

        //PACKET
        [Serializable]
        [StructLayout(LayoutKind.Explicit, Pack = 1)]
        public struct t_lcu_data
        {
            /*[FieldOffset(0)]
            [MarshalAs(UnmanagedType.ByValArray)] //, SizeConst = LcuCommon.PACKET_DATA_LEN, ArraySubType = UnmanagedType.I1
            public uint8_t[] uData;*/
            [FieldOffset(0)]
            public t_lcu_data_sys_init sysInit;
            [FieldOffset(0)]
            public t_lcu_data_keep_time keepTime;
            [FieldOffset(0)]
            public t_lcu_data_grip_vel gripVel;
            [FieldOffset(0)]
            public t_lcu_data_error errorData;
            [FieldOffset(0)]
            public t_lcu_data_ai_state aiState;
            [FieldOffset(0)]
            public t_lcu_data_ao_state aoState;
            [FieldOffset(0)]
            public t_lcu_data_ao_cmd aoCmd;
            [FieldOffset(0)]
            public t_lcu_data_dio_dir dioDir;
            [FieldOffset(0)]
            public t_lcu_data_dio_state dioState;
            [FieldOffset(0)]
            public t_lcu_data_dio_set dioSet;
            [FieldOffset(0)]
            public t_lcu_data_temp temperature;
            [FieldOffset(0)]
            public t_lcu_data_can_packet canPacket;
            [FieldOffset(0)]
            public t_lcu_data_can_udp_packet canUdpPacket;  //Added by Hmshin _ 20221115
            [FieldOffset(0)]
            public t_lcu_data_can_udp_rx_packet canUdpRxPacket;	//Added by Hmshin _ 20221115
            [FieldOffset(0)]
            public t_lcu_data_joint_cmd jointCmd;
            [FieldOffset(0)]
            public t_lcu_data_world_cmd worldCmd;
            [FieldOffset(0)]
            public t_lcu_data_monitor monitoringData;
            [FieldOffset(0)]
            public t_lcu_data_monitor2 monitoringData2;
            [FieldOffset(0)]
            public t_lcu_data_monitor3 monitoringData3;
            [FieldOffset(0)]
            public t_lcu_data_monitor4 monitoringData4;
            [FieldOffset(0)]
            public t_lcu_data_monitor5 monitoringData5;
            [FieldOffset(0)]
            public t_lcu_data_tcp tcpData;
            [FieldOffset(0)]
            public t_lcu_data_joint_cmd_pos jointTarget;
            [FieldOffset(0)]
            public t_lcu_data_world_cmd_pos_M worldTarget_M;
            [FieldOffset(0)]
            public t_lcu_data_world_cmd_pos_D worldTarget_D;
            [FieldOffset(0)]
            public t_lcu_data_joint_cmd_pos_prof jointTargetProf;
            [FieldOffset(0)]
            public t_lcu_data_world_cmd_pos_prof worldTargetProf;
            [FieldOffset(0)]
            public t_lcu_data_pos_gain posGain;
            [FieldOffset(0)]
            public t_lcu_data_ip_packet ipData;
            [FieldOffset(0)]
            public t_lcu_data_SV_state svState;
            [FieldOffset(0)]
            public t_lcu_data_mobile_cmd mobileCmd;
            [FieldOffset(0)]
            public t_lcu_data_mobile_monitor mobileMonitoringData;
        }


        /*[Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_packet_parse
        {
            public uint8_t uChecksum;
            public uint8_t uSender;
            public uint8_t uReceiver;
            public uint8_t uAcknowledge;
            public uint8_t uCode;
            public uint8_t uValue;
            public uint16_t nDataSize;
            public t_lcu_data uData;
        }*/

        //*Use Const size packet for Pointer Error
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_lcu_packet_const_size
        {
            public uint8_t uChecksum;
            public uint8_t uSender;
            public uint8_t uReceiver;
            public uint8_t uAcknowledge;
            public uint8_t uCode;
            public uint8_t uValue;
            public uint16_t nDataSize;
            [MarshalAs(UnmanagedType.ByValArray)]
            public uint8_t[] uData;
        }
    }
#endif

#if KNR_RM_COMMUNICATION_H
    public class RmCommunication
    {
        public const int nRM_VERSION = 210; //major(LCU ver) + minor(Header ver) + patch
        public const string RM_VERSION = "2.1.0"; //major(LCU ver).minor(Header ver).patch
        public const string RM_BUILD = "2017.07.05";

        /////////////////////////////////////////////////////////////////////////////
        //
        // RM&TE State
        // system mode(������)
        public const int MODE_STAT_INIT = 0;
        public const int MODE_STAT_MANUAL = 1;
        public const int MODE_STAT_AUTORUN = 2;
        public const int MODE_STAT_STEP = 3;
        public const int MODE_STAT_ERROR = 4;
        public const int MODE_STAT_ESTOP = 5;
        public const int MODE_STAT_TERMINATE = 6;

        // job execution state(������)
        public const int EXEC_STAT_IDLE = 0; //WORK_TYPE_NONE
        public const int EXEC_STAT_EXECUTING = 1; //EXEC_TYPE_JOB
        public const int EXEC_STAT_EXCEPT = 2; //WORK_TYPE_HOME

        // service exceptional working motion(������)
        public const int WORK_TYPE_NONE = 0;
        public const int WORK_TYPE_JOB = 1;
        public const int WORK_TYPE_HOME = 2;
        public const int WORK_TYPE_JOG = 3;

        /////////////////////////////////////////////////////////////////////////////
        //
        //RM SERVICE LIST (1 ~ 50)
        //System ����
        public const int RM_SVC_SYS_INI_START = 1; //�ý��� �ʱ�ȭ(system initialize)
        public const int RM_SVC_SYSTEM_STOP = 2; //�ý��� �������(��-��� ����)
        public const int RM_VAL_SYSTEM_STOP_NORMAL = 0;
        public const int RM_VAL_SYSTEM_STOP_EMERGENCY = 1;
        public const int RM_SVC_STATE_TO_TE = 3; //������
        public const int RM_SVC_ERROR_RESET = 4; //������

        public const int RM_SVC_JOB_START = 11; //������
        public const int RM_VAL_JOB_START_STEP = 0; //������
        public const int RM_VAL_JOB_START_THRU = 1; //������
        public const int RM_SVC_JOB_STOP = 12; //������
        public const int RM_SVC_JOB_COMPILE = 13; //������

        public const int RM_SVC_HOME = 21; //������
        public const int RM_VAL_HOME_MOVE = 0; //������
        public const int RM_VAL_HOME_EDIT = 1; //������


        /////////////////////////////////////////////////////////////////////////////
        //
        // Service Name: RM_SVC_SYS_INI_START
        //
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_rm_data_sys_init
        {
            uint16_t nVersion;
        }

        /////////////////////////////////////////////////////////////////////////////
        //
        // Service Name: RM_SVC_JOB_START
        //
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_rm_job_start_data
        {
            uint16_t nReqLineIndex;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.JOB_NAME_SIZE)]
            uint8_t[] szJobName;
        }

        /////////////////////////////////////////////////////////////////////////////
        //
        // Service Name: RM_SVC_JOB_COMPILE
        //
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_rm_job_compile_data
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.JOB_NAME_SIZE)]
            uint8_t[] szJobName;
        }

        /////////////////////////////////////////////////////////////////////////////
        //
        // Service Name: RM_SVC_HOME
        //
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_rm_home_move_data
        {
            uint16_t nHomeIdx;
            double dbHomeSpd;
        }

        /////////////////////////////////////////////////////////////////////////////
        //
        // Service Name: RM_SVC_STATE_TO_TE
        //
        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_rm_state_to_te_data
        {
            uint16_t nErrorCode;
            uint16_t nSysMode;
            uint16_t nWorkType;
            uint16_t nExecStat;
            uint16_t nEstopState;
        }

        [Serializable]
        [StructLayout(LayoutKind.Explicit, Pack = 1)]
        public struct rm_msg_data
        {
            /*[FieldOffset(0)]
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.PACKET_DATA_LEN)]
            uint8_t[] uData;*/
            [FieldOffset(0)]
            t_rm_data_sys_init sysInit;
            [FieldOffset(0)]
            t_rm_job_start_data jobStart;
            [FieldOffset(0)]
            t_rm_job_compile_data jobCompile;
            [FieldOffset(0)]
            t_rm_home_move_data homeCmd;
            [FieldOffset(0)]
            t_rm_state_to_te_data stateToTE;
        }

        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_rm_packet
        {
            uint8_t uChecksum;
            uint8_t uSender;
            uint8_t uReceiver;
            uint8_t uAcknowledge;
            uint8_t uCode;
            uint8_t uValue;
            uint16_t nDataSize;

            rm_msg_data uData;
        }
    }
#endif

#if KNR_TE_COMMUNICATION_H

    public class TeCommunication
    {
        public const int nTE_VERSION = 210; //major(LCU ver) + minor(Header ver) + patch
        public const string TE_VERSION = "2.1.0"; //major(LCU ver).minor(Header ver).patch
        public const string TE_BUILD = "2017.07.05";

        /////////////////////////////////////////////////////////////////////////////
        //
        //TE SERVICE LIST (50 ~ 100)
        //

        public const int TE_SVC_STATE_TO_RM = 51; // Send TE state to RM	//cycle: 100ms

        /////////////////////////////////////////////////////////////////////////////
        //
        // Service Name: RM_SVC_SYS_INI_START
        public struct t_te_data_sys_init
        {
            uint16_t nVersion;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.LCU_DIGITAL_PORT_CNT_MAX)]
            uint8_t[] uDioDir;  //0: Input, 1: Output
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            float[] fMaxAngle;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.ROBOT_AXIS_CNT_MAX)]
            float[] fMinAngle;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.LCU_ANALOG_PORT_CNT_MAX)]
            uint8_t uAoConfig; // 0: Access Prohibited, 1: User
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = LcuCommon.LCU_ANALOG_PORT_CNT_MAX)]
            uint8_t uAiConfig; // 0: Access Prohibited, 1: User
        }

        /////////////////////////////////////////////////////////////////////////////
        //
        // Service Name: TE_SVC_STATE_TO_RM

        public struct t_te_state_to_rm_data
        {
            uint16_t nErrorCode;
            uint16_t nRunLineIndex;
            uint16_t nSysMode;
            uint16_t nWorkType;
            uint16_t nExeStat;
        }

        [Serializable]
        [StructLayout(LayoutKind.Explicit, Pack = 1)]
        public struct te_msg_data
        {
            [FieldOffset(0)]
            t_te_data_sys_init sysInit;
            [FieldOffset(0)]
            t_te_state_to_rm_data stateToRM;
        }

        [Serializable]
        [StructLayout(LayoutKind.Sequential, Pack = 1)]
        public struct t_te_packet
        {
            uint8_t uChecksum;
            uint8_t uSender;
            uint8_t uReceiver;
            uint8_t uAcknowledge;
            uint8_t uCode;
            uint8_t uValue;
            uint16_t nDataSize;

            te_msg_data uData;
        }
    }
#endif

#if MANIPULATORERROR_H
    public class LcuManipulatorError
    {
        public const string MANIPULATORERROR_VERSION = "2.0.0";
        public const string MANIPULATORERROR_BUILD = "2017.12.28";

        public const UInt16 ERR_FROM_LCU = 0x2000;

        public const UInt16 ERR_CRITICAL = 0x0000;
        public const UInt16 ERR_WARNING = 0x0800;

        //////////////////////////////////////////////////////////
        // LCU  | WARN | LCU
        //0x2000|0x0800|0x0000
        public const UInt16 ERR_LCU_COMM_VER_DIFFERENCE = 0x2800;
        public const UInt16 ERR_LCU_NO_START = 0x2801;
        public const UInt16 ERR_LCU_OVER_KEEPTIME = 0x2802;
        public const UInt16 ERR_LCU_OVER_TEMP = 0x2803;
        public const UInt16 ERR_LCU_ANALOG_PORT_UNDEFINED = 0x2804;
        public const UInt16 ERR_LCU_ANALOG_PORT_DENIED = 0x2805;
        public const UInt16 ERR_LCU_ANALOG_CMD_OVERRAGE = 0x2806;
        public const UInt16 ERR_LCU_DIO_PORT_UNDEFINED = 0x2807;
        public const UInt16 ERR_LCU_DIO_PORT_WRITE_DENIED = 0x2808;
        public const UInt16 ERR_LCU_DIO_WRONG_VALUE = 0x2809;

        public const UInt16 ERR_LCU_CAN_ID_EXCEEDED = 0x280A;
        public const UInt16 ERR_LCU_CAN_DLC_EXCEEDED = 0x280B;

        public const UInt16 ERR_LCU_PACKET_SIZE = 0x2810;
        public const UInt16 ERR_LCU_CHECKSUM = 0x2811;
        public const UInt16 ERR_LCU_SENDER = 0x2812;
        public const UInt16 ERR_LCU_RECEIVER = 0x2813;
        public const UInt16 ERR_LCU_UNKOWN_CODE = 0x2814;
        public const UInt16 ERR_LCU_UNDEFINED_VALUE = 0x2815; //2016.12.12.
        public const UInt16 ERR_LCU_DATA_SIZE = 0x2816;

        public const UInt16 ERR_LCU_MOTION_NOT_STARTED = 0x2821;
        public const UInt16 ERR_LCU_MODE_UNDEFINED = 0x2822;
        public const UInt16 ERR_LCU_MODE_UNMATCHED = 0x2823;

        //////////////////////////////////////////////////////////
        // LCU  | WARN | HAC
        //0x2000|0x0800|0x0400
        public const UInt16 ERR_LCU_QLIM_LOW_AXIS = 0x2C10;
        public const UInt16 ERR_LCU_QLIM_LOW_AXIS1 = 0x2C11;
        public const UInt16 ERR_LCU_QLIM_LOW_AXIS2 = 0x2C12;
        public const UInt16 ERR_LCU_QLIM_LOW_AXIS3 = 0x2C13;
        public const UInt16 ERR_LCU_QLIM_LOW_AXIS4 = 0x2C14;
        public const UInt16 ERR_LCU_QLIM_LOW_AXIS5 = 0x2C15;
        public const UInt16 ERR_LCU_QLIM_LOW_AXIS6 = 0x2C16;
        public const UInt16 ERR_LCU_QLIM_HIGH_AXIS = 0x2C20;
        public const UInt16 ERR_LCU_QLIM_HIGH_AXIS1 = 0x2C21;
        public const UInt16 ERR_LCU_QLIM_HIGH_AXIS2 = 0x2C22;
        public const UInt16 ERR_LCU_QLIM_HIGH_AXIS3 = 0x2C23;
        public const UInt16 ERR_LCU_QLIM_HIGH_AXIS4 = 0x2C24;
        public const UInt16 ERR_LCU_QLIM_HIGH_AXIS5 = 0x2C25;
        public const UInt16 ERR_LCU_QLIM_HIGH_AXIS6 = 0x2C26;
        public const UInt16 ERR_LCU_POS_DIFF_AXIS = 0x2C30;
        public const UInt16 ERR_LCU_POS_DIFF_AXIS1 = 0x2C31;
        public const UInt16 ERR_LCU_POS_DIFF_AXIS2 = 0x2C32;
        public const UInt16 ERR_LCU_POS_DIFF_AXIS3 = 0x2C33;
        public const UInt16 ERR_LCU_POS_DIFF_AXIS4 = 0x2C34;
        public const UInt16 ERR_LCU_POS_DIFF_AXIS5 = 0x2C35;
        public const UInt16 ERR_LCU_POS_DIFF_AXIS6 = 0x2C36;
        public const UInt16 ERR_LCU_OVER_TEMP_AXIS = 0x2C40;
        public const UInt16 ERR_LCU_OVER_TEMP_AXIS1 = 0x2C41;
        public const UInt16 ERR_LCU_OVER_TEMP_AXIS2 = 0x2C42;
        public const UInt16 ERR_LCU_OVER_TEMP_AXIS3 = 0x2C43;
        public const UInt16 ERR_LCU_OVER_TEMP_AXIS4 = 0x2C44;
        public const UInt16 ERR_LCU_OVER_TEMP_AXIS5 = 0x2C45;
        public const UInt16 ERR_LCU_OVER_TEMP_AXIS6 = 0x2C46;
        public const UInt16 ERR_LCU_QLIM_SW_LOW_AXIS = 0x2C50;
        public const UInt16 ERR_LCU_QLIM_SW_LOW_AXIS1 = 0x2C51;
        public const UInt16 ERR_LCU_QLIM_SW_LOW_AXIS2 = 0x2C52;
        public const UInt16 ERR_LCU_QLIM_SW_LOW_AXIS3 = 0x2C53;
        public const UInt16 ERR_LCU_QLIM_SW_LOW_AXIS4 = 0x2C54;
        public const UInt16 ERR_LCU_QLIM_SW_LOW_AXIS5 = 0x2C55;
        public const UInt16 ERR_LCU_QLIM_SW_LOW_AXIS6 = 0x2C56;
        public const UInt16 ERR_LCU_QLIM_SW_HIGH_AXIS = 0x2C60;
        public const UInt16 ERR_LCU_QLIM_SW_HIGH_AXIS1 = 0x2C61;
        public const UInt16 ERR_LCU_QLIM_SW_HIGH_AXIS2 = 0x2C62;
        public const UInt16 ERR_LCU_QLIM_SW_HIGH_AXIS3 = 0x2C63;
        public const UInt16 ERR_LCU_QLIM_SW_HIGH_AXIS4 = 0x2C64;
        public const UInt16 ERR_LCU_QLIM_SW_HIGH_AXIS5 = 0x2C65;
        public const UInt16 ERR_LCU_QLIM_SW_HIGH_AXIS6 = 0x2C66;
    }
#endif

#if KNR_LCU_CAN_H
    public class LcuCan
    {
        public const UInt16 LCU_CAN_ID_RECV = 0x0280;
        public const UInt16 LCU_CAN_ID_SEND = 0x0300;
        public const UInt16 LCU_CAN_DLC = 0x0008;

        public const byte LCU_CAN_SET_UINT = 0x00;
        public const byte LCU_CAN_GET_UINT = 0x01;
        public const byte LCU_CAN_SET_FLOAT = 0x02;
        public const byte LCU_CAN_GET_FLOAT = 0x03;

        public const UInt16 LCU_CAN_INDEX_SERIAL_CMD = 0x0100;
        public const UInt16 LCU_CAN_INDEX_ENCODER_RAW_ANGLE = 0x0101;
        public const UInt16 LCU_CAN_INDEX_WRITE_GAIN = 0x1010;

        public const UInt16 LCU_CAN_INDEX_OUTPUT_SIGN = 0x3001;
        public const UInt16 LCU_CAN_INDEX_OUTPUT_SV = 0x3002;
        public const UInt16 LCU_CAN_INDEX_START_MODE = 0x3003;
        public const UInt16 LCU_CAN_INDEX_POSITION_GAIN = 0x3010;
        public const UInt16 LCU_CAN_INDEX_TORQUE_GAIN = 0x3011;
        public const UInt16 LCU_CAN_INDEX_COMPLIANCE_GAIN = 0x3012;
        public const UInt16 LCU_CAN_INDEX_VELOCITY_GAIN = 0x3013;
        public const UInt16 LCU_CAN_INDEX_ESUM_CLEAR = 0x3014;
        public const UInt16 LCU_CAN_INDEX_SET_MIN_MAX_POS = 0x3015;
        public const UInt16 LCU_CAN_INDEX_BOARD_TEMPERATURE = 0x3020;
        public const UInt16 LCU_CAN_INDEX_THERMOCOUPLER_TEMPERATURE = 0x3021;
        public const UInt16 LCU_CAN_INDEX_ANALOG_READ = 0x3030;
        public const UInt16 LCU_CAN_INDEX_STOP = 0x6040;
        public const UInt16 LCU_CAN_INDEX_CONTROL_MODE = 0x6060;
        public const UInt16 LCU_CAN_INDEX_TARGET_POSITION = 0x6062;
        public const UInt16 LCU_CAN_INDEX_ACTUAL_POSITION = 0x6063;
        public const UInt16 LCU_CAN_INDEX_TARGET_VELOCITY = 0x606B;
        public const UInt16 LCU_CAN_INDEX_ACTUAL_VELOCITY = 0x606C;
        public const UInt16 LCU_CAN_INDEX_TARGET_TORQUE = 0x6071;
        public const UInt16 LCU_CAN_INDEX_ACTUAL_TORQUE = 0x6077;

        public const byte HAC_CONTROL_MODE_OPEN_CAN = 0x00;
        public const byte HAC_CONTROL_MODE_OPEN_ANALOG = 0x0F;
        public const byte HAC_CONTROL_MODE_POSITIONCONTROL_CAN = 0x01;
        public const byte HAC_CONTROL_MODE_POSITIONCONTROL_ANALOG = 0x02;
        public const byte HAC_CONTROL_MODE_FORCECONTROL_CAN = 0x03;
        public const byte HAC_CONTROL_MODE_FORCECONTROL_ANALOG = 0x04;
        public const byte HAC_CONTROL_MODE_COMPLIANCECONTROL_CAN = 0x05;
        public const byte HAC_CONTROL_MODE_COMPLIANCECONTROL_ANALOG = 0x06;

        public const byte HAC_CONTROL_MODE_ENCODER_ERROR_STOP = 0x10;
        public const byte HAC_CONTROL_MODE_AUTO_SERVO_VIBR = 0x11;
        public const byte HAC_CONTROL_MODE_AUTO_CALIBRATION = 0x12;
        public const byte HAC_CONTROL_MODE_AUTO_LOOP_A = 0x1A;
        public const byte HAC_CONTROL_MODE_AUTO_LOOP_B = 0x1B;
        public const byte HAC_CONTROL_MODE_AUTO_LOOP_C = 0x1C;
    }
#endif
}