using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

using KnrHeader;

public class Setup
{

    private UdpClient _udpClient;
    private IPEndPoint _ipEndPoint;

    private byte _sender;
    private byte _receiver;
        
    public Setup(UdpClient client, IPEndPoint ipEndPoint)
    {
        _udpClient = client;
        _ipEndPoint = ipEndPoint;

        _sender = LcuCommon.HEADER_PC1;
        _receiver = LcuCommon.HEADER_LCU1;
    }

    //설명: Conversion gain 설정 - Target 수렴 속도 조절
    public void SendSetPosGain(float _conveGain, float[] _velLimit)
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
        data.fConveGain = _conveGain;
        data.fVelLimit = _velLimit;

        packet.uData = TypeConvert.StructToBytes(data, packet.nDataSize);

        SendToLCU(packet);
    }


    //설명: UDP 통신을 활용하여 LCU로 데이터를 보냄.
    //*Use Const size packet for Covertring Union to Pointer Error
    public void SendToLCU(LcuCommunication.t_lcu_packet_const_size packet)
    {
        //구조체를 바이트 배열로 변환
        // first of All, packet should have all data before use ClacCheckSum()
        //*Use Const size packet for Covertring Union to Pointer Error
        byte[] bytes = TypeConvert.StructToBytes(packet, LcuCommon.PACKET_HEADER_LEN + LcuCommon.PACKET_NULL_LEN + packet.nDataSize);

        for (int i = 0; i < packet.nDataSize; i++)
        {
            bytes[LcuCommon.PACKET_HEADER_LEN + i] = packet.uData[i];
        }
        bytes[bytes.Length - 1] = 0; //마지막 바이트: NULL
        bytes[0] = LcuCommon.CalcChecksum(bytes); //checksum Index = 0

        ////ParseReceive(bytes); //test

        //Debug.WriteLine("SendToLCU : " + bytes.Length);

        ////응답 확인 여부 결정
        ////Check Wait Acknowledge
        //if (packet.uAcknowledge > 0) SetIsWaitAck(packet.uCode, true);
        //else SetIsWaitAck(packet.uCode, false);

        try
        {
            //LCU로 해당데이터를 보냄
            _udpClient.Send(bytes, bytes.Length, _ipEndPoint);
        }
        catch (Exception err)
        {
            Debug.WriteLine(err.ToString());
        }
    }

}
