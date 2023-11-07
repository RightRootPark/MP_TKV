using KnrHeader;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

public class LCU_Manager
{
    #region public
    public LCU_Communication arm { get; private set; }
    public LCU_Communication arm2 { get; private set; }
    public LCU_Communication arm3 { get; private set; }

    public LCU_Communication arm4 { get; private set; }

    public LCU_Communication arm5 { get; private set; }


    public LCU_Communication lcupro { get; private set; }
    public enum Mode { Joint, World, Position_Joint, Position_World_M, Position_World_D, Force }
    #endregion

    Thread dpThread;
    private bool stopThread;

    public LCU_Manager()
    {
        //SelectDeviceLCU(0);
    }

    ////Data monitoring thread
    //public void start()
    //{
    //    if (arm == null) return;

    //    dpThread = new Thread(threadProc);
    //    dpThread.Start();
    //}
    //public void stop()
    //{
    //    stopThread = true;
    //    if (null != dpThread) dpThread.Join();
    //    dpThread = null;
    //    stopThread = false;
    //}

    //void threadProc(object obj)
    //{
    //    //long currentTime = 0;
    //    //long nextTime = 0;

    //    //// Random walk variables
    //    //Random rand = new Random(9);
    //    float[] degree_axis = new float[6];

    //    // Variables to keep track of the timing
    //    Stopwatch timer = new Stopwatch();
    //    timer.Start();

    //    while (!stopThread)
    //    {
    //        // Compute the next data value
    //        //currentTime = timer.Elapsed.Ticks / 10000;

    //        for (int i = 0; i < 6; i++) degree_axis[i] = arm.monitoringData.fJointPos[i];

    //        Thread.Sleep(10);
    //    }
    //}

    ////설명: targetDevice가 UW3 인지 Mobile 인지 선택
    ////기능: Change Event
    //public void SelectDeviceLCU(int deviceIndex)
    //{
    //    switch (deviceIndex)
    //    {
    //        case 0:
    //            targetDevice = arm;
    //            Debug.WriteLine("target Device = Arm");
    //            break;
    //        case 1:
    //            targetDevice = mobile;
    //            Debug.WriteLine("target Device = Mobile");
    //            break;
    //    }
    //}

    //설명: 소캣을 받아와서 LCU 클래스 객체 생성. Manipulator로 사용
    public void SocketUDPCreateArm(UdpClient client, IPEndPoint ipEndPoint)
    {
        if (arm != null) return;
        arm = new LCU_Communication(client, ipEndPoint, 1);
    }
    public void SocketUDPCreateArm2(UdpClient client, IPEndPoint ipEndPoint)
    {
        if (arm2 != null) return;
        arm2 = new LCU_Communication(client, ipEndPoint, 2);
    }
    public void SocketUDPCreateArm3(UdpClient client, IPEndPoint ipEndPoint)
    {
        if (arm3 != null) return;
        arm3 = new LCU_Communication(client, ipEndPoint, 3);
    }
    public void SocketUDPCreateArm4(UdpClient client, IPEndPoint ipEndPoint)
    {
        if (arm4 != null) return;
        arm4 = new LCU_Communication(client, ipEndPoint, 4);
    }
    public void SocketUDPCreateArm5(UdpClient client, IPEndPoint ipEndPoint)
    {
        if (arm5 != null) return;
        arm5 = new LCU_Communication(client, ipEndPoint, 5);
    }

    //설명: 최초 혹은 초기화할 때 아래의 옵션들을 LCU로 보냄.
    private void SendSetInitAll()
    {
        arm.SendErrorSumClear();
        //arm.SendSetReplyTime(10);
        //arm.SendSetMode(LcuCommunication.LCU_VAL_MODE_JOINT_VEL);
        //arm.SendSetKeepTime(200);
        //arm.SendSetLimitStop(10);
    }

    public void ClickControlStart()
    {
        //targetDevice.SendInitStart();
        arm.SendInitStart();
        arm2.SendInitStart();
        arm3.SendInitStart();
        arm4.SendInitStart();
        arm5.SendInitStart();
        arm.SendErrorSumClear();
        arm2.SendErrorSumClear();
        arm3.SendErrorSumClear();
        arm4.SendErrorSumClear();
        arm5.SendErrorSumClear();
        //arm.SendSetReplyTime(10);
        //arm.SendSetMode(LcuCommunication.LCU_VAL_MODE_JOINT_VEL);
        //arm.SendSetKeepTime(200);
        //arm.SendSetLimitStop(10);

        //SendSetInitAll();
        Debug.WriteLine("click control start");
        //targetDevice.SendInitStart();
        //arm.SendInitStart();
    }

    //설명: LCU(UW3/mobile)를 긴급정지시킨다. 솔레노이드 밸브를 닫는다.
    //기능: Click Event
    public void ClickControlStop()
    {
        //targetDevice.SendSetEmergency();
        arm.SendSetEmergency();
        arm2.SendSetEmergency();
        arm3.SendSetEmergency();
        arm4.SendSetEmergency();
    }

    //설명: LCU의 에러를 리셋.
    //기능: Click Event
    public void ClickControlReset()
    {
        //targetDevice.SendErrorReset();
        arm.SendErrorReset();
        arm2.SendErrorReset();
        arm3.SendErrorReset();
        arm4.SendErrorReset();
    }

    //설명: SendSetMasterControl참고
    //기능: Click Event
    public void ClickControlSetMaster()
    {
        //targetDevice.SendSetMasterControl();
        arm.SendSetMasterControl();
        arm2.SendSetMasterControl();
        arm3.SendSetMasterControl();
        arm4.SendSetMasterControl();
    }

    //설명: SendSetMode 참고
    //기능: Change Event
    public void ChangeControlMode(int index)
    {
        switch (index)
        {
            case (int)Mode.Joint:
                //if (arm.mode != LcuCommunication.LCU_VAL_MODE_JOINT_VEL)
                arm.SendSetMode(LcuCommunication.LCU_VAL_MODE_JOINT_VEL);
                arm2.SendSetMode(LcuCommunication.LCU_VAL_MODE_JOINT_VEL);
                arm3.SendSetMode(LcuCommunication.LCU_VAL_MODE_JOINT_VEL);
                arm4.SendSetMode(LcuCommunication.LCU_VAL_MODE_JOINT_VEL);
                arm5.SendSetMode(LcuCommunication.LCU_VAL_MODE_JOINT_VEL);
                break;
            case (int)Mode.World:
                //if (arm.mode != LcuCommunication.LCU_VAL_MODE_WORLD_VEL)
                arm.SendSetMode(LcuCommunication.LCU_VAL_MODE_WORLD_VEL);
                break;
            case (int)Mode.Position_Joint:
                //if (arm.mode != LcuCommunication.LCU_VAL_MODE_JOINT_POS)
                arm.SendSetMode(LcuCommunication.LCU_VAL_MODE_JOINT_POS);
                arm2.SendSetMode(LcuCommunication.LCU_VAL_MODE_JOINT_POS);
                arm3.SendSetMode(LcuCommunication.LCU_VAL_MODE_JOINT_POS);
                arm4.SendSetMode(LcuCommunication.LCU_VAL_MODE_JOINT_POS);
                arm5.SendSetMode(LcuCommunication.LCU_VAL_MODE_JOINT_POS);
                break;
            case (int)Mode.Position_World_M:
                //if (arm.mode != LcuCommunication.LCU_VAL_MODE_JOINT_POS)
                arm.SendSetMode(LcuCommunication.LCU_VAL_MODE_WORLD_POS_M);
                break;
            case (int)Mode.Position_World_D:
                //if (arm.mode != LcuCommunication.LCU_VAL_MODE_JOINT_POS)
                arm.SendSetMode(LcuCommunication.LCU_VAL_MODE_WORLD_POS_D);
                break;
            case (int)Mode.Force:
                //if (arm.mode != LcuCommunication.LCU_VAL_MODE_JOINT_POS)
                arm.SendSetMode(LcuCommunication.LCU_VAL_MODE_FORCE);
                break;
        }

    }
}