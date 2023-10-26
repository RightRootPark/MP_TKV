using System;
using System.Drawing;
using System.Collections;
using System.ComponentModel;
using System.Windows.Forms;
using System.Data;
using NationalInstruments.DAQmx;
using System.Diagnostics;

public class NiDaqMax
{
    public bool[] _DI_array;
    public bool[] _DI_array_Pre;
    public bool[] _DO_array;

    private Task digitalInTask;
    private Task digitalOutTask;

    private DigitalSingleChannelReader reader;
    private System.ComponentModel.IContainer components;

    //DIO 의 배열 인덱스
    public enum DI
    {
        safetyInput,    //DI_1 _ Dev1 _ P0.0 _ input
        start,          //DI_20 _ Dev1 _ P0.1 _ input
        pause,          //DI_2 _ Dev1 _ P0.2 _ input
        stop,           //DI_21 _ Dev1 _ P0.3 _ input
        empty_04,       //비어있는 핀 _ P0.4
        empty_05,       //비어있는 핀 _ P0.5
        empty_06,       //비어있는 핀 _ P0.6
        empty_07,       //비어있는 핀 _ P0.7
    }

    public NiDaqMax()
    {
        _DI_array = new bool[8];
        _DI_array_Pre = new bool[8];
        _DO_array = new bool[8];
    }

    public bool _StartOn = false;
    public bool _StartOff = false;
    public bool _PauseOn = false;
    public bool _PauseOff = false;
    public bool _StopOn = false;
    public bool _StopOff = false;

    /// <summary>
    /// Clean up any resources being used.
    /// </summary>
    //protected override void Dispose(bool disposing)
    //{
    //    if (disposing)
    //    {
    //        if (components != null)
    //        {
    //            components.Dispose();
    //        }
    //    }
    //    base.Dispose(disposing);
    //}

    public void UpdateDIState()
    {
        try
        {
            {
                digitalInTask = new Task();

                digitalInTask.DIChannels.CreateChannel(
                    "Dev1/port0/line0:7",
                    "MyDIChannel",
                    ChannelLineGrouping.OneChannelForAllLines);

                reader = new DigitalSingleChannelReader(digitalInTask.Stream);

                _DI_array = reader.ReadSingleSampleMultiLine();

                //_DI_array.CopyTo(_DI_array_Pre, 0);
            }
        }
        catch (DaqException exception)
        {

        }
    }

    public void _DO()
    {
        try
        {
            using (digitalOutTask = new Task())
            {
                digitalOutTask.DOChannels.CreateChannel(
                    "Dev1/port2/line0:7",
                    "MyDOChannel",
                    ChannelLineGrouping.OneChannelForAllLines);

                DigitalSingleChannelWriter writer = new DigitalSingleChannelWriter(digitalOutTask.Stream);
                digitalOutTask.Control(TaskAction.Verify);

                writer.WriteSingleSampleMultiLine(true, _DO_array);
            }
        }
        catch (DaqException exception)
        {

        }
    }

    //상태 변화를 관찰하기 위해 SystemManager의 Update문에서 DI의 이전 값을 갱신하기위해 호출
    public void UpdateDIPreState()
    {
        //Debug.WriteLine(_DI_array[1] + " " + _DI_array_Pre[1]);
        _DI_array.CopyTo(_DI_array_Pre, 0);
                
    }

    //// Safety 확인
    public bool IsSafetyOn()
    {
        return CheckDIChangeOn(0);
    }
    public bool IsSafetyOff()
    {
        return CheckDIChangeOff(0);
    }

    // Start 확인
    public bool IsStartOn()
    {
        _StartOn = true;

        return CheckDIChangeOn(1);
    }
    public bool IsStartOff()
    {
        _StartOn = false;
        _StartOff = true;

        return CheckDIChangeOff(1);
    }

    // Pause 확인
    public bool IsPauseOn()
    {
        _PauseOn = true;
        return CheckDIChangeOn(2);
    }
    public bool IsPauseOff()
    {
        _PauseOff = true;
        return CheckDIChangeOff(2);
    }

    // Stop 확인
    public bool IsStopOn()
    {
        _StopOn = true;
        return CheckDIChangeOn(3);
    }
    public bool IsStopOff()
    {
        _StopOff = true;
        return CheckDIChangeOff(3);
    }

    //DI 상태가 Off에서 On로 전환되면 true를 반환.
    private bool CheckDIChangeOn(int index)
    {
        if (_DI_array[index] != _DI_array_Pre[index])
        {
            if (_DI_array[index]) return true;
        }
        return false;
    }

    //DI 상태가 On에서 Off로 전환되면 true를 반환.
    private bool CheckDIChangeOff(int index)
    {
        if (_DI_array[index] != _DI_array_Pre[index])
        {
            if (!_DI_array[index]) return true;
        }
        return false;
    }
}

