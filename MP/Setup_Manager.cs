using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

public class Setup_Manager
{
    public Setup _gain { get; private set; }

    public Setup_Manager()
    {

    }
    public void SocketUDPCreateGain(UdpClient client, IPEndPoint ipEndPoint)
    {
        if (_gain != null) return;
        _gain = new Setup(client, ipEndPoint);
    }
}