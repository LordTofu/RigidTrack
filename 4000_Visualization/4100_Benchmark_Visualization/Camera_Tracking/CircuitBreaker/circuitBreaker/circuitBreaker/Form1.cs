using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace circuitBreaker
{
    public partial class Form1 : Form
    {

        static void SendUdp(int srcPort, string dstIp, int dstPort, byte[] data)
        {
            using (UdpClient c = new UdpClient(srcPort))
                c.Send(data, data.Length, dstIp, dstPort);
        }

        public Form1()
        {
            InitializeComponent();

        }

        private void Form1_Load(object sender, EventArgs e)
        {
            timer1.Start();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            SendUdp(5150, "192.168.4.1", 5150, Encoding.ASCII.GetBytes("9"));
            SendUdp(5150, "192.168.4.1", 5150, Encoding.ASCII.GetBytes("1"));
        }

        private void button2_Click(object sender, EventArgs e)
        {
            SendUdp(5150, "192.168.4.1", 5150, Encoding.ASCII.GetBytes("0"));
        }

        private void timer1_Tick(object sender, EventArgs e)
        {

            // TODO implement is key down here ...
        }
    }

}


