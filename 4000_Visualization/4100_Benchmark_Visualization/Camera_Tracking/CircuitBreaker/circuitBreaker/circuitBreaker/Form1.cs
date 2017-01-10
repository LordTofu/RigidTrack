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
using System.Windows.Input;

namespace circuitBreaker
{
    public partial class Form1 : Form
    {
        int port = 5000;
        string ip = "192.168.4.1";
        bool spacePressed = false;

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
           
        }


        private void timer1_Tick(object sender, EventArgs e)
        {
                SendUdp(port, ip, port, Encoding.ASCII.GetBytes("1"));
        }

        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
           if(e.KeyCode == Keys.Space)
            {
                spacePressed = true;
            }
        }

        private void btnArm(object sender, EventArgs e)
        {
            port =  Int32.Parse(tbPort.Text.ToString());
            ip = tbIP.Text.ToString();
            if(spacePressed)
            {
                SendUdp(port, ip, port, Encoding.ASCII.GetBytes("9"));
                SendUdp(port, ip, port, Encoding.ASCII.GetBytes("1"));
                timer1.Start();
            }
        }

        private void button1_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Space)
            {
                spacePressed = true;
            }
        }

        private void button1_KeyUp(object sender, KeyEventArgs e)
        {
            SendUdp(port, ip, port, Encoding.ASCII.GetBytes("0"));
            spacePressed = false;
            timer1.Stop();
        }

       
    }

}


