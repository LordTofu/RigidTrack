﻿using System;
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
using System.Net;
using System.Net.NetworkInformation;

namespace circuitBreaker
{
    public partial class Form1 : Form
    {
        int timerCount = 0;
        int port = 9156;
        string ip = "192.168.4.1";
        bool spacePressed = false;
        //byte[] dataMessage = { 0x46, 0x24, 0x10, 0x00, 0x46, 0x24, 0x10, 0x00, 0x46, 0x24, 0x10, 0x00, 0x46, 0x24, 0x10, 0x00 };

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
            button1.Enabled = false;
        }


        private void timer1_Tick(object sender, EventArgs e)
        {
            SendUdp(port, ip, port, Encoding.ASCII.GetBytes("1"));
            //SendUdp(port, ip, port, dataMessage);
        }

        private void btnArm(object sender, EventArgs e)
        {
            button1.BackColor = Color.Green;
            port =  Int32.Parse(tbPort.Text.ToString());
            ip = tbIP.Text.ToString();
            if (spacePressed)
            {
                SendUdp(port, ip, port, Encoding.ASCII.GetBytes("9"));
                SendUdp(port, ip, port, Encoding.ASCII.GetBytes("1"));
                timer1.Start();
            }
        }

        private void textBox1_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Space)
            {
                spacePressed = true;
                button1.Enabled = true;
            }
        }

        private void textBox1_KeyUp(object sender, KeyEventArgs e)
        {
            button1.BackColor = Color.Red;
            SendUdp(port, ip, port, Encoding.ASCII.GetBytes("0"));
            spacePressed = false;
            button1.Enabled = false;
            timer1.Stop();
        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {

        }
    }

}


