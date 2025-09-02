using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

using System.IO.Ports;
using System.Windows;
using System.Windows.Threading;

namespace SerialComm

{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private SerialPort serialPort;
        private DispatcherTimer refreshTimer;

        public MainWindow()
        {
            InitializeComponent();
            InitializeSerialPort();
            RefreshPortList();
        }

        private void InitializeSerialPort()
        {
            serialPort = new SerialPort();
            serialPort.DataReceived += SerialPort_DataReceived;
        }

        private void RefreshPortList()
        {
            string currentSelection = cmbPorts.SelectedItem?.ToString();
            cmbPorts.Items.Clear();
            string[] ports = SerialPort.GetPortNames();

            foreach (string port in ports)
            {
                cmbPorts.Items.Add(port);
            }
            if (!string.IsNullOrEmpty(currentSelection) && cmbPorts.Items.Contains(currentSelection))
            {
                cmbPorts.SelectedItem = currentSelection;
            }
            else if (cmbPorts.Items.Count > 0)
            {
                cmbPorts.SelectedIndex = 0;
            }
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string data = serialPort.ReadExisting();
                Dispatcher.Invoke(() =>
                {
                    txtReceived.AppendText(data);
                    txtReceived.ScrollToEnd();
                });
            }
            catch (Exception ex)
            {
                Dispatcher.Invoke(() =>
                    MessageBox.Show($"Error receiving data: {ex.Message}"));
            }
        }

        private void BtnConnect_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (serialPort.IsOpen)
                {
                    serialPort.Close();
                    btnConnect.Content = "Connect";
                }
                else
                {
                    if (cmbPorts.SelectedItem == null)
                    {
                        MessageBox.Show("Please select a COM port");
                        return;
                    }
                    serialPort.PortName = cmbPorts.SelectedItem.ToString();
                    serialPort.BaudRate = 9600;
                    serialPort.Parity = Parity.None;
                    serialPort.DataBits = 8;
                    serialPort.StopBits = StopBits.One;
                    serialPort.Handshake = Handshake.None;
                    serialPort.Open();
                    btnConnect.Content = "Disconnect";
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error: {ex.Message}");
            }
        }

        private void BtnSend_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (serialPort.IsOpen)
                {
                    string data = txtSend.Text;
                    if (chkNewLine.IsChecked == true)
                    {
                        data += Environment.NewLine;
                    }
                    serialPort.Write(data);
                    txtSend.Clear();
                }
                else
                {
                    MessageBox.Show("Serial port is not connected");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error sending data: {ex.Message}");
            }
        }
    }
}