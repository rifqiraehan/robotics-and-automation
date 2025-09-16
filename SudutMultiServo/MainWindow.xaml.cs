using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace SudutMultiServo
{
    public partial class ServoViewModel : ObservableObject
    {
        private SerialPort _serialPort = new SerialPort();

        [ObservableProperty]
        private string? selectedPort;

        [ObservableProperty]
        private double sudut = 90; // Default ke 90°

        [ObservableProperty]
        private double minPulse = 500;

        [ObservableProperty]
        private double maxPulse = 2500;

        [ObservableProperty]
        private double interpolatedPulse; // Read-only, dihitung otomatis

        [ObservableProperty]
        private int? selectedServo; // 0, 2, or 3

        [ObservableProperty]
        private bool isGripperOpen; // Untuk toggle switch servo 4

        public IEnumerable<int> ServoOptions { get; } = new List<int> { 0, 2, 3 }; // Opsi servo

        public IEnumerable<string> AvailablePorts { get; } = SerialPort.GetPortNames(); // Dinamis

        public ServoViewModel()
        {
            // PartialOnPropertyChanged untuk auto-update interpolasi saat properti berubah
            this.PropertyChanged += (s, e) =>
            {
                if (e.PropertyName is nameof(Sudut) or nameof(MinPulse) or nameof(MaxPulse) or nameof(SelectedServo) or nameof(IsGripperOpen))
                {
                    UpdateInterpolatedPulse();
                    if (_serialPort.IsOpen)
                    {
                        if (SelectedServo.HasValue) SendCommandAsync(); // Hanya untuk servo 0, 2, 3
                        if (IsGripperOpen) SendGripperCommandAsync(); // Untuk servo 4
                    }
                }
            };
        }

        private void UpdateInterpolatedPulse()
        {
            if (MinPulse >= MaxPulse || Sudut < 0 || Sudut > 180)
            {
                InterpolatedPulse = 0;
                return;
            }

            if (SelectedServo.HasValue)
            {
                switch (SelectedServo.Value)
                {
                    case 0: // 0-180°
                        InterpolatedPulse = MinPulse + (MaxPulse - MinPulse) * (Sudut / 180.0);
                        break;
                    case 2: // -90-90°
                    case 3: // -90-90°
                        double normalizedAngle = Sudut - 90; // Ubah 0-180 ke -90-90
                        InterpolatedPulse = MinPulse + (MaxPulse - MinPulse) * ((normalizedAngle + 90) / 180.0);
                        break;
                    default:
                        InterpolatedPulse = 0;
                        break;
                }
            }
            else
            {
                InterpolatedPulse = 0; // Default jika tidak ada servo dipilih
            }
        }

        [RelayCommand]
        private void OpenPort()
        {
            try
            {
                if (!_serialPort.IsOpen && !string.IsNullOrEmpty(SelectedPort))
                {
                    _serialPort.PortName = SelectedPort;
                    _serialPort.BaudRate = 115200;
                    _serialPort.Parity = Parity.None;
                    _serialPort.DataBits = 8;
                    _serialPort.StopBits = StopBits.One;
                    _serialPort.Handshake = Handshake.None;

                    _serialPort.Open();
                    MessageBox.Show($"Port {SelectedPort} opened.");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error opening port: {ex.Message}");
            }
        }

        [RelayCommand]
        private void ClosePort()
        {
            try
            {
                if (_serialPort.IsOpen)
                {
                    _serialPort.Close();
                    MessageBox.Show("Port closed.");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error closing port: {ex.Message}");
            }
        }

        [RelayCommand]
        private void Cancel()
        {
            if (_serialPort.IsOpen) _serialPort.Close();
            Application.Current.Shutdown();
        }

        private async Task SendCommandAsync()
        {
            if (!_serialPort.IsOpen || !SelectedServo.HasValue || InterpolatedPulse <= 0) return;

            try
            {
                int y = (int)InterpolatedPulse;
                string channel = $"#{SelectedServo.Value}";
                byte[] header = { 0x0D, 0x0A };
                string bodyStr = $"{channel} P{y:D4} S1000\r";
                byte[] body = Encoding.ASCII.GetBytes(bodyStr);
                byte[] tail = { 0x0D, 0x0A, 0x00 };

                byte[] fullCommand = new byte[header.Length + body.Length + tail.Length];
                Array.Copy(header, 0, fullCommand, 0, header.Length);
                Array.Copy(body, 0, fullCommand, header.Length, body.Length);
                Array.Copy(tail, 0, fullCommand, header.Length + body.Length, tail.Length);

                await Task.Run(() => _serialPort.Write(fullCommand, 0, fullCommand.Length));
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error sending data: {ex.Message}");
            }
        }

        private async Task SendGripperCommandAsync()
        {
            if (!_serialPort.IsOpen) return;

            try
            {
                int pulse = IsGripperOpen ? 2500 : 500; // 2500 untuk buka, 500 untuk tutup
                byte[] header = { 0x0D, 0x0A };
                string bodyStr = "#4 P{pulse:D4} S1000\r";
                byte[] body = Encoding.ASCII.GetBytes(bodyStr.Replace("{pulse}", pulse.ToString()));
                byte[] tail = { 0x0D, 0x0A, 0x00 };

                byte[] fullCommand = new byte[header.Length + body.Length + tail.Length];
                Array.Copy(header, 0, fullCommand, 0, header.Length);
                Array.Copy(body, 0, fullCommand, header.Length, body.Length);
                Array.Copy(tail, 0, fullCommand, header.Length + body.Length, tail.Length);

                await Task.Run(() => _serialPort.Write(fullCommand, 0, fullCommand.Length));
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error sending gripper data: {ex.Message}");
            }
        }
    }
}