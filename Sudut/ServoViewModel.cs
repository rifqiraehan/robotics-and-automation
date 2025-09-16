using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace Sudut
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

        public IEnumerable<string> AvailablePorts { get; } = SerialPort.GetPortNames(); // Dinamis

        public ServoViewModel()
        {
            // PartialOnPropertyChanged untuk auto-update interpolasi saat Sudut/Min/Max berubah
            this.PropertyChanged += (s, e) =>
            {
                if (e.PropertyName is nameof(Sudut) or nameof(MinPulse) or nameof(MaxPulse))
                {
                    UpdateInterpolatedPulse();
                    if (_serialPort.IsOpen)
                    {
                        SendCommandAsync(); // Kirim command saat nilai berubah
                    }
                }
            };
        }

        private void UpdateInterpolatedPulse()
        {
            if (MinPulse >= MaxPulse || Sudut < 0 || Sudut > 180)
            {
                InterpolatedPulse = 0; // Invalid, bisa tambah error message
                return;
            }

            // Interpolasi linier: y = min + (max - min) * (sudut / 180)
            InterpolatedPulse = MinPulse + (MaxPulse - MinPulse) * (Sudut / 180.0);
        }

        [RelayCommand]
        private void OpenPort()
        {
            try
            {
                if (!_serialPort.IsOpen && !string.IsNullOrEmpty(SelectedPort))
                {
                    _serialPort.PortName = SelectedPort;
                    _serialPort.BaudRate = 115200; // Sesuai PDF
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
            if (!_serialPort.IsOpen || InterpolatedPulse <= 0) return;

            try
            {
                int y = (int)InterpolatedPulse; // Pulse width

                // Bangun command sesuai protokol SSC-32
                byte[] header = { 0x0D, 0x0A };
                string bodyStr = $"#0 P{y:D4} S1000 "; // Channel 3, pulse 4 digits, speed 1000
                byte[] body = Encoding.ASCII.GetBytes(bodyStr);
                byte[] tail = { 0x0D, 0x0A, 0x00 }; // <cr> sebagai \r, tapi gunakan byte

                // Gabung array
                byte[] fullCommand = new byte[header.Length + body.Length + tail.Length];
                Array.Copy(header, 0, fullCommand, 0, header.Length);
                Array.Copy(body, 0, fullCommand, header.Length, body.Length);
                Array.Copy(tail, 0, fullCommand, header.Length + body.Length, tail.Length);

                // Kirim async agar UI responsif
                await Task.Run(() => _serialPort.Write(fullCommand, 0, fullCommand.Length));
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error sending data: {ex.Message}");
            }
        }
    }
}