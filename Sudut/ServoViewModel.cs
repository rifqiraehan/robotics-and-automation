using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace Sudut
{
    public enum ServoMode
    {
        Servo0,   // 0..180
        Servo23   // -90..+90
    }

    public partial class ServoViewModel : ObservableObject
    {
        private readonly SerialPort _serialPort = new();

        [ObservableProperty] private string? selectedPort;
        [ObservableProperty] private double sudut = 0;
        [ObservableProperty] private double minPulse = 500;
        [ObservableProperty] private double maxPulse = 2500;
        [ObservableProperty] private double interpolatedPulse;

        // Reverse Mapping
        [ObservableProperty] private bool isReversed = false;

        // Servo yang dikontrol user
        [ObservableProperty] private int selectedServoChannel = 3; // default channel 3
        [ObservableProperty] private ServoMode servoMode = ServoMode.Servo23;
        // [ObservableProperty] private int selectedServoChannel = 0; // default channel 0
        // [ObservableProperty] private ServoMode servoMode = ServoMode.Servo0;

        // === Penjepit (servo 4) ===
        [ObservableProperty] private bool clampClosed;
        public int ClampOpenPulse { get; set; } = 770;
        public int ClampClosePulse { get; set; } = 1700;
        public string ClampButtonText => ClampClosed ? "Close" : "Open";

        public IEnumerable<string> AvailablePorts { get; } = SerialPort.GetPortNames();

        // Range untuk slider
        public double SudutMinimum => ServoMode == ServoMode.Servo0 ? 0 : -90;
        public double SudutMaximum => ServoMode == ServoMode.Servo0 ? 180 : 90;

        public ServoViewModel()
        {
            this.PropertyChanged += (s, e) =>
            {
                if (e.PropertyName is nameof(Sudut) or nameof(MinPulse) or nameof(MaxPulse) or nameof(ServoMode))
                {
                    UpdateInterpolatedPulse();
                    if (_serialPort.IsOpen)
                        _ = SendServoCommandAsync(SelectedServoChannel, (int)InterpolatedPulse);
                }
                else if (e.PropertyName == nameof(ServoMode))
                {
                    OnPropertyChanged(nameof(SudutMinimum));
                    OnPropertyChanged(nameof(SudutMaximum));
                }
            };
        }

        partial void OnClampClosedChanged(bool value)
        {
            OnPropertyChanged(nameof(ClampButtonText));
            if (_serialPort.IsOpen)
                _ = SendServoCommandAsync(4, value ? ClampClosePulse : ClampOpenPulse);
        }

        private void UpdateInterpolatedPulse()
        {
            if (MinPulse >= MaxPulse)
            {
                InterpolatedPulse = 0;
                return;
            }

            switch (ServoMode)
            {
                case ServoMode.Servo0:
                    if (Sudut < 0 || Sudut > 180)
                    {
                        InterpolatedPulse = 0;
                        return;
                    }

                    if (IsReversed)
                        InterpolatedPulse = MaxPulse - (MaxPulse - MinPulse) * (Sudut / 180.0); // reverse
                    else
                        InterpolatedPulse = MinPulse + (MaxPulse - MinPulse) * (Sudut / 180.0); // normal
                    break;

                case ServoMode.Servo23:
                    if (Sudut < -90 || Sudut > 90)
                    {
                        InterpolatedPulse = 0;
                        return;
                    }

                    double normalized = (Sudut + 90) / 180.0;
                    if (IsReversed)
                        InterpolatedPulse = MaxPulse - (MaxPulse - MinPulse) * normalized; // reverse
                    else
                        InterpolatedPulse = MinPulse + (MaxPulse - MinPulse) * normalized; // normal
                    break;
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

                    ToggleClampCommand.NotifyCanExecuteChanged();
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
                    ToggleClampCommand.NotifyCanExecuteChanged();
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

        private async Task SendServoCommandAsync(int channel, int pulse)
        {
            if (!_serialPort.IsOpen || pulse <= 0) return;

            try
            {
                byte[] header = { 0x0D, 0x0A };
                string bodyStr = $"#{channel} P{pulse:D4} S1000 ";
                byte[] body = Encoding.ASCII.GetBytes(bodyStr);
                byte[] tail = { 0x0D, 0x0A, 0x00 };

                byte[] fullCommand = new byte[header.Length + body.Length + tail.Length];
                Array.Copy(header, fullCommand, header.Length);
                Array.Copy(body, 0, fullCommand, header.Length, body.Length);
                Array.Copy(tail, 0, fullCommand, header.Length + body.Length, tail.Length);

                await Task.Run(() => _serialPort.Write(fullCommand, 0, fullCommand.Length));
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error sending servo data: {ex.Message}");
            }
        }

        [RelayCommand]
        private async Task SendServo1PulseAsync()
        {
            // Pastikan serial port terbuka
            if (!_serialPort.IsOpen) return;

            // Tentukan channel dan pulse
            int channel = 1;  // Servo 1
            int pulse = 2275; // Pulse yang diinginkan

            // Kirim perintah
            await SendServoCommandAsync(channel, pulse);
        }

        [RelayCommand(CanExecute = nameof(CanToggleClamp))]
        private void ToggleClamp() => ClampClosed = !ClampClosed;
        private bool CanToggleClamp() => _serialPort.IsOpen;
    }
}