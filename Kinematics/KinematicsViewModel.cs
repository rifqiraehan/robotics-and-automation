using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace Kinematics
{
    public partial class KinematicsViewModel : ObservableObject
    {
        private readonly SerialPort _serialPort = new();

        // === Robot Parameters ===
        [ObservableProperty] private double a1 = 110;   // panjang link 1
        [ObservableProperty] private double a2 = 190;   // panjang link 2

        // === Forward Kinematics Result ===
        [ObservableProperty] private double px;
        [ObservableProperty] private double py;

        // === Inverse Kinematics Results ===
        [ObservableProperty] private double theta1Sol1;
        [ObservableProperty] private double theta2Sol1;
        [ObservableProperty] private double theta1Sol2;
        [ObservableProperty] private double theta2Sol2;

        // === COM Port ===
        [ObservableProperty] private string? selectedPort;
        public IEnumerable<string> AvailablePorts { get; } = SerialPort.GetPortNames();

        // === Servo ViewModels ===
        public ServoViewModel Servo0 { get; } = new(0);
        public ServoViewModel Servo2 { get; } = new(2);
        public ServoViewModel Servo3 { get; } = new(3);
        public ServoViewModel Servo4 { get; } = new(4);

        public KinematicsViewModel()
        {
            // Helper untuk semua servo
            AttachServoHandler(Servo0, true);
            AttachServoHandler(Servo2, true);
            AttachServoHandler(Servo3, true);
            AttachServoHandler(Servo4, false);

            CalculateForward(); // initial
        }

        private void AttachServoHandler(ServoViewModel servo, bool triggerForward)
        {
            servo.PropertyChanged += (s, e) =>
            {
                if (triggerForward && e.PropertyName == nameof(ServoViewModel.Sudut))
                    CalculateForward();
                if (e.PropertyName == nameof(ServoViewModel.InterpolatedPulse))
                    _ = SendServoCommandAsync(servo.Channel, (int)servo.InterpolatedPulse);
            };
        }

        // === Forward Kinematics ===
        [RelayCommand]
        private void CalculateForward()
        {
            (Px, Py) = KinematicsSolver.Forward(A1, A2, Servo2.Sudut, Servo3.Sudut);
        }

        // === Inverse Kinematics ===
        [RelayCommand]
        private void CalculateInverse()
        {
            var solutions = KinematicsSolver.Inverse(A1, A2, Px, Py);

            Theta1Sol1 = solutions[0].theta1Deg;
            Theta2Sol1 = solutions[0].theta2Deg;
            Theta1Sol2 = solutions[1].theta1Deg;
            Theta2Sol2 = solutions[1].theta2Deg;
        }

        [RelayCommand]
        private void RunSolution1()
        {
            Servo2.Sudut = Theta1Sol1;
            Servo3.Sudut = Theta2Sol1;
        }

        [RelayCommand]
        private void RunSolution2()
        {
            Servo2.Sudut = Theta1Sol2;
            Servo3.Sudut = Theta2Sol2;
        }

        // === COM Port Open/Close ===
        [RelayCommand]
        private void OpenPort()
        {
            try
            {
                if (!_serialPort.IsOpen && !string.IsNullOrEmpty(SelectedPort))
                {
                    _serialPort.PortName = SelectedPort;
                    _serialPort.BaudRate = 115200;
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

        // === Origin Position ===
        [RelayCommand]
        private void Origin()
        {
            Servo0.Sudut = 90;  // posisi tengah
            Servo2.Sudut = 0;
            Servo3.Sudut = 0;
            Servo4.Sudut = 0;   // Jepit default
        }

        [RelayCommand]
        private void Exit()
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
                Array.Copy(header, 0, fullCommand, 0, header.Length);
                Array.Copy(body, 0, fullCommand, header.Length, body.Length);
                Array.Copy(tail, 0, fullCommand, header.Length + body.Length, tail.Length);
                await Task.Run(() => _serialPort.Write(fullCommand, 0, fullCommand.Length));
            }
            catch (Exception ex)
            {
                await Application.Current.Dispatcher.InvokeAsync(() =>
                    MessageBox.Show($"Error sending servo data: {ex.Message}")
                );
            }
        }
    }

    // =======================
    // Servo ViewModel
    // =======================
    public partial class ServoViewModel : ObservableObject
    {
        public int Channel { get; }

        [ObservableProperty] private double sudut;
        [ObservableProperty] private double minPulse = 500;
        [ObservableProperty] private double maxPulse = 2500;
        [ObservableProperty] private double interpolatedPulse;

        public double GripperOpenMm { get; set; } = 1700;
        public double GripperCloseMm { get; set; } = 770;

        public ServoViewModel(int channel)
        {
            Channel = channel;
            Sudut = channel == 0 ? 90 : 0; // default sudut
            UpdateInterpolatedPulse();

            this.PropertyChanged += (s, e) =>
            {
                if (e.PropertyName is nameof(Sudut) or nameof(MinPulse) or nameof(MaxPulse))
                {
                    UpdateInterpolatedPulse();
                    OnPropertyChanged(nameof(GripMm));
                }
            };
        }

        private void UpdateInterpolatedPulse()
        {
            if (MinPulse >= MaxPulse) { InterpolatedPulse = 0; return; }

            if (Channel == 0 || Channel == 4)
            {
                if (Sudut < 0 || Sudut > 180) { InterpolatedPulse = 0; return; }
                InterpolatedPulse = MinPulse + (MaxPulse - MinPulse) * (Sudut / 180.0);
            }
            else
            {
                if (Sudut < -90 || Sudut > 90) { InterpolatedPulse = 0; return; }
                double normalized = (Sudut + 90) / 180.0;
                InterpolatedPulse = MinPulse + (MaxPulse - MinPulse) * normalized;
            }
        }

        public double GripMm
        {
            get
            {
                if (Channel != 4) return 0;
                double normalized = (Sudut / 180.0);
                return GripperOpenMm + (GripperCloseMm - GripperOpenMm) * normalized;
            }
        }
    }

    // =======================
    // Kinematics Solver
    // =======================
    public static class KinematicsSolver
    {
        public static (double px, double py) Forward(double a1, double a2, double theta1Deg, double theta2Deg)
        {
            double t1 = theta1Deg * Math.PI / 180.0;
            double t2 = theta2Deg * Math.PI / 180.0;

            double px = a1 * Math.Cos(t1) + a2 * Math.Cos(t1 + t2);
            double py = a1 * Math.Sin(t1) + a2 * Math.Sin(t1 + t2);

            return (px, py);
        }

        public static (double theta1Deg, double theta2Deg)[] Inverse(double a1, double a2, double px, double py)
        {
            double cosT2 = (px * px + py * py - a1 * a1 - a2 * a2) / (2 * a1 * a2);
            cosT2 = Math.Max(-1, Math.Min(1, cosT2));
            double t2a = Math.Acos(cosT2);
            double t2b = -t2a;

            double t1a = Math.Atan2(py, px) - Math.Atan2(a2 * Math.Sin(t2a), a1 + a2 * Math.Cos(t2a));
            double t1b = Math.Atan2(py, px) - Math.Atan2(a2 * Math.Sin(t2b), a1 + a2 * Math.Cos(t2b));

            return new[]
            {
                (t1a * 180.0 / Math.PI, t2a * 180.0 / Math.PI),
                (t1b * 180.0 / Math.PI, t2b * 180.0 / Math.PI)
            };
        }
    }
}
