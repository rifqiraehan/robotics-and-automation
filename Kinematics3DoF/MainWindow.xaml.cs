using System;
using System.IO.Ports;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace Kinematics3DoF
{
    public partial class MainWindow : Window
    {
        private readonly SerialPort _serialPort = new SerialPort();
        private bool _isUpdating = false;

        private double a1 => double.TryParse(TxtA1.Text, out var v1) ? v1 : 120.0;
        private double a2 => double.TryParse(TxtA2.Text, out var v2) ? v2 : 130.0;
        private double a3 => double.TryParse(TxtA3.Text, out var v3) ? v3 : 85.0;

        public MainWindow()
        {
            InitializeComponent();
            LoadPorts();

            BtnCalcSol1.Click += BtnCalcSol_Click;
            BtnCalcSol2.Click += BtnCalcSol_Click;
            BtnCompute.Click += BtnCompute_Click;

            BtnRunSol1.Click += BtnRunSol1_Click;
            BtnRunSol2.Click += BtnRunSol2_Click;
            BtnOpenPort.Click += BtnOpenPort_Click;
            BtnClosePort.Click += BtnClosePort_Click;
            BtnExit.Click += BtnExit_Click;
            BtnOrigin.Click += BtnOrigin_Click;
            BtnCancel.Click += BtnCancel_Click;
            SliderGrip.ValueChanged += SliderGrip_ValueChanged;

            SliderServo0.ValueChanged += SliderServo0_ValueChanged;
            SliderServo2.ValueChanged += SliderServo2_ValueChanged;
            SliderServo3.ValueChanged += SliderServo3_ValueChanged;

            UpdateForwardFromSliders();
        }

        private void LoadPorts()
        {
            CmbPorts.ItemsSource = SerialPort.GetPortNames();
        }

        // ------------------- Slider Events -------------------
        private void SliderServo0_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_isUpdating) return;
            TxtServo0Sudut.Text = SliderServo0.Value.ToString("0");
            UpdateForwardKinematicsUI();
            _ = SendServoJointCommand(0, SliderServo0.Value);
        }

        private void SliderServo2_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_isUpdating) return;
            TxtServo2Sudut.Text = SliderServo2.Value.ToString("0");
            UpdateForwardKinematicsUI();
            _ = SendServoJointCommand(2, SliderServo2.Value);
        }

        private void SliderServo3_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_isUpdating) return;
            TxtServo3Sudut.Text = SliderServo3.Value.ToString("0");
            UpdateForwardKinematicsUI();
            _ = SendServoJointCommand(3, SliderServo3.Value);
        }

        private async void SliderGrip_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_isUpdating) return;

            double pulse = SliderGrip.Value;

            TxtGripInterp.Text = ((int)pulse).ToString();

            double openPercent = (pulse - 770) / (1700 - 770);
            double mm = 380 - 380 * openPercent;
            TxtGripMm.Text = mm.ToString("0.##");

            await SendServoCommandAsync(4, (int)pulse);
        }


        // ------------------- Button Events -------------------
        private void BtnCompute_Click(object sender, RoutedEventArgs e)
        {
            if (!double.TryParse(TxtTheta1Sol1.Text, out var t1) ||
                !double.TryParse(TxtTheta2Sol1.Text, out var t2) ||
                !double.TryParse(TxtTheta3Sol1.Text, out var t3))
            {
                MessageBox.Show("Solusi 1 belum lengkap (θ₁, θ₂, θ₃).");
                return;
            }

            var (qx, qy) = KinematicsSolver.Forward(a1, a2, a3, t1, t2, t3);

            TxtQx.Text = qx.ToString("0.##");
            TxtQy.Text = qy.ToString("0.##");
            TxtOrientasi.Text = (t1 + t2 + t3).ToString("0.##");
        }

        private void BtnCalcSol_Click(object sender, RoutedEventArgs e)
        {
            if (!double.TryParse(TxtQx.Text, out var qx) ||
                !double.TryParse(TxtQy.Text, out var qy) ||
                !double.TryParse(TxtOrientasi.Text, out var phi))
            {
                MessageBox.Show("Masukkan nilai Px, Py, dan Orientasi φ yang valid.");
                return;
            }

            var sols = KinematicsSolver.Inverse(a1, a2, a3, qx, qy, phi);
            if (sols == null)
            {
                MessageBox.Show("Posisi di luar jangkauan robot.");
                return;
            }

            TxtTheta1Sol1.Text = sols[0].theta1Deg.ToString("0.##");
            TxtTheta2Sol1.Text = sols[0].theta2Deg.ToString("0.##");
            TxtTheta3Sol1.Text = sols[0].theta3Deg.ToString("0.##");

            TxtTheta1Sol2.Text = sols[1].theta1Deg.ToString("0.##");
            TxtTheta2Sol2.Text = sols[1].theta2Deg.ToString("0.##");
            TxtTheta3Sol2.Text = sols[1].theta3Deg.ToString("0.##");
        }

        private void BtnRunSol1_Click(object sender, RoutedEventArgs e)
        {
            if (!double.TryParse(TxtTheta1Sol1.Text, out var t1) ||
                !double.TryParse(TxtTheta2Sol1.Text, out var t2) ||
                !double.TryParse(TxtTheta3Sol1.Text, out var t3))
            {
                MessageBox.Show("Solusi 1 belum lengkap.");
                return;
            }

            MoveRobot(t1, t2, t3);
        }

        private void BtnRunSol2_Click(object sender, RoutedEventArgs e)
        {
            if (!double.TryParse(TxtTheta1Sol2.Text, out var t1) ||
                !double.TryParse(TxtTheta2Sol2.Text, out var t2) ||
                !double.TryParse(TxtTheta3Sol2.Text, out var t3))
            {
                MessageBox.Show("Solusi 2 belum lengkap.");
                return;
            }

            MoveRobot(t1, t2, t3);
        }

        private void BtnOpenPort_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (!_serialPort.IsOpen)
                {
                    if (CmbPorts.SelectedItem == null)
                    {
                        MessageBox.Show("Pilih COM port terlebih dahulu.");
                        return;
                    }
                    _serialPort.PortName = CmbPorts.SelectedItem.ToString();
                    _serialPort.BaudRate = 115200;
                    _serialPort.Open();
                    MessageBox.Show($"Port {_serialPort.PortName} dibuka.");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error membuka port: {ex.Message}");
            }
        }

        private void BtnClosePort_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (_serialPort.IsOpen)
                {
                    _serialPort.Close();
                    MessageBox.Show("Port ditutup.");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Error menutup port: {ex.Message}");
            }
        }

        private void BtnOrigin_Click(object sender, RoutedEventArgs e)
        {
            // MoveRobot(0, 0, 0);
            MoveRobot(4, -3, -4);
            _ = SendServoCommandAsync(1, 2220);
        }

        private void BtnCancel_Click(object sender, RoutedEventArgs e)
        {
            BtnOrigin_Click(sender, e);
        }

        private void BtnExit_Click(object sender, RoutedEventArgs e)
        {
            if (_serialPort.IsOpen) _serialPort.Close();
            Application.Current.Shutdown();
        }

        // ------------------- Forward Kinematics Update -------------------
        private void UpdateForwardFromSliders()
        {
            double t1deg = SliderServo0.Value;
            double t2deg = SliderServo2.Value;
            double t3deg = SliderServo3.Value;

            var (qx, qy) = KinematicsSolver.Forward(a1, a2, a3, t1deg, t2deg, t3deg);
            TxtQx.Text = qx.ToString("0.##");
            TxtQy.Text = qy.ToString("0.##");
            TxtOrientasi.Text = (t1deg + t2deg + t3deg).ToString("0.##");
        }

        private void UpdateForwardKinematicsUI()
        {
            UpdateForwardFromSliders();
        }

        // ------------------- Motion + Serial -------------------
        private async Task SendServoJointCommand(int channel, double angleDeg)
        {
            double pulse = AngleToPulse(channel, angleDeg);

            if (channel == 0) TxtServo0Interp.Text = pulse.ToString("0");
            else if (channel == 2) TxtServo2Interp.Text = pulse.ToString("0");
            else if (channel == 3) TxtServo3Interp.Text = pulse.ToString("0");
            else if (channel == 4) TxtGripInterp.Text = pulse.ToString("0");

            await SendServoCommandAsync(channel, (int)pulse);
        }

        private async Task SendServoCommandAsync(int channel, int pulse)
        {
            if (!_serialPort.IsOpen || pulse <= 0) return;
            try
            {
                byte[] header = { 0x0D, 0x0A };
                string bodyStr = $"#{channel} P{pulse:D4} S500 ";
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
                MessageBox.Show($"Error sending servo data: {ex.Message}");
            }
        }

        private double AngleToPulse(int channel, double angleDeg)
        {
            // Default min/max pulse
            double minPulse = 500, maxPulse = 2500;
            double minAngle = -90, maxAngle = 90;

            double normalized = (angleDeg - minAngle) / (maxAngle - minAngle);
            return minPulse + (maxPulse - minPulse) * normalized;
        }

        private void MoveRobot(double t1, double t2, double t3)
        {
            _isUpdating = true;
            try
            {
                SliderServo0.Value = Clamp(t1, -90, 90);
                SliderServo2.Value = Clamp(t2, -90, 90);
                SliderServo3.Value = Clamp(t3, -90, 90);
            }
            finally
            {
                _isUpdating = false;
            }

            UpdateForwardFromSliders();
            _ = SendServoJointCommand(0, t1);
            _ = SendServoJointCommand(2, t2);
            _ = SendServoJointCommand(3, t3);
        }

        private static double Clamp(double v, double min, double max) => Math.Max(min, Math.Min(max, v));
    }

    // ------------------- Kinematics Solver -------------------
    public static class KinematicsSolver
    {
        private static double DegreesToRadians(double d) => d * Math.PI / 180.0;
        private static double RadiansToDegrees(double r) => r * 180.0 / Math.PI;

        public static (double qx, double qy) Forward(double a1, double a2, double a3, double t1Deg, double t2Deg, double t3Deg)
        {
            double t1 = DegreesToRadians(t1Deg);
            double t2 = DegreesToRadians(t2Deg);
            double t3 = DegreesToRadians(t3Deg);

            double qx = a1 * Math.Cos(t1) + a2 * Math.Cos(t1 + t2) + a3 * Math.Cos(t1 + t2 + t3);
            double qy = a1 * Math.Sin(t1) + a2 * Math.Sin(t1 + t2) + a3 * Math.Sin(t1 + t2 + t3);
            return (qx, qy);
        }

        public static (double theta1Deg, double theta2Deg, double theta3Deg)[]? Inverse(double a1, double a2, double a3, double qx, double qy, double phiDeg)
        {
            double phi = DegreesToRadians(phiDeg);

            double px = qx - a3 * Math.Cos(phi);
            double py = qy - a3 * Math.Sin(phi);

            double r2 = px * px + py * py;
            double cosT2 = (r2 - a1 * a1 - a2 * a2) / (2 * a1 * a2);
            if (cosT2 < -1 || cosT2 > 1) return null;

            double t2a = Math.Acos(cosT2);
            double t2b = -t2a;

            double t1a = Math.Atan2(py, px) - Math.Atan2(a2 * Math.Sin(t2a), a1 + a2 * Math.Cos(t2a));
            double t1b = Math.Atan2(py, px) - Math.Atan2(a2 * Math.Sin(t2b), a1 + a2 * Math.Cos(t2b));

            double t3a = phi - (t1a + t2a);
            double t3b = phi - (t1b + t2b);

            return new[]
            {
                (RadiansToDegrees(t1a), RadiansToDegrees(t2a), RadiansToDegrees(t3a)),
                (RadiansToDegrees(t1b), RadiansToDegrees(t2b), RadiansToDegrees(t3b))
            };
        }
    }
}

