using System;
using System.IO.Ports;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;

namespace Kinematics
{
    public partial class MainWindow : Window
    {
        private readonly SerialPort _serialPort = new SerialPort();
        private bool _isUpdating = false;
        private double a1 => double.TryParse(TxtA1.Text, out var v1) ? v1 : 120.0;
        private double a2 => double.TryParse(TxtA2.Text, out var v2) ? v2 : 210.0;

        private const double GripPulseOpen = 770.0;   // µs -> corresponds to max open mm
        private const double GripPulseClose = 1700.0; // µs -> corresponds to fully closed (0 mm)
        private const double GripOpenMm = 380.0;      // mm at pulse = 770
        private const double GripCloseMm = 0.0;       // mm at pulse = 1700

        public MainWindow()
        {
            InitializeComponent();
            LoadPorts();

            // Px/Py editable at runtime
            TxtPx.IsReadOnly = false;
            TxtPy.IsReadOnly = false;
            TxtA1.IsReadOnly = false;
            TxtA2.IsReadOnly = false;

            SliderServo2.ValueChanged += SliderServo2_ValueChanged;
            SliderServo3.ValueChanged += SliderServo3_ValueChanged;
            SliderServo0.ValueChanged += SliderServo0_ValueChanged;

            // SliderServo3.ValueChanged += (s, e) => UpdateForwardFromSliders(); // ensure FK updates
            // SliderServo2.ValueChanged += (s, e) => UpdateForwardFromSliders();

            TxtServo2Sudut.LostFocus += TxtServo2Sudut_LostFocus;
            TxtServo3Sudut.LostFocus += TxtServo3Sudut_LostFocus;
            TxtPx.LostFocus += TxtPx_LostFocus;
            TxtPy.LostFocus += TxtPy_LostFocus;

            SliderGrip.ValueChanged += SliderGrip_ValueChanged;

            BtnCalcSol1.Click += BtnCalcSol_Click;
            BtnCalcSol2.Click += BtnCalcSol_Click;
            BtnRunSol1.Click += BtnRunSol1_Click;
            BtnRunSol2.Click += BtnRunSol2_Click;
            BtnCompute.Click += BtnCompute_Click;

            BtnOpenPort.Click += BtnOpenPort_Click;
            BtnClosePort.Click += BtnClosePort_Click;
            BtnOrigin.Click += BtnOrigin_Click;
            BtnExit.Click += BtnExit_Click;
            BtnCancel.Click += BtnCancel_Click;

            TxtServo2Sudut.Text = SliderServo2.Value.ToString("0");
            TxtServo3Sudut.Text = SliderServo3.Value.ToString("0");
            TxtServo0Sudut.Text = SliderServo0.Value.ToString("0");

            TxtGripMin.Text = GripPulseOpen.ToString("0");
            TxtGripMax.Text = GripPulseClose.ToString("0");
            SliderGrip.Minimum = GripPulseOpen;
            SliderGrip.Maximum = GripPulseClose;
            SliderGrip.Value = GripPulseOpen;
            UpdateGripFieldsFromPulse(SliderGrip.Value);

            UpdateForwardFromSliders();
        }
        private void LoadPorts()
        {
            var ports = SerialPort.GetPortNames();
            CmbPorts.ItemsSource = ports;
        }

        // -----------------------
        // Event handlers - Sliders
        // -----------------------
        private void SliderServo2_ValueChanged(object? sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_isUpdating) return;

            TxtServo2Sudut.Text = SliderServo2.Value.ToString("0");

            UpdateForwardKinematicsUI();

            _ = SendServoJointCommand(2, SliderServo2.Value);
        }

        private void SliderServo3_ValueChanged(object? sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_isUpdating) return;

            TxtServo3Sudut.Text = SliderServo3.Value.ToString("0");

            UpdateForwardKinematicsUI();

            _ = SendServoJointCommand(3, SliderServo3.Value);
        }

        private void SliderServo0_ValueChanged(object? sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_isUpdating) return;

            double angle = SliderServo0.Value;
            TxtServo0Sudut.Text = ((int)angle).ToString();

            UpdateForwardKinematicsUI();

            _ = SendServoJointCommand(0, angle);
        }

        // Grip slider
        private async void SliderGrip_ValueChanged(object? sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_isUpdating) return;

            var pulse = SliderGrip.Value;
            TxtGripInterp.Text = ((int)pulse).ToString();
            TxtGripMm.Text = PulseToGripMm(pulse).ToString("0.00");
            await SendServoCommandAsync(4, (int)pulse);
        }

        private void BtnCompute_Click(object? sender, RoutedEventArgs e)
        {

            if (!double.TryParse(TxtTheta1Sol1.Text, out var t1) ||
                !double.TryParse(TxtTheta2Sol1.Text, out var t2))
            {
                MessageBox.Show("Solusi 1 (Theta 1 atau Theta 2) belum terisi atau nilainya tidak valid (numeric).");
                return;
            }

            var (px, py) = KinematicsSolver.Forward(a1, a2, t1, t2);

            _isUpdating = true;
            try
            {
                TxtPx.Text = px.ToString("0.##");
                TxtPy.Text = py.ToString("0.##");
            }
            finally
            {
                _isUpdating = false;
            }
        }

        // -----------------------
        // Textbox lost-focus events (user typed angles / Px/Py)
        // -----------------------
        private void TxtServo2Sudut_LostFocus(object? sender, RoutedEventArgs e)
        {
            if (double.TryParse(TxtServo2Sudut.Text, out var val))
            {
                val = Clamp(val, -90, 90);
                _isUpdating = true;
                try
                {
                    SliderServo2.Value = val;
                }
                finally
                {
                    _isUpdating = false;
                }
                UpdateForwardKinematicsUI();
                _ = SendServoJointCommand(2, SliderServo2.Value);
            }
            else
            {
                TxtServo2Sudut.Text = SliderServo2.Value.ToString("0");
            }
        }

        private void TxtServo3Sudut_LostFocus(object? sender, RoutedEventArgs e)
        {
            if (double.TryParse(TxtServo3Sudut.Text, out var val))
            {
                val = Clamp(val, -90, 90);
                _isUpdating = true;
                try
                {
                    SliderServo3.Value = val;
                }
                finally
                {
                    _isUpdating = false;
                }
                UpdateForwardKinematicsUI();
                _ = SendServoJointCommand(3, SliderServo3.Value);
            }
            else
            {
                TxtServo3Sudut.Text = SliderServo3.Value.ToString("0");
            }
        }

        private void TxtPx_LostFocus(object? sender, RoutedEventArgs e)
        {
            if (!double.TryParse(TxtPx.Text, out _))
                UpdateForwardFromSliders();
        }

        private void TxtPy_LostFocus(object? sender, RoutedEventArgs e)
        {
            if (!double.TryParse(TxtPy.Text, out _))
                UpdateForwardFromSliders();
        }

        // -----------------------
        // Buttons - Calculate / Run / Open/Close / Origin / Exit / Cancel
        // -----------------------
        private void BtnCalcSol_Click(object? sender, RoutedEventArgs e)
        {
            if (!double.TryParse(TxtPx.Text, out var px) || !double.TryParse(TxtPy.Text, out var py))
            {
                MessageBox.Show("Masukkan nilai Px dan Py yang valid (numeric).");
                return;
            }

            var sols = KinematicsSolver.Inverse(a1, a2, px, py);
            if (sols == null)
            {
                MessageBox.Show("Posisi di luar jangkauan robot.");
                return;
            }

            TxtTheta1Sol1.Text = sols[0].theta1Deg.ToString("0.##");
            TxtTheta2Sol1.Text = sols[0].theta2Deg.ToString("0.##");
            TxtTheta1Sol2.Text = sols[1].theta1Deg.ToString("0.##");
            TxtTheta2Sol2.Text = sols[1].theta2Deg.ToString("0.##");
        }

        private void BtnRunSol1_Click(object? sender, RoutedEventArgs e)
        {
            if (!double.TryParse(TxtTheta1Sol1.Text, out var t1) || !double.TryParse(TxtTheta2Sol1.Text, out var t2))
            {
                MessageBox.Show("Solusi 1 belum terisi. Tekan Calculate terlebih dahulu.");
                return;
            }

            t1 = Clamp(t1, -90, 90);
            t2 = Clamp(t2, -90, 90);
            _isUpdating = true;
            try
            {
                SliderServo0.Value = t1;
                SliderServo2.Value = t2;

                TxtServo0Sudut.Text = t1.ToString("0");
                TxtServo2Sudut.Text = t2.ToString("0");
            }
            finally
            {
                _isUpdating = false;
            }
            // UpdateForwardFromSliders();
            _ = SendServoJointCommand(0, t1);
            _ = SendServoJointCommand(2, t2);
        }

        private void BtnRunSol2_Click(object? sender, RoutedEventArgs e)
        {
            if (!double.TryParse(TxtTheta1Sol2.Text, out var t1) || !double.TryParse(TxtTheta2Sol2.Text, out var t2))
            {
                MessageBox.Show("Solusi 2 belum terisi. Tekan Calculate terlebih dahulu.");
                return;
            }

            t1 = Clamp(t1, -90, 90);
            t2 = Clamp(t2, -90, 90);
            _isUpdating = true;
            try
            {
                SliderServo0.Value = t1;
                SliderServo2.Value = t2;

                TxtServo0Sudut.Text = t1.ToString("0");
                TxtServo2Sudut.Text = t2.ToString("0");
            }
            finally
            {
                _isUpdating = false;
            }
            // UpdateForwardFromSliders();
            _ = SendServoJointCommand(0, t1);
            _ = SendServoJointCommand(2, t2);
        }

        private void BtnOpenPort_Click(object? sender, RoutedEventArgs e)
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

        private void BtnClosePort_Click(object? sender, RoutedEventArgs e)
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

        private void BtnOrigin_Click(object? sender, RoutedEventArgs e)
        {
            const double T0 = 91;
            const double T2 = 0;
            const double T3 = 3;

            _isUpdating = true;
            try
            {
                SliderServo0.Value = T0;
                SliderServo2.Value = T2;
                SliderServo3.Value = T3;
                SliderGrip.Value = GripPulseOpen;

                TxtServo0Sudut.Text = T0.ToString("0");
                TxtServo2Sudut.Text = T2.ToString("0");
                TxtServo3Sudut.Text = T3.ToString("0");

                _ = SendServoCommandAsync(1, 2250);
            }
            finally
            {
                _isUpdating = false;
            }
            UpdateForwardFromSliders();
        }

        private void BtnExit_Click(object? sender, RoutedEventArgs e)
        {
            if (_serialPort.IsOpen) _serialPort.Close();
            Application.Current.Shutdown();
        }

        private void BtnCancel_Click(object? sender, RoutedEventArgs e)
        {
            BtnOrigin_Click(sender, e);
        }

        // -----------------------
        // Helpers - FK / IK / Clamp / Conversions
        // -----------------------
        private async void UpdateForwardFromSliders()
        {
            double t1deg = SliderServo0.Value;
            double t2deg = SliderServo2.Value;

            var (px, py) = KinematicsSolver.Forward(a1, a2, t1deg, t2deg);

            TxtPx.Text = px.ToString("0.##");
            TxtPy.Text = py.ToString("0.##");

            var pulse0 = AngleToPulse(0, t1deg);
            var pulse2 = AngleToPulse(2, t2deg);
            var pulse3 = AngleToPulse(3, SliderServo3.Value);

            TxtServo0Interp.Text = pulse0.ToString("0");
            TxtServo2Interp.Text = pulse2.ToString("0");
            TxtServo3Interp.Text = pulse3.ToString("0");

            await SendServoCommandAsync(0, (int)pulse0);
            await SendServoCommandAsync(2, (int)pulse2);
            await SendServoCommandAsync(3, (int)pulse3);
        }

        private static double Clamp(double v, double min, double max) => Math.Max(min, Math.Min(max, v));

        private double AngleToPulse(int channel, double angleDeg)
        {
            double minPulse = double.TryParse(
                channel == 0 ? TxtServo0Min.Text :
                channel == 2 ? TxtServo2Min.Text :
                channel == 3 ? TxtServo3Min.Text :
                TxtGripMin.Text, out var mp) ? mp : 500;

            double maxPulse = double.TryParse(
                channel == 0 ? TxtServo0Max.Text :
                channel == 2 ? TxtServo2Max.Text :
                channel == 3 ? TxtServo3Max.Text :
                TxtGripMax.Text, out var xp) ? xp : 2500;

            double minAngleDeg, maxAngleDeg;

            if (channel == 0)
            {
                minAngleDeg = -90;
                maxAngleDeg = 90;
            }
            else if (channel == 2 || channel == 3)
            {
                minAngleDeg = -90;
                maxAngleDeg = 90;
            }
            else
            {
                minAngleDeg = 0;
                maxAngleDeg = 180;
            }

            double angle = Clamp(angleDeg, minAngleDeg, maxAngleDeg);

            double normalized = (angle - minAngleDeg) / (maxAngleDeg - minAngleDeg);
            return minPulse + (maxPulse - minPulse) * normalized;
        }

        private double PulseToGripMm(double pulse)
        {
            // linear map: pulse = GripPulseOpen -> mm = GripOpenMm
            //             pulse = GripPulseClose -> mm = 0
            var p = Clamp(pulse, GripPulseOpen, GripPulseClose);
            double mm = (GripPulseClose - p) / (GripPulseClose - GripPulseOpen) * GripOpenMm;
            return mm;
        }

        private double GripMmToPulse(double mm)
        {
            var m = Clamp(mm, GripCloseMm, GripOpenMm);
            double pulse = GripPulseClose - (m / GripOpenMm) * (GripPulseClose - GripPulseOpen);
            return pulse;
        }

        private void UpdateGripFieldsFromPulse(double pulse)
        {
            TxtGripMm.Text = PulseToGripMm(pulse).ToString("0.##");
            TxtGripInterp.Text = ((int)pulse).ToString();
        }

        // Serial send helper
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

        private void UpdateForwardKinematicsUI()
        {
            double t1deg = SliderServo0.Value;
            double t2deg = SliderServo2.Value;

            var (px, py) = KinematicsSolver.Forward(a1, a2, t1deg, t2deg);

            TxtPx.Text = px.ToString("0.###");
            TxtPy.Text = py.ToString("0.###");
        }

        private async Task SendServoJointCommand(int channel, double angleDeg)
        {
            if (_isUpdating) return;

            double pulse = AngleToPulse(channel, angleDeg);

            if (channel == 0) TxtServo0Interp.Text = pulse.ToString("0");
            else if (channel == 2) TxtServo2Interp.Text = pulse.ToString("0");
            else if (channel == 3) TxtServo3Interp.Text = pulse.ToString("0");

            await SendServoCommandAsync(channel, (int)pulse);
        }
    }

    // Kinematics solver
    public static class KinematicsSolver
    {
        private static double DegreesToRadians(double d) => d * Math.PI / 180.0;
        private static double RadiansToDegrees(double r) => r * 180.0 / Math.PI;

        public static (double px, double py) Forward(double a1, double a2, double theta1Deg, double theta2Deg)
        {

            double t1 = DegreesToRadians(theta1Deg);
            double t2 = DegreesToRadians(theta2Deg);

            double px = a1 * Math.Cos(t1) + a2 * Math.Cos(t1 + t2);
            double py = a1 * Math.Sin(t1) + a2 * Math.Sin(t1 + t2);
            return (px, py);
        }

        public static (double theta1Deg, double theta2Deg)[]? Inverse(double a1, double a2, double px, double py)
        {
            double r2 = px * px + py * py;

            if (r2 > Math.Pow(a1 + a2, 2) * 1.00001 ||
                r2 < Math.Pow(Math.Abs(a1 - a2), 2) * 0.99999)
                return null;


            double cosT2 = (r2 - a1 * a1 - a2 * a2) / (2 * a1 * a2);
            cosT2 = Math.Max(-1.0, Math.Min(1.0, cosT2));

            double t2a = Math.Acos(cosT2);
            double t2b = -t2a;

            double t1a_std = Math.Atan2(py, px) - Math.Atan2(a2 * Math.Sin(t2a), a1 + a2 * Math.Cos(t2a));
            double t1b_std = Math.Atan2(py, px) - Math.Atan2(a2 * Math.Sin(t2b), a1 + a2 * Math.Cos(t2b));

            double t1a_offset = RadiansToDegrees(t1a_std);
            double t1b_offset = RadiansToDegrees(t1b_std);

            t1a_offset = NormalizeAngle(t1a_offset);
            t1b_offset = NormalizeAngle(t1b_offset);

            return new[]
            {
            (t1a_offset, RadiansToDegrees(t2a)),
            (t1b_offset, RadiansToDegrees(t2b))
        };
        }

        private static double NormalizeAngle(double angle)
        {
            angle %= 360.0;
            if (angle > 180.0) angle -= 360.0;
            else if (angle < -180.0) angle += 360.0;
            return angle;
        }
    }
}