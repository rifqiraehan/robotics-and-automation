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
        private double a1 => double.TryParse(TxtA1.Text, out var v1) ? v1 : 120.0;
        private double a2 => double.TryParse(TxtA2.Text, out var v2) ? v2 : 213.0;

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

            // Wire events (sliders <-> textboxes)
            SliderServo2.ValueChanged += SliderServo2_ValueChanged;
            SliderServo3.ValueChanged += SliderServo3_ValueChanged;
            SliderServo0.ValueChanged += SliderServo0_ValueChanged;

            SliderServo3.ValueChanged += (s, e) => UpdateForwardFromSliders(); // ensure FK updates
            SliderServo2.ValueChanged += (s, e) => UpdateForwardFromSliders();

            TxtServo2Sudut.LostFocus += TxtServo2Sudut_LostFocus;
            TxtServo3Sudut.LostFocus += TxtServo3Sudut_LostFocus;
            TxtPx.LostFocus += TxtPx_LostFocus;
            TxtPy.LostFocus += TxtPy_LostFocus;

            SliderGrip.ValueChanged += SliderGrip_ValueChanged;

            BtnCalcSol1.Click += BtnCalcSol_Click;
            BtnCalcSol2.Click += BtnCalcSol_Click;
            BtnRunSol1.Click += BtnRunSol1_Click;
            BtnRunSol2.Click += BtnRunSol2_Click;

            BtnOpenPort.Click += BtnOpenPort_Click;
            BtnClosePort.Click += BtnClosePort_Click;
            BtnOrigin.Click += BtnOrigin_Click;
            BtnExit.Click += BtnExit_Click;
            BtnCancel.Click += BtnCancel_Click;

            // Initialize UI values (use the sliders' defaults)
            TxtServo2Sudut.Text = SliderServo2.Value.ToString("0");
            TxtServo3Sudut.Text = SliderServo3.Value.ToString("0");
            TxtServo0Sudut.Text = SliderServo0.Value.ToString("0");

            // Initialize grip fields
            TxtGripMin.Text = GripPulseOpen.ToString("0");
            TxtGripMax.Text = GripPulseClose.ToString("0");
            SliderGrip.Minimum = GripPulseOpen;
            SliderGrip.Maximum = GripPulseClose;
            SliderGrip.Value = GripPulseOpen; // open by default
            UpdateGripFieldsFromPulse(SliderGrip.Value);

            // Compute initial FK
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
        private async void SliderServo0_ValueChanged(object? sender, RoutedPropertyChangedEventArgs<double> e)
        {
            double angle = SliderServo0.Value;
            TxtServo0Sudut.Text = ((int)angle).ToString();
            TxtServo0Interp.Text = AngleToPulse(0, angle).ToString("0");

            await SendServoCommandAsync(0, (int)AngleToPulse(0, angle));
        }


        private void SliderServo2_ValueChanged(object? sender, RoutedPropertyChangedEventArgs<double> e)
        {
            TxtServo2Sudut.Text = SliderServo2.Value.ToString("0");
        }

        private void SliderServo3_ValueChanged(object? sender, RoutedPropertyChangedEventArgs<double> e)
        {
            TxtServo3Sudut.Text = SliderServo3.Value.ToString("0");
        }

        // Grip slider
        private async void SliderGrip_ValueChanged(object? sender, RoutedPropertyChangedEventArgs<double> e)
        {
            var pulse = SliderGrip.Value;
            TxtGripInterp.Text = ((int)pulse).ToString();
            TxtGripMm.Text = PulseToGripMm(pulse).ToString("0.00");
            await SendServoCommandAsync(4, (int)pulse);
        }

        // -----------------------
        // Textbox lost-focus events (user typed angles / Px/Py)
        // -----------------------
        private void TxtServo2Sudut_LostFocus(object? sender, RoutedEventArgs e)
        {
            if (double.TryParse(TxtServo2Sudut.Text, out var val))
            {
                if (val < -90) val = -90;
                if (val > 90) val = 90;
                SliderServo2.Value = val;
                UpdateForwardFromSliders();
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
                if (val < -90) val = -90;
                if (val > 90) val = 90;
                SliderServo3.Value = val;
                UpdateForwardFromSliders();
            }
            else
            {
                TxtServo3Sudut.Text = SliderServo3.Value.ToString("0");
            }
        }

        private void TxtPx_LostFocus(object? sender, RoutedEventArgs e)
        {
            // If user manually changed Px/Py, compute IK immediately or leave to Calculate button.
            if (!double.TryParse(TxtPx.Text, out _))
                UpdateForwardFromSliders(); // reset to computed FK
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

            // Fill solution text fields
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

            // Apply to sliders (clamp to allowed ranges)
            t1 = Clamp(t1, -90, 90);
            t2 = Clamp(t2, -90, 90);
            SliderServo2.Value = t1;
            SliderServo3.Value = t2;
            UpdateForwardFromSliders();
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
            SliderServo2.Value = t1;
            SliderServo3.Value = t2;
            UpdateForwardFromSliders();
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
            SliderServo0.Value = 91;
            SliderServo2.Value = 0;
            SliderServo3.Value = 3;
            SliderGrip.Value = GripPulseOpen;
            UpdateForwardFromSliders();

            _ = SendServoCommandAsync(1, 2250);
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
        private void UpdateForwardFromSliders()
        {
            double servo0Deg = SliderServo0.Value;
            double t1deg = SliderServo2.Value;
            double t2deg = SliderServo3.Value;

            // Forward kinematics
            var (px, py) = KinematicsSolver.Forward(a1, a2, t1deg, t2deg);

            TxtPx.Text = px.ToString("0.##");
            TxtPy.Text = py.ToString("0.##");

            // compute interpolated pulses for servo channels and show
            TxtServo0Interp.Text = AngleToPulse(0, servo0Deg).ToString("0");
            TxtServo2Interp.Text = AngleToPulse(2, t1deg).ToString("0");
            TxtServo3Interp.Text = AngleToPulse(3, t2deg).ToString("0");

            // If serial open, send updated servo positions
            _ = SendServoCommandAsync(0, (int)AngleToPulse(0, servo0Deg));
            _ = SendServoCommandAsync(2, (int)AngleToPulse(2, t1deg));
            _ = SendServoCommandAsync(3, (int)AngleToPulse(3, t2deg));
        }

        private static double Clamp(double v, double min, double max) => Math.Max(min, Math.Min(max, v));

        private double AngleToPulse(int channel, double angleDeg)
        {
            // channel 0 and 4 : angle range 0..180
            // channel 2 & 3     : angle range -90..90  (we map to 0..180 internally)
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

            double normalized;
            if (channel == 0 || channel == 4)
            {
                normalized = angleDeg / 180.0; // 0..1
            }
            else
            {
                normalized = (angleDeg + 90.0) / 180.0; // map -90..90 -> 0..1
            }

            normalized = Clamp(normalized, 0.0, 1.0);
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
    }

    // Kinematics solver
    public static class KinematicsSolver
    {
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
            double cosT2 = (r2 - a1 * a1 - a2 * a2) / (2 * a1 * a2);

            if (cosT2 < -1.0 - 1e-9 || cosT2 > 1.0 + 1e-9) return null;

            cosT2 = Math.Max(-1.0, Math.Min(1.0, cosT2));
            double t2a = Math.Acos(cosT2);
            double t2b = -t2a;

            double t1a = Math.Atan2(py, px) - Math.Atan2(a2 * Math.Sin(t2a), a1 + a2 * Math.Cos(t2a));
            double t1b = Math.Atan2(py, px) - Math.Atan2(a2 * Math.Sin(t2b), a1 + a2 * Math.Cos(t2b));

            return new[]
            {
                (RadiansToDegrees(t1a), RadiansToDegrees(t2a)),
                (RadiansToDegrees(t1b), RadiansToDegrees(t2b))
            };
        }

        private static double DegreesToRadians(double d) => d * Math.PI / 180.0;
        private static double RadiansToDegrees(double r) => r * 180.0 / Math.PI;
    }
}
