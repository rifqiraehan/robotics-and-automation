using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO.Ports;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace TrajectoryPlanning
{
    public partial class MainWindow : Window
    {
        private readonly DispatcherTimer _motionTimer = new DispatcherTimer();
        private int _trajectoryIndex = 0;
        private int _spaceMode = 0;
        private int _mN = 0;
        private double _mT = 0;
        private double _a1Val, _a2Val, _a3Val;

        private readonly List<double> _teta1Traj = new List<double>();
        private readonly List<double> _teta2Traj = new List<double>();
        private readonly List<double> _teta3Traj = new List<double>();
        private readonly List<double> _qxTraj = new List<double>();
        private readonly List<double> _qyTraj = new List<double>();
        private readonly List<double> _orientasiTraj = new List<double>();

        private readonly SerialPort _serialPort = new SerialPort();

        private bool _isConnected = false;

        private readonly int ch0 = 0, ch1 = 1, ch2 = 2, ch3 = 3;
        //private readonly double pwmMinCh0 = 2380, pwmMaxCh0 = 570;
        //private readonly double pwmMinCh1 = 2410, pwmMaxCh1 = 510;
        //private readonly double pwmMinCh2 = 2450, pwmMaxCh2 = 610;
        //private readonly double pwmMinCh3 = 2360, pwmMaxCh3 = 560;

        private readonly double pwmMinCh0 = 2500, pwmMaxCh0 = 500;
        private readonly double pwmMinCh1 = 2500, pwmMaxCh1 = 500;
        private readonly double pwmMinCh2 = 2500, pwmMaxCh2 = 500;
        private readonly double pwmMinCh3 = 2500, pwmMaxCh3 = 500;

        private Ellipse? m_workspaceEllipse;
        private const double GridSquareMm = 10.0;

        public MainWindow()
        {
            InitializeComponent();

            BtnCalcJoint.Click += BtnCalcJoint_Click;
            BtnRunJoint.Click += BtnRunJoint_Click;
            BtnCalcWork.Click += BtnCalcWork_Click;
            BtnRunWork.Click += BtnRunWork_Click;
            BtnClearPath.Click += BtnClearPath_Click;
            BtnExit.Click += BtnExit_Click;
            _motionTimer.Tick += MotionTimer_Tick;

            InitializeSerialPort();
            PopulateComPorts();
            BtnDisconnect.IsEnabled = false;

            BtnConnect.Click += BtnConnect_Click;
            BtnDisconnect.Click += BtnDisconnect_Click;
            BtnOriginPose.Click += BtnOriginPose_Click;

            ListJointSpace.SelectionChanged += ListJointSpace_SelectionChanged;
            ListWorkSpace.SelectionChanged += ListWorkSpace_SelectionChanged;

            CreateWorkspaceEllipse();

            m_chart.SizeChanged += (_, __) => UpdateWorkspaceCircle();
            TxtA1.TextChanged += (_, __) => UpdateWorkspaceCircle();
            TxtA2.TextChanged += (_, __) => UpdateWorkspaceCircle();
            TxtA3.TextChanged += (_, __) => UpdateWorkspaceCircle();

            UpdateWorkspaceCircle();
            ArmDraw(0, 0, 0);
        }

        private void BtnCalcJoint_Click(object sender, RoutedEventArgs e)
        {
            ListJointSpace.Items.Clear();
            _teta1Traj.Clear(); _teta2Traj.Clear(); _teta3Traj.Clear();
            if (!UpdateAndParseInputs()) return;

            double t1_0 = double.Parse(TxtTeta1_0.Text);
            double t2_0 = double.Parse(TxtTeta2_0.Text);
            double t3_0 = double.Parse(TxtTeta3_0.Text);
            double t1_1 = double.Parse(TxtTeta1_1.Text);
            double t2_1 = double.Parse(TxtTeta2_1.Text);
            double t3_1 = double.Parse(TxtTeta3_1.Text);

            for (int i = 0; i <= _mN; i++)
            {
                double t1 = t1_0 + ((t1_1 - t1_0) / _mN) * i;
                double t2 = t2_0 + ((t2_1 - t2_0) / _mN) * i;
                double t3 = t3_0 + ((t3_1 - t3_0) / _mN) * i;

                if (!ValidateJointAngles(t1, t2, t3, out string reason))
                {
                    MessageBox.Show($"Titik {i} menghasilkan sudut tidak valid: {reason}", "Batas Sudut", MessageBoxButton.OK, MessageBoxImage.Warning);
                    ListJointSpace.Items.Clear(); _teta1Traj.Clear(); _teta2Traj.Clear(); _teta3Traj.Clear();
                    return;
                }
                _teta1Traj.Add(t1); _teta2Traj.Add(t2); _teta3Traj.Add(t3);
                ListJointSpace.Items.Add($"{i,3}: {t1,6:F1} {t2,6:F1} {t3,6:F1}");
            }

            DrawPathFromJointTrajectory();
        }

        private void BtnCalcWork_Click(object sender, RoutedEventArgs e)
        {
            ListWorkSpace.Items.Clear();
            _qxTraj.Clear(); _qyTraj.Clear(); _orientasiTraj.Clear();
            if (!UpdateAndParseInputs()) return;

            double qx_0 = double.Parse(TxtQx_0.Text);
            double qy_0 = double.Parse(TxtQy_0.Text);
            double phi_0 = double.Parse(TxtOrientasi_0.Text);
            double qx_1 = double.Parse(TxtQx_1.Text);
            double qy_1 = double.Parse(TxtQy_1.Text);
            double phi_1 = double.Parse(TxtOrientasi_1.Text);

            for (int i = 0; i <= _mN; i++)
            {
                double qx = qx_0 + ((qx_1 - qx_0) / _mN) * i;
                double qy = qy_0 + ((qy_1 - qy_0) / _mN) * i;
                double phi = phi_0 + ((phi_1 - phi_0) / _mN) * i;

                if (!IsWorkspacePoseReachable(qx, qy, phi, out string reason))
                {
                    MessageBox.Show($"Target {i} ({qx:F1}, {qy:F1}) di luar jangkauan: {reason}", "Di Luar Jangkauan", MessageBoxButton.OK, MessageBoxImage.Warning);
                    ListWorkSpace.Items.Clear(); _qxTraj.Clear(); _qyTraj.Clear(); _orientasiTraj.Clear();
                    return;
                }
                var (t1, t2, t3) = ComputeInverseKinematicAngles(qx, qy, phi);
                if (!ValidateJointAngles(t1, t2, t3, out string jreason))
                {
                    MessageBox.Show($"Target {i} menghasilkan sudut tidak valid: {jreason}", "Batas Sudut", MessageBoxButton.OK, MessageBoxImage.Warning);
                    ListWorkSpace.Items.Clear(); _qxTraj.Clear(); _qyTraj.Clear(); _orientasiTraj.Clear();
                    return;
                }
                _qxTraj.Add(qx); _qyTraj.Add(qy); _orientasiTraj.Add(phi);
                ListWorkSpace.Items.Add($"{i,3}: {qx,6:F1} {qy,6:F1} {phi,6:F1}");
            }

            DrawPathFromWorkTrajectory();
        }

        private void BtnRunJoint_Click(object sender, RoutedEventArgs e)
        {
            if (!UpdateAndParseInputs() || _teta1Traj.Count == 0) { MessageBox.Show("Hitung (Calculate) lintasan terlebih dahulu."); return; }

            _trajectoryIndex = 0; _spaceMode = 0;
            int jenis = (RadioTimePath.IsChecked == true) ? 0 : 1;
            int timeInterval = (jenis == 0) ? (int)_mT : (int)(_mT / _mN);
            if (timeInterval <= 0) timeInterval = 1;
            _motionTimer.Interval = TimeSpan.FromMilliseconds(timeInterval);
            _motionTimer.Start();
        }

        private void BtnRunWork_Click(object sender, RoutedEventArgs e)
        {
            if (!UpdateAndParseInputs() || _qxTraj.Count == 0) { MessageBox.Show("Hitung (Calculate) lintasan terlebih dahulu."); return; }

            _trajectoryIndex = 0; _spaceMode = 1;
            int jenis = (RadioTimePath.IsChecked == true) ? 0 : 1;
            int timeInterval = (jenis == 0) ? (int)_mT : (int)(_mT / _mN);
            if (timeInterval <= 0) timeInterval = 1;
            _motionTimer.Interval = TimeSpan.FromMilliseconds(timeInterval);
            _motionTimer.Start();
        }

        private void BtnClearPath_Click(object sender, RoutedEventArgs e)
        {
            m_armCanvas.Children.Clear();

            for (int i = m_pathCanvas.Children.Count - 1; i >= 0; i--)
            {
                if (m_pathCanvas.Children[i] is FrameworkElement fe && fe.Tag as string == "__grid") continue;
                if (m_workspaceEllipse != null && m_pathCanvas.Children[i] == m_workspaceEllipse) continue;
                m_pathCanvas.Children.RemoveAt(i);
            }
        }

        private void BtnExit_Click(object sender, RoutedEventArgs e)
        {
            if (_isConnected) Disconnect();
            Application.Current.Shutdown();
        }

        private void MotionTimer_Tick(object sender, EventArgs e)
        {
            int intervalMs = (int)_motionTimer.Interval.TotalMilliseconds;
            if (intervalMs <= 0) intervalMs = 50;
            if (_trajectoryIndex > _mN) { _motionTimer.Stop(); return; }

            if (_spaceMode == 0)
            {
                double t1 = _teta1Traj[_trajectoryIndex];
                double t2 = _teta2Traj[_trajectoryIndex];
                double t3 = _teta3Traj[_trajectoryIndex];

                ArmDraw(t1, t2, t3);
                UpdateIndicatorsFromAngles(t1, t2, t3);
                if (_isConnected) SendServos0_2_3(t1, t2, t3, intervalMs);
            }
            else
            {
                double qx = _qxTraj[_trajectoryIndex];
                double qy = _qyTraj[_trajectoryIndex];
                double phi = _orientasiTraj[_trajectoryIndex];

                var (t1, t2, t3) = ComputeInverseKinematicAngles(qx, qy, phi);
                ArmDraw(t1, t2, t3);
                UpdateIndicators(qx, qy, t1, t2, t3, phi);
                if (_isConnected) SendServos0_2_3(t1, t2, t3, intervalMs);
            }
            _trajectoryIndex++;
        }

        private void ArmDraw(double sdt1, double sdt2, double sdt3)
        {
            m_armCanvas.Children.Clear();
            ReadLinkLengths();

            double w = m_armCanvas.ActualWidth;
            double h = m_armCanvas.ActualHeight;
            if (w == 0 || h == 0) { w = m_chart.ActualWidth; h = m_chart.ActualHeight; }

            double t1 = KinematicsSolver.DegreesToRadians(sdt1);
            double t2 = KinematicsSolver.DegreesToRadians(sdt2);
            double t3 = KinematicsSolver.DegreesToRadians(sdt3);

            double x0 = 0, y0 = 0;
            double x1 = x0 + _a1Val * Math.Cos(t1);
            double y1 = y0 + _a1Val * Math.Sin(t1);
            double x2 = x1 + _a2Val * Math.Cos(t1 + t2);
            double y2 = y1 + _a2Val * Math.Sin(t1 + t2);
            double x3 = x2 + _a3Val * Math.Cos(t1 + t2 + t3);
            double y3 = y2 + _a3Val * Math.Sin(t1 + t2 + t3);

            var p0 = ToCanvas(x0, y0, w, h);
            var p1 = ToCanvas(x1, y1, w, h);
            var p2 = ToCanvas(x2, y2, w, h);
            var p3 = ToCanvas(x3, y3, w, h);

            var link1 = new Line { X1 = p0.x, Y1 = p0.y, X2 = p1.x, Y2 = p1.y, Stroke = Brushes.DarkRed, StrokeThickness = 6 };
            var link2 = new Line { X1 = p1.x, Y1 = p1.y, X2 = p2.x, Y2 = p2.y, Stroke = Brushes.DarkOrange, StrokeThickness = 6 };
            var link3 = new Line { X1 = p2.x, Y1 = p2.y, X2 = p3.x, Y2 = p3.y, Stroke = Brushes.DodgerBlue, StrokeThickness = 5 };

            m_armCanvas.Children.Add(link1);
            m_armCanvas.Children.Add(link2);
            m_armCanvas.Children.Add(link3);

            AddJointEllipse(p0.x, p0.y, 10, Brushes.Black);
            AddJointEllipse(p1.x, p1.y, 10, Brushes.LightGray);
            AddJointEllipse(p2.x, p2.y, 10, Brushes.LightGray);
            AddJointEllipse(p3.x, p3.y, 8, Brushes.Green);
        }

        private void DrawPathFromJointTrajectory()
        {
            BtnClearPath_Click(null, null);
            ReadLinkLengths();
            double w = m_pathCanvas.ActualWidth;
            double h = m_pathCanvas.ActualHeight;
            if (w == 0 || h == 0) { w = m_chart.ActualWidth; h = m_chart.ActualHeight; }

            var poly = new Polyline { Stroke = Brushes.Blue, StrokeThickness = 1.5 };

            for (int i = 0; i < _teta1Traj.Count; i++)
            {
                var (ex, ey) = KinematicsSolver.Forward(_a1Val, _a2Val, _a3Val, _teta1Traj[i], _teta2Traj[i], _teta3Traj[i]);
                var p = ToCanvas(ex, ey, w, h);
                poly.Points.Add(new Point(p.x, p.y));
            }
            m_pathCanvas.Children.Add(poly);
        }

        private void DrawPathFromWorkTrajectory()
        {
            BtnClearPath_Click(null, null);
            ReadLinkLengths();
            double w = m_pathCanvas.ActualWidth;
            double h = m_pathCanvas.ActualHeight;
            if (w == 0 || h == 0) { w = m_chart.ActualWidth; h = m_chart.ActualHeight; }

            var poly = new Polyline { Stroke = Brushes.DarkGreen, StrokeThickness = 1.5 };

            for (int i = 0; i < _qxTraj.Count; i++)
            {
                var p = ToCanvas(_qxTraj[i], _qyTraj[i], w, h);
                poly.Points.Add(new Point(p.x, p.y));
            }
            m_pathCanvas.Children.Add(poly);
        }

        private void ReadLinkLengths()
        {
            if (!double.TryParse(TxtA1.Text, out _a1Val) || _a1Val <= 0) _a1Val = 120.0;
            if (!double.TryParse(TxtA2.Text, out _a2Val) || _a2Val <= 0) _a2Val = 130.0;
            if (!double.TryParse(TxtA3.Text, out _a3Val) || _a3Val <= 0) _a3Val = 80.0;
        }

        private void AddJointEllipse(double cx, double cy, double r, Brush fill)
        {
            var el = new Ellipse { Width = r, Height = r, Fill = fill, Stroke = Brushes.Black, StrokeThickness = 1 };
            Canvas.SetLeft(el, cx - r / 2.0);
            Canvas.SetTop(el, cy - r / 2.0);
            m_armCanvas.Children.Add(el);
        }

        private (double x, double y) ToCanvas(double ux, double uy, double canvasWidth, double canvasHeight)
        {
            ReadLinkLengths();
            double totalLength = Math.Max(1.0, _a1Val + _a2Val + _a3Val);
            double marginFactor = 0.9;
            double minDim = Math.Min(canvasWidth, canvasHeight);
            double scale = (minDim * marginFactor) / (2.0 * totalLength);
            double cx = canvasWidth / 2.0 + ux * scale;
            double cy = canvasHeight / 2.0 - uy * scale;
            return (cx, cy);
        }

        private void CreateWorkspaceEllipse()
        {
            if (m_workspaceEllipse != null) return;
            m_workspaceEllipse = new Ellipse
            {
                Stroke = Brushes.Red,
                StrokeThickness = 1.8,
                Fill = Brushes.Transparent,
                IsHitTestVisible = false,
                Tag = "__workspace"
            };
            m_pathCanvas.Children.Insert(0, m_workspaceEllipse);
        }

        private void UpdateWorkspaceCircle()
        {
            if (m_pathCanvas == null || m_chart == null) return;
            double w = m_chart.ActualWidth;
            double h = m_chart.ActualHeight;
            if (w <= 0 || h <= 0) return;

            ReadLinkLengths();
            double totalReachMm = Math.Max(1.0, _a1Val + _a2Val + _a3Val);
            double marginFactor = 0.9;
            double minDim = Math.Min(w, h);
            double pixelsPerMm = (minDim * marginFactor) / (2.0 * totalReachMm);

            double indicatorRadiusMm = _a1Val + _a2Val + _a3Val;
            double radiusPx = indicatorRadiusMm * pixelsPerMm;

            var center = ToCanvas(0, 0, w, h);

            if (m_workspaceEllipse != null)
            {
                m_workspaceEllipse.Width = radiusPx * 2.0;
                m_workspaceEllipse.Height = radiusPx * 2.0;
                Canvas.SetLeft(m_workspaceEllipse, center.x - radiusPx);
                Canvas.SetTop(m_workspaceEllipse, center.y - radiusPx);
            }
            DrawGrid(w, h);
        }

        private void DrawGrid(double w, double h)
        {
            if (w <= 0 || h <= 0) return;
            ReadLinkLengths();

            double totalReachMm = Math.Max(1.0, _a1Val + _a2Val + _a3Val);
            double marginFactor = 0.9;
            double minDim = Math.Min(w, h);
            double pixelsPerMm = (minDim * marginFactor) / (2.0 * totalReachMm);
            if (pixelsPerMm <= 0) return;

            double gridPx = GridSquareMm * pixelsPerMm;

            for (int i = m_pathCanvas.Children.Count - 1; i >= 0; i--)
            {
                if (m_pathCanvas.Children[i] is FrameworkElement fe && fe.Tag as string == "__grid")
                    m_pathCanvas.Children.RemoveAt(i);
            }

            var center = ToCanvas(0, 0, w, h);
            var gridBrush = new SolidColorBrush(Color.FromArgb(0x22, 0, 0, 0));

            int startN = (int)Math.Floor(-center.x / gridPx) - 2;
            int endN = (int)Math.Ceiling((w - center.x) / gridPx) + 2;
            for (int n = startN; n <= endN; n++)
            {
                double x = center.x + n * gridPx;
                var line = new Line { X1 = x, X2 = x, Y1 = 0, Y2 = h, Stroke = gridBrush, StrokeThickness = 1, Tag = "__grid" };
                m_pathCanvas.Children.Insert(0, line);
            }

            int startM = (int)Math.Floor(-center.y / gridPx) - 2;
            int endM = (int)Math.Ceiling((h - center.y) / gridPx) + 2;
            for (int m = startM; m <= endM; m++)
            {
                double y = center.y + m * gridPx;
                var line = new Line { X1 = 0, X2 = w, Y1 = y, Y2 = y, Stroke = gridBrush, StrokeThickness = 1, Tag = "__grid" };
                m_pathCanvas.Children.Insert(0, line);
            }
        }

        private bool UpdateAndParseInputs()
        {
            try
            {
                ReadLinkLengths();
                _mN = int.Parse(TxtN.Text);
                _mT = double.Parse(TxtT.Text);
                if (_mN <= 0) _mN = 1;
                return true;
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Input error: {ex.Message}");
                _motionTimer.Stop();
                return false;
            }
        }

        private void PopulateComPorts()
        {
            try
            {
                CmbPorts.ItemsSource = SerialPort.GetPortNames();
                if (CmbPorts.Items.Count > 0) CmbPorts.SelectedIndex = 0;
            }
            catch { }
        }

        private void InitializeSerialPort()
        {
            _serialPort.BaudRate = 115200; _serialPort.Parity = Parity.None;
            _serialPort.DataBits = 8; _serialPort.StopBits = StopBits.One;
            _serialPort.Handshake = Handshake.None; _serialPort.Encoding = Encoding.ASCII;
            _serialPort.WriteTimeout = 500;
        }

        public bool Connect(string portName)
        {
            try
            {
                if (_serialPort.IsOpen) _serialPort.Close();
                _serialPort.PortName = portName; _serialPort.Open();
                _isConnected = true; return true;
            }
            catch { _isConnected = false; return false; }
        }

        public void Disconnect()
        {
            try { if (_serialPort.IsOpen) _serialPort.Close(); } catch { }
            _isConnected = false;
        }

        private void SendRawCommand(string cmd)
        {
            if (!_isConnected) return;
            try
            {
                if (!cmd.EndsWith("\r") && !cmd.EndsWith("\n")) cmd += "\r\n";
                _serialPort.Write(cmd);
            }
            catch
            {
                try { _serialPort.Close(); } catch { }
                _isConnected = false;
                BtnConnect.IsEnabled = true; BtnDisconnect.IsEnabled = false;
            }
        }

        private int MapAngleToPulse(double angleDeg, double angleMin, double angleMax, double pwmMin, double pwmMax)
        {
            if (angleDeg < Math.Min(angleMin, angleMax)) angleDeg = Math.Min(angleMin, angleMax);
            if (angleDeg > Math.Max(angleMin, angleMax)) angleDeg = Math.Max(angleMin, angleMax);
            double t = (angleDeg - angleMin) / (angleMax - angleMin);
            double pw = pwmMin + t * (pwmMax - pwmMin);
            return (int)Math.Round(pw);
        }

        private void SendServos0_2_3(double deg0, double deg2, double deg3, int timeMs)
        {
            if (!_isConnected) return;
            int p0 = MapAngleToPulse(deg0, 0.0, 180.0, pwmMinCh0, pwmMaxCh0);
            int p2 = MapAngleToPulse(deg2, -90.0, 90.0, pwmMinCh2, pwmMaxCh2);
            int p3 = MapAngleToPulse(deg3, -90.0, 90.0, pwmMinCh3, pwmMaxCh3);
            SendRawCommand($"#{ch0} P{p0} T{timeMs}");
            SendRawCommand($"#{ch2} P{p2} T{timeMs}");
            SendRawCommand($"#{ch3} P{p3} T{timeMs}");
        }

        private void BtnConnect_Click(object sender, RoutedEventArgs e)
        {
            if (_isConnected) return;
            if (CmbPorts.SelectedItem == null) { MessageBox.Show("Pilih COM port dulu."); return; }
            var port = CmbPorts.SelectedItem.ToString();
            if (Connect(port!))
            {
                BtnConnect.IsEnabled = false; BtnDisconnect.IsEnabled = true;
                SendRawCommand($"#{ch0} P1544 T1000"); SendRawCommand($"#{ch1} P2220 T1000");
                SendRawCommand($"#{ch2} P1467 T1000"); SendRawCommand($"#{ch3} P1456 T1000");
            }
            else { MessageBox.Show("Gagal membuka port " + port); }
        }

        private void BtnDisconnect_Click(object sender, RoutedEventArgs e)
        {
            if (!_isConnected) return;
            Disconnect();
            BtnConnect.IsEnabled = true; BtnDisconnect.IsEnabled = false;
        }

        private void BtnOriginPose_Click(object sender, RoutedEventArgs e)
        {
            if (!_isConnected) { MessageBox.Show("Port belum terbuka. Silakan Connect terlebih dahulu."); return; }
            SendRawCommand($"#{ch0} P1544 T1000"); SendRawCommand($"#{ch1} P2220 T1000");
            SendRawCommand($"#{ch2} P1467 T1000"); SendRawCommand($"#{ch3} P1456 T1000");
        }

        private void ListJointSpace_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            int idx = ListJointSpace.SelectedIndex;
            if (idx < 0 || _teta1Traj.Count <= idx) return;
            if (_motionTimer.IsEnabled) _motionTimer.Stop();
            double th1 = _teta1Traj[idx], th2 = _teta2Traj[idx], th3 = _teta3Traj[idx];
            if (!ValidateJointAngles(th1, th2, th3, out _)) return;
            ArmDraw(th1, th2, th3);
            UpdateIndicatorsFromAngles(th1, th2, th3);
            if (_isConnected) SendServos0_2_3(th1, th2, th3, 1000);
        }

        private void ListWorkSpace_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            int idx = ListWorkSpace.SelectedIndex;
            if (idx < 0 || _qxTraj.Count <= idx) return;
            if (_motionTimer.IsEnabled) _motionTimer.Stop();
            double qx = _qxTraj[idx], qy = _qyTraj[idx], phi = _orientasiTraj[idx];
            var (deg1, deg2, deg3) = ComputeInverseKinematicAngles(qx, qy, phi);
            if (!ValidateJointAngles(deg1, deg2, deg3, out _)) return;
            ArmDraw(deg1, deg2, deg3);
            UpdateIndicators(qx, qy, deg1, deg2, deg3, phi);
            if (_isConnected) SendServos0_2_3(deg1, deg2, deg3, 1000);
        }

        private bool IsWorkspacePoseReachable(double iqx, double iqy, double phiDeg, out string reason)
        {
            ReadLinkLengths();
            double phi = KinematicsSolver.DegreesToRadians(phiDeg);
            double wx = iqx - _a3Val * Math.Cos(phi);
            double wy = iqy - _a3Val * Math.Sin(phi);
            double d = Math.Sqrt(wx * wx + wy * wy);
            double min = Math.Abs(_a1Val - _a2Val);
            double max = _a1Val + _a2Val;
            if (d > max + 1e-6) { reason = $"Jarak wrist {d:0.0}mm > maks {max:0.0}mm"; return false; }
            if (d < min - 1e-6) { reason = $"Jarak wrist {d:0.0}mm < min {min:0.0}mm"; return false; }
            reason = string.Empty; return true;
        }

        private bool ValidateJointAngles(double deg1, double deg2, double deg3, out string reason)
        {
            if (double.IsNaN(deg1) || double.IsNaN(deg2) || double.IsNaN(deg3)) { reason = "Hasil IK tidak valid (NaN)."; return false; }
            if (deg1 < 0.0 - 1e-6 || deg1 > 180.0 + 1e-6) { reason = $"teta1 = {deg1:0.0}° di luar [0°, 180°]."; return false; }
            if (deg2 < -90.0 - 1e-6 || deg2 > 90.0 + 1e-6) { reason = $"teta2 = {deg2:0.0}° di luar [-90°, 90°]."; return false; }
            if (deg3 < -90.0 - 1e-6 || deg3 > 90.0 + 1e-6) { reason = $"teta3 = {deg3:0.0}° di luar [-90°, 90°]."; return false; }
            reason = string.Empty; return true;
        }

        private (double deg1, double deg2, double deg3) ComputeInverseKinematicAngles(double iqx, double iqy, double iorientasi)
        {
            var sols = KinematicsSolver.Inverse(_a1Val, _a2Val, _a3Val, iqx, iqy, iorientasi);
            if (sols != null) { return sols[0]; }
            return (double.NaN, double.NaN, double.NaN);
        }

        private void UpdateIndicators(double qxVal, double qyVal, double t1Deg, double t2Deg, double t3Deg, double phiDeg)
        {
            IndicatorQx.Text = qxVal.ToString("0.0", CultureInfo.InvariantCulture);
            IndicatorQy.Text = qyVal.ToString("0.0", CultureInfo.InvariantCulture);
            IndicatorPhi.Text = phiDeg.ToString("0.0", CultureInfo.InvariantCulture);
            IndicatorT1.Text = t1Deg.ToString("0.0", CultureInfo.InvariantCulture);
            IndicatorT2.Text = t2Deg.ToString("0.0", CultureInfo.InvariantCulture);
            IndicatorT3.Text = t3Deg.ToString("0.0", CultureInfo.InvariantCulture);
        }

        private void UpdateIndicatorsFromAngles(double t1Deg, double t2Deg, double t3Deg)
        {
            var (qxVal, qyVal) = KinematicsSolver.Forward(_a1Val, _a2Val, _a3Val, t1Deg, t2Deg, t3Deg);
            double phiDeg = t1Deg + t2Deg + t3Deg;
            UpdateIndicators(qxVal, qyVal, t1Deg, t2Deg, t3Deg, phiDeg);
        }
    }

    public static class KinematicsSolver
    {
        public static double DegreesToRadians(double d) => d * Math.PI / 180.0;
        public static double RadiansToDegrees(double r) => r * 180.0 / Math.PI;

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