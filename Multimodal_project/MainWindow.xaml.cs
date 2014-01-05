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
using System.Windows.Interop;
using Microsoft.Kinect;
using Microsoft.Kinect.Toolkit;
using mshtml;
using Coding4Fun.Kinect.Wpf;
using System.Runtime.InteropServices;

namespace Multimodal_project
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Notes...
        /// 
        /// HitTest to detect browser ui elements at a given position of the browser
        /// http://msdn.microsoft.com/en-us/library/ms608753%28v=VS.90%29.aspx
        /// 
        /// 
        /// Summary:
        /// 
        /// This class is almost only built using the tutorials shown in.
        /// http://www.microsoft.com/en-us/kinectforwindowsdev/start.aspx
        /// With focus on the setting up development environment and skeletal tracking videos
        /// </summary>

        ///


        public MainWindow()
        {
            SetCursorPos(0, 0);
            InitializeComponent();
            browser.Navigate("http://www.w3schools.com/");
            //            browser.Navigate(System.AppDomain.CurrentDomain.BaseDirectory + "web/start_page.html");

            max_iterations = new Dictionary<GestureState, int>();
            max_iterations[GestureState.ZOOM] = 8;
            max_iterations[GestureState.SCROLL] = 32; // unused
            max_iterations[GestureState.CLICK] = 4;

            InitializeHandFigures();
        }

        #region Hand visualization functionality.

        /* Fields */
        Rectangle LeftRect, RightRect;
        Ellipse LeftEllipse, RightEllipse;
        Polygon LeftStar, RightStar;
        bool rightActive = false, leftActive = false;

        /* Methods */
        private void InitializeHandFigures()
        {
            LeftRect = new Rectangle();
            LeftRect.Width = 50;
            LeftRect.Height = 50;
            LeftRect.Fill = System.Windows.Media.Brushes.Green;
            LeftRect.Stroke = System.Windows.Media.Brushes.HotPink;
            LeftRect.StrokeThickness = 10;
            LeftRect.Opacity = 0.5;

            RightRect = new Rectangle();
            RightRect.Width = 50;
            RightRect.Height = 50;
            RightRect.Fill = System.Windows.Media.Brushes.Blue;
            RightRect.Stroke = System.Windows.Media.Brushes.HotPink;
            RightRect.StrokeThickness = 10;
            RightRect.Opacity = 0.5;

            LeftEllipse = new Ellipse();
            LeftEllipse.Width = 50;
            LeftEllipse.Height = 50;
            LeftEllipse.Fill = System.Windows.Media.Brushes.Green;
            LeftEllipse.Stroke = System.Windows.Media.Brushes.HotPink;
            LeftEllipse.StrokeThickness = 10;
            LeftEllipse.Opacity = 0.5;

            RightEllipse = new Ellipse();
            RightEllipse.Width = 50;
            RightEllipse.Height = 50;
            RightEllipse.Fill = System.Windows.Media.Brushes.Green;
            RightEllipse.Stroke = System.Windows.Media.Brushes.HotPink;
            RightEllipse.StrokeThickness = 10;
            RightEllipse.Opacity = 0.5;


            PointCollection points = new PointCollection();
            double arms = 5;
            double angle = Math.PI / arms;
            double rOuter = 20;
            double rInner = 10;
            for (int i = 0; i < 2 * arms; i++)
            {
                double r = (i & 1) == 0 ? rOuter : rInner;
                points.Add(new Point(25 + Math.Cos(i * angle) * r, 25 + Math.Sin(i * angle) * r));
            }
            points.Add(new Point(25 + Math.Cos(0) * rOuter, 25 + Math.Sin(0) * rOuter));

            LeftStar = new Polygon();
            LeftStar.Fill = System.Windows.Media.Brushes.Green;
            LeftStar.Stroke = System.Windows.Media.Brushes.HotPink;
            LeftStar.StrokeThickness = 5;
            LeftStar.Opacity = 1;
            LeftStar.Points = points;

            RightStar = new Polygon();
            RightStar.Fill = System.Windows.Media.Brushes.Green;
            RightStar.Stroke = System.Windows.Media.Brushes.HotPink;
            RightStar.StrokeThickness = 5;
            RightStar.Opacity = 1;
            RightStar.Points = points;

            // Code used to change shape of an object
            // Shape standards are Rectangle, Ellipse, Line, Path, Polygon, Polyline Preferably 
            // Should probably implement separate functionality to change color of hands
            // so that all Shape objects are kept up to date and can be changed
            // without having to manage current color value
            LeftPopup.Child = LeftEllipse;
            RightPopup.Child = RightEllipse;
        }
        void setLeftColor(System.Windows.Media.Brush new_color)
        {
            LeftEllipse.Stroke = new_color;
            LeftRect.Stroke = new_color;
            LeftStar.Stroke = new_color;
        }
        void setRightColor(System.Windows.Media.Brush new_color)
        {
            RightEllipse.Stroke = new_color;
            RightRect.Stroke = new_color;
            RightStar.Stroke = new_color;
        }
        void setLeftShape(PointerShape shape)
        {
            switch (shape)
            {
                case PointerShape.ELLIPSE:
                    LeftPopup.Child = LeftEllipse;
                    break;
                case PointerShape.RECTANGLE:
                    LeftPopup.Child = LeftRect;
                    break;
                case PointerShape.STAR:
                    LeftPopup.Child = LeftStar;
                    break;
            }
        }
        void setRightShape(PointerShape shape)
        {
            switch (shape)
            {
                case PointerShape.ELLIPSE:
                    RightPopup.Child = RightEllipse;
                    break;
                case PointerShape.RECTANGLE:
                    RightPopup.Child = RightRect;
                    break;
                case PointerShape.STAR:
                    RightPopup.Child = RightStar;
                    break;
            }
        }

        /* Enums */
        enum PointerShape { RECTANGLE, ELLIPSE, STAR };
        #endregion

        #region Kinect stuff.
        /* Fields */
        bool closing = false;
        KinectSensor _sensor;
        const int skeletonCount = 6;
        Skeleton[] allSkeletons = new Skeleton[skeletonCount];

        /* Methods */
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            var parameters = new TransformSmoothParameters
            {
                Smoothing = 0.05f,
                Correction = 0.4f,
                Prediction = 0.0f,
                JitterRadius = 1.0f,
                MaxDeviationRadius = 0.5f
            };
            if (KinectSensor.KinectSensors.Count > 0)
            {
                _sensor = KinectSensor.KinectSensors[0];
                if (_sensor.Status == KinectStatus.Connected)
                {

                    _sensor.ColorStream.Enable();
                    _sensor.DepthStream.Enable();
                    _sensor.SkeletonStream.Enable(parameters);
                    _sensor.AllFramesReady += _sensor_AllFramesReady;
                    try
                    {
                        _sensor.Start();
                    }
                    catch (System.IO.IOException)
                    {

                    }
                }
            }

        }

        private void StopKinect(KinectSensor sensor)
        {
            if (sensor != null && sensor.IsRunning)
            {
                sensor.Stop();
            }
        }

        void _sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            if (closing)
                return;

            Skeleton first = getFirstSkeleton(e);

            if (first == null)
                return;

            UpdateHandPositions(first, e);
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            closing = true;
            StopKinect(_sensor);
        }

        Skeleton getFirstSkeleton(AllFramesReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrameData = e.OpenSkeletonFrame())
            {
                if (skeletonFrameData == null)
                {
                    return null;
                }
                skeletonFrameData.CopySkeletonDataTo(allSkeletons);
                Skeleton first = allSkeletons.Where(s => s.TrackingState == SkeletonTrackingState.Tracked).FirstOrDefault();
                return first;
            }
        }

        private void UpdateHandPositions(Skeleton skeleton, AllFramesReadyEventArgs e)
        {
            using (DepthImageFrame depth = e.OpenDepthImageFrame())
            {
                if (depth == null || _sensor == null)
                    return;

                double Width = MainWpfWindow.Width;
                double Height = MainWpfWindow.Height;

                //                Using the Coding4Fun assembly to calculate hand positions
                Joint left_hand = skeleton.Joints[JointType.HandLeft];
                Joint right_hand = skeleton.Joints[JointType.HandRight];

                Joint left_hand_scaled = left_hand.ScaleTo((int)Width, (int)Height * 2, 0.5f, 0.5f);
                Joint right_hand_scaled = right_hand.ScaleTo((int)Width, (int)Height * 2, 0.5f, 0.5f);

                if (current_state != GestureState.CLICK)
                {
                    RightPopup.HorizontalOffset = right_hand_scaled.Position.X;
                    RightPopup.VerticalOffset = right_hand_scaled.Position.Y;
                }

                LeftPopup.HorizontalOffset = left_hand_scaled.Position.X;
                LeftPopup.VerticalOffset = left_hand_scaled.Position.Y;

                /*
                 * If the hand positions are 0.25 m closer than the head position
                 */
                double threshold = 0.25;
                if (skeleton.Joints[JointType.Head].Position.Z > skeleton.Joints[JointType.HandLeft].Position.Z + threshold)
                {
                    if (!leftActive)
                        setLeftColor(System.Windows.Media.Brushes.HotPink);
                    leftActive = true;
                }
                else
                {
                    if (leftActive)
                        setLeftColor(System.Windows.Media.Brushes.White);
                    leftActive = false;
                }
                if (skeleton.Joints[JointType.Head].Position.Z > skeleton.Joints[JointType.HandRight].Position.Z + threshold)
                {
                    if (!rightActive)
                        setRightColor(System.Windows.Media.Brushes.HotPink);
                    rightActive = true;
                }
                else
                {
                    if (rightActive)
                        setRightColor(System.Windows.Media.Brushes.White);
                    rightActive = false;
                }

                // Code for gestures / interaction

                gestureStep(left_hand_scaled, right_hand_scaled, skeleton);

            }
        }
        #endregion

        #region Gesture analysis functionality.
        /* Fields */
        Dictionary<GestureState, int> max_iterations;
        LinkedList<Joint> prev_left = new LinkedList<Joint>();
        LinkedList<Joint> prev_right = new LinkedList<Joint>();
        GestureState current_state;
        enum GestureState { ZOOM, SCROLL, CLICK, UNKNOWN };

        /* Methods */
        private void gestureStep(Joint left_hand_scaled, Joint right_hand_scaled, Skeleton skeleton)
        {
            prev_left.AddFirst(left_hand_scaled);
            if (prev_left.Count > 32)
            {
                prev_left.RemoveLast();
            }
            prev_right.AddFirst(right_hand_scaled);
            if (prev_right.Count > 32)
            {
                prev_right.RemoveLast();
            }

            if (!leftActive && !rightActive)
            {
                prev_left.Clear();
                prev_right.Clear();
                SetState(GestureState.UNKNOWN);
                return;
            }

            if (leftActive && !rightActive)
            {
                prev_right.Clear();
                SetState(GestureState.UNKNOWN);
            }
            else if (rightActive && !leftActive)
            {
                prev_left.Clear();
                if (!can_click)
                    return;

                if (current_state == GestureState.CLICK)
                {
                    if ((right_hand_scaled.Position.Z) < click_start_pos.Z - 0.1)
                    {
                        int X = int.Parse(Math.Floor(click_start_pos.X).ToString()) + 25;
                        int Y = int.Parse(Math.Floor(click_start_pos.Y).ToString()) + 25;
                        SetCursorPos(X, Y);
                        mouse_event(MOUSEEVENTF_LEFTDOWN | MOUSEEVENTF_LEFTUP, (uint)X, (uint)Y, 0, 0);
                        SetCursorPos(0, 0);
                        SetState(GestureState.UNKNOWN);
                        prev_right.Clear();
                        BlockClicks();
                        setRightColor(System.Windows.Media.Brushes.HotPink);
                        Console.WriteLine("Triggered click.");
                    }
                    else if ((right_hand_scaled.Position.Z) > click_start_pos.Z + 0.1)
                    {
                        SetState(GestureState.UNKNOWN);
                        Console.WriteLine("Cancelled click");
                    }
                    return;
                }

                Joint? tmp = null;
                int iter = 0;
                double dx = 0;
                double dy = 0;
                double dz = 0;
                foreach (Joint j in prev_right)
                {
                    if (tmp != null)
                    {
                        if (iter <= max_iterations[GestureState.CLICK])
                        {
                            dx += j.Position.X - tmp.Value.Position.X;
                            dy += j.Position.Y - tmp.Value.Position.Y;
                            dz += j.Position.Z - tmp.Value.Position.Z;
                        }
                    }
                    tmp = j;
                }
                if (dz > 0.05 &&
                    Math.Abs(dx) < 100 &&
                    Math.Abs(dy) < 100)
                {
                    setRightColor(System.Windows.Media.Brushes.Red);
                    SetState(GestureState.CLICK);
                    click_start_pos = right_hand_scaled.Position;
                }
            }
            else if (rightActive && leftActive)
            {
                Joint? tmp = null;
                float left_dy = 0, left_dx = 0, right_dy = 0, right_dx = 0;
                int iter = 0;
                foreach (Joint j in prev_left)
                {
                    iter++;
                    if (tmp != null)
                    {
                        if (iter <= max_iterations[GestureState.ZOOM])
                        {
                            left_dy += j.Position.Y - tmp.Value.Position.Y;
                            left_dx += j.Position.X - tmp.Value.Position.X;
                        }
                    }
                    tmp = j;
                }
                tmp = null;
                iter = 0;
                foreach (Joint j in prev_right)
                {
                    iter++;
                    if (tmp != null)
                    {
                        if (iter <= max_iterations[GestureState.ZOOM])
                        {
                            right_dy += j.Position.Y - tmp.Value.Position.Y;
                            right_dx += j.Position.X - tmp.Value.Position.X;
                        }
                    }
                    tmp = j;
                }
                double left_dist = Math.Sqrt(Math.Pow(left_dy, 2) + Math.Pow(left_dx, 2));
                double right_dist = Math.Sqrt(Math.Pow(right_dy, 2) + Math.Pow(right_dx, 2));
                double dist = Math.Sqrt(Math.Pow(left_dy - right_dy, 2) + Math.Pow(left_dx - right_dx, 2));



                if (current_state == GestureState.UNKNOWN)
                {
                    if (
                        dist > 50 &&
                        dist * 1.05 > (left_dist + right_dist) &&
                        left_dist / right_dist > 0.8 &&
                        left_dist / right_dist < 1.2)
                    {
                        SetState(GestureState.ZOOM);
                    }
                    else if ((
                            Math.Abs(left_dy) > 50 &&
                            left_dy / right_dy > 0.8 &&
                            left_dy / right_dy < 1.2
                           ) || (
                            Math.Abs(left_dx) > 50 &&
                            left_dx / right_dx > 0.8 &&
                            left_dx / right_dx < 1.2
                            ))
                    {
                        SetState(GestureState.SCROLL);
                    }
                    else
                    {
                        SetState(GestureState.UNKNOWN);
                    }
                }

                if (current_state == GestureState.ZOOM)
                {
                    if (prev_left.Count > 1 && prev_right.Count > 1)
                    {
                        double current_dist = Math.Sqrt(Math.Pow(left_hand_scaled.Position.X - right_hand_scaled.Position.X, 2) + Math.Pow(left_hand_scaled.Position.Y - right_hand_scaled.Position.Y, 2));
                        double prev_dist = Math.Sqrt(Math.Pow(prev_left.ElementAt(1).Position.X - prev_right.ElementAt(1).Position.X, 2) + Math.Pow(prev_left.ElementAt(1).Position.Y - prev_right.ElementAt(1).Position.Y, 2));
                        double zoom = current_dist - prev_dist;
                        SetZoomRel(zoom / 2000);
                        // call zoom function
                    }
                }
                else if (current_state == GestureState.SCROLL)
                {
                    if (prev_left.Count > 1 && prev_right.Count > 1)
                    {
                        double current_x = (left_hand_scaled.Position.X + right_hand_scaled.Position.X) / 2;
                        double prev_x = (prev_left.ElementAt(1).Position.X + prev_right.ElementAt(1).Position.X) / 2;
                        double current_y = (left_hand_scaled.Position.Y + right_hand_scaled.Position.Y) / 2;
                        double prev_y = (prev_left.ElementAt(1).Position.Y + prev_right.ElementAt(1).Position.Y) / 2;
                        double dx = current_x - prev_x;
                        double dy = current_y - prev_y;
                        const double H_SCROLL = 1.7;
                        const double V_SCROLL = 1.5;
                        SetScrollRel(-dx * H_SCROLL, -dy * V_SCROLL);
                    }
                }
            }
        }

        void SetState(GestureState new_state)
        {
            if (current_state == new_state)
                return;
            Console.WriteLine("Changing state from: " + current_state + " to: " + new_state);
            current_state = new_state;
            switch (new_state)
            {
                case GestureState.ZOOM:
                    setLeftShape(PointerShape.STAR);
                    setRightShape(PointerShape.STAR);
                    break;
                case GestureState.SCROLL:
                    setLeftShape(PointerShape.RECTANGLE);
                    setRightShape(PointerShape.RECTANGLE);
                    break;
                case GestureState.CLICK:
                    break;
                case GestureState.UNKNOWN:
                    setLeftShape(PointerShape.ELLIPSE);
                    setRightShape(PointerShape.ELLIPSE);
                    break;
            }



        }
        #endregion

        #region Browser manipulation.
        /// <summary>
        /// Browser zoom management.
        /// </summary>
        const double MAX_ZOOM = 5.0;
        const double MIN_ZOOM = 0.1;
        double zoom_current = 1.0;
        void SetZoomRel(double zoom_change)
        {

            zoom_current += zoom_change;
            zoom_current = Math.Max(MIN_ZOOM, Math.Min(zoom_current, MAX_ZOOM));
            mshtml.IHTMLDocument2 doc = browser.Document as mshtml.IHTMLDocument2;
            doc.parentWindow.execScript("if(document.body != null) document.body.style.zoom = " + zoom_current.ToString().Replace(",", ".") + ";");
        }

        /// <summary>
        /// Browser 
        /// </summary>
        /// <param name="horizontal_scroll"></param>
        /// <param name="vertical_scroll"></param>
        void SetScrollRel(double horizontal_scroll, double vertical_scroll)
        {
            mshtml.IHTMLDocument2 doc = browser.Document as mshtml.IHTMLDocument2;
            doc.parentWindow.scrollBy((int)Math.Floor(horizontal_scroll), (int)Math.Floor(vertical_scroll));
            //doc.parentWindow.execScript("if(window != null && document.body != null) window.scrollTo( document.body.scrollLeft + "+ horizontal_scroll.ToString().Replace(",", ".") + ", document.body.scrollTop + "+ vertical_scroll.ToString().Replace(",", ".") + ") ;");
        }

        #endregion

        #region Cursor management.
        /* Fields */
        bool can_click = true;
        SkeletonPoint click_start_pos;

        /* Methods */
        [DllImport("user32.dll", CharSet = CharSet.Auto, CallingConvention = CallingConvention.StdCall)]
        public static extern void mouse_event(uint dwFlags, uint dx, uint dy, uint cButtons, uint dwExtraInfo);
        [DllImport("user32.dll", EntryPoint = "SetCursorPos")]
        private static extern bool SetCursorPos(int X, int Y);

        async void BlockClicks()
        {
            can_click = false;
            await Task.Delay(CLICK_COOLDOWN_MS);
            can_click = true;
        }

        /* Constants */
        private const int MOUSEEVENTF_LEFTDOWN = 0x02;
        private const int MOUSEEVENTF_LEFTUP = 0x04;
        private const int MOUSEEVENTF_RIGHTDOWN = 0x08;
        private const int MOUSEEVENTF_RIGHTUP = 0x10;
        private const int CLICK_COOLDOWN_MS = 2000;

        
        #endregion
    }
}
