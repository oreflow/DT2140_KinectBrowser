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
using Coding4Fun.Kinect.Wpf;

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


        Dictionary<GestureState, int> max_iterations;
        public MainWindow()
        {
            InitializeComponent();
            browser.Navigate("http://www.google.se/");

            max_iterations = new Dictionary<GestureState, int>();
            max_iterations[GestureState.ZOOM] = 8;
            max_iterations[GestureState.SCROLL] = 32; // unused
            max_iterations[GestureState.CLICK] = 4;

            InitializeHandFigures();
            /*
                testing
                int Width = 1920;
                int Height = 1080;
                RightEllipse.HorizontalOffset = ((0-320) / 640.0) * Width + Width / 2;
                RightEllipse.VerticalOffset = ((0-240) / 480.0) * Height + Height / 2;

                LeftEllipse.HorizontalOffset = ((320-320) / 640.0) * Width + Width / 2;
                LeftEllipse.VerticalOffset = ((120-240) / 480.0) * Height + Height / 2;
            */
        }

        enum PointerShape {RECTANGLE, ELLIPSE, STAR};
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
        void setLeftShape(PointerShape  shape)
        {
            switch(shape)
            {
                case PointerShape.ELLIPSE :
                    LeftPopup.Child = LeftEllipse;
                    break;
                case PointerShape.RECTANGLE :
                    LeftPopup.Child = LeftRect;
                    break;
                case PointerShape.STAR :
                    LeftPopup.Child = LeftStar;
                    break;
            }
        }
        void setRightShape(PointerShape  shape)
        {
            switch(shape)
            {
                case PointerShape.ELLIPSE :
                    RightPopup.Child = RightEllipse;
                    break;
                case PointerShape.RECTANGLE :
                    RightPopup.Child = RightRect;
                    break;
                case PointerShape.STAR :
                    RightPopup.Child = RightStar;
                    break;
            }
        }

        KinectSensor _sensor;
        bool closing = false;
        const int skeletonCount = 6;
        Skeleton[] allSkeletons = new Skeleton[skeletonCount];

        Rectangle LeftRect, RightRect;
        Ellipse LeftEllipse, RightEllipse;
        Polygon LeftStar, RightStar;
        bool rightActive = false, leftActive = false;

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

            SetHandPositions(first, e);
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
        private void SetHandPositions(Skeleton skeleton, AllFramesReadyEventArgs e)
        {
            using (DepthImageFrame depth = e.OpenDepthImageFrame())
            {
                if (depth == null || _sensor == null)
                    return;

                double Width = MainWpfWindow.Width;
                double Height = MainWpfWindow.Height;
                Microsoft.Kinect.CoordinateMapper mapper = new CoordinateMapper(_sensor);

                DepthImagePoint headDepthPoint = mapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.Head].Position, DepthImageFormat.Resolution640x480Fps30);
                DepthImagePoint leftHandDepthPoint = mapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.HandLeft].Position, DepthImageFormat.Resolution640x480Fps30);
                DepthImagePoint rightHandDepthPoint = mapper.MapSkeletonPointToDepthPoint(skeleton.Joints[JointType.HandRight].Position, DepthImageFormat.Resolution640x480Fps30);

                /*
                 * Code that transforms skeleton to color image coordinates,
                 * Used if skeleton data should be displayed on the camera color image, removing the minor error that is between color and depth data.
                 */
                /*
                ColorImagePoint headColorPoint = depth.MapToColorImagePoint(headDepthPoint.X, headDepthPoint.Y, ColorImageFormat.RgbResolution640x480Fps30);
                ColorImagePoint leftHandColorPoint = depth.MapToColorImagePoint(leftHandDepthPoint.X, leftHandDepthPoint.Y, ColorImageFormat.RgbResolution640x480Fps30);
                ColorImagePoint rightHandColorPoint = depth.MapToColorImagePoint(rightHandDepthPoint.X, rightHandDepthPoint.Y, ColorImageFormat.RgbResolution640x480Fps30);

                Console.Write("right hand color image pos: (" + rightHandColorPoint.X + ", " + rightHandColorPoint.Y + ")");
                Console.Write("right hand color image pos: (" + leftHandColorPoint.X + ", " + leftHandColorPoint.Y + ")");

                RightEllipse.HorizontalOffset = ((rightHandColorPoint.X-320) / 640.0) * Width + Width / 2;
                RightEllipse.VerticalOffset = ((rightHandColorPoint.Y-240) / 480.0) * Height + Height / 2;

                LeftEllipse.HorizontalOffset = ((leftHandColorPoint.X-320) / 640.0) * Width + Width / 2;
                LeftEllipse.VerticalOffset = ((leftHandColorPoint.Y-240) / 480.0) * Height + Height / 2;

                */

                //                Using the Coding4Fun assembly to calculate hand positions
                Joint left_hand = skeleton.Joints[JointType.HandLeft];
                Joint right_hand = skeleton.Joints[JointType.HandRight];

                Joint left_hand_scaled = left_hand.ScaleTo((int)Width, (int)Height * 2, 0.5f, 0.5f);
                Joint right_hand_scaled = right_hand.ScaleTo((int)Width, (int)Height * 2, 0.5f, 0.5f);

                RightPopup.HorizontalOffset = right_hand_scaled.Position.X;
                RightPopup.VerticalOffset = right_hand_scaled.Position.Y;

                LeftPopup.HorizontalOffset = left_hand_scaled.Position.X;
                LeftPopup.VerticalOffset = left_hand_scaled.Position.Y;


                /*
                 * If the hand positions are (250) mm closer than the head position
                 */
                int threshold = 250;
                if (headDepthPoint.Depth > leftHandDepthPoint.Depth + threshold)
                {
                    if(!leftActive)
                        setLeftColor(System.Windows.Media.Brushes.HotPink);
                    leftActive = true;
                }
                else
                {
                    if(leftActive)
                        setLeftColor(System.Windows.Media.Brushes.White);
                    leftActive = false;
                }
                if (headDepthPoint.Depth > rightHandDepthPoint.Depth + threshold)
                {
                    if(!rightActive)
                        setRightColor(System.Windows.Media.Brushes.HotPink);
                    rightActive = true;
                }
                else
                {
                    if(rightActive)
                        setRightColor(System.Windows.Media.Brushes.White);
                    rightActive = false;
                }
                // Code for gestures / interaction

                gestureStep(left_hand_scaled, right_hand_scaled, skeleton);

            }
        }

        LinkedList<Joint> prev_left = new LinkedList<Joint>();
        LinkedList<Joint> prev_right = new LinkedList<Joint>();
        enum GestureState { ZOOM, SCROLL, CLICK, UNKNOWN };
        GestureState current_state;

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
                SetState(GestureState.UNKNOWN);

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
                Console.WriteLine("dx: " + dx + " dy: " + dy + " dz: " + dz);
                if(dz > 0.1 &&
                    Math.Abs(dx) < 100 &&
                    Math.Abs(dy) < 100)
                {
                    setRightColor(System.Windows.Media.Brushes.Red);
                    SetState(GestureState.CLICK);
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

                if(current_state == GestureState.ZOOM)
                {
                    if (prev_left.Count > 1 && prev_right.Count > 1)
                    {
                        double current_dist = Math.Sqrt(Math.Pow(left_hand_scaled.Position.X - right_hand_scaled.Position.X, 2) + Math.Pow(left_hand_scaled.Position.Y - right_hand_scaled.Position.Y, 2));
                        double prev_dist = Math.Sqrt(Math.Pow(prev_left.ElementAt(1).Position.X - prev_right.ElementAt(1).Position.X, 2) + Math.Pow(prev_left.ElementAt(1).Position.Y - prev_right.ElementAt(1).Position.Y, 2));
                        double zoom = current_dist - prev_dist;
                        Console.WriteLine(zoom);
                        // call zoom function
                    }
                }
                else if(current_state == GestureState.SCROLL)
                {
                    if (prev_left.Count > 1 && prev_right.Count > 1)
                    {
                        double current_x = (left_hand_scaled.Position.X + right_hand_scaled.Position.X) / 2;
                        double prev_x = (prev_left.ElementAt(1).Position.X + prev_right.ElementAt(1).Position.X) / 2;
                        double current_y = (left_hand_scaled.Position.Y + right_hand_scaled.Position.Y) / 2;
                        double prev_y = (prev_left.ElementAt(1).Position.Y + prev_right.ElementAt(1).Position.Y) / 2;
                        double dx = current_x - prev_x;
                        double dy = current_y - prev_y;
                        Console.WriteLine("Scrolling dx: " + dx + ", dy: " + dy);
                    }
                }
                Console.WriteLine("ldx: " + left_dx + ", ldy: " + left_dy + ", rdx: " + right_dx + ", rdy: " + right_dy);
                Console.WriteLine(current_state);
            }
        }

        void SetState(GestureState new_state)
        {
            if (current_state == new_state)
                return;
            Console.WriteLine("Changing state from: " + current_state + " to: " + new_state);
            current_state = new_state;
            switch(new_state)
            {
                case GestureState.ZOOM :
                    setLeftShape(PointerShape.STAR);
                    setRightShape(PointerShape.STAR);
                    break;
                case GestureState.SCROLL :
                    setLeftShape(PointerShape.RECTANGLE);
                    setRightShape(PointerShape.RECTANGLE);
                    break;
                case GestureState.CLICK :
                    break;
                case GestureState.UNKNOWN :
                    setLeftShape(PointerShape.ELLIPSE);
                    setRightShape(PointerShape.ELLIPSE);
                    break;
            }



        }
    }
}
