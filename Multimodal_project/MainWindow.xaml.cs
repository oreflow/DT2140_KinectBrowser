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

        
        public MainWindow()
        {
            InitializeComponent();
            browser.Navigate("http://www.google.se/");

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

        KinectSensor _sensor;
        bool closing = false;
        const int skeletonCount = 6;
        Skeleton[] allSkeletons = new Skeleton[skeletonCount];

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            if(KinectSensor.KinectSensors.Count > 0)
            {
                _sensor = KinectSensor.KinectSensors[0];
                if(_sensor.Status == KinectStatus.Connected)
                {
                    _sensor.ColorStream.Enable();
                    _sensor.DepthStream.Enable();
                    _sensor.SkeletonStream.Enable();
                    _sensor.AllFramesReady += _sensor_AllFramesReady;
                    try
                    {
                        _sensor.Start();
                    }
                    catch(System.IO.IOException)
                    {

                    }
                }
            }

        }
        private void StopKinect(KinectSensor sensor)
        {
            if(sensor != null && sensor.IsRunning)
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
                if(skeletonFrameData == null)
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
                DepthImagePoint headDepthPoint = depth.MapFromSkeletonPoint(skeleton.Joints[JointType.Head].Position);
                DepthImagePoint leftHandDepthPoint = depth.MapFromSkeletonPoint(skeleton.Joints[JointType.HandLeft].Position);
                DepthImagePoint rightHandDepthPoint = depth.MapFromSkeletonPoint(skeleton.Joints[JointType.HandRight].Position);

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

                Joint left_hand_scaled = left_hand.ScaleTo((int)Width, (int)Height*2, 0.5f, 0.5f);
                Joint right_hand_scaled = right_hand.ScaleTo((int)Width, (int)Height*2, 0.5f, 0.5f);

                RightEllipse.HorizontalOffset = right_hand_scaled.Position.X;
                RightEllipse.VerticalOffset = right_hand_scaled.Position.Y;

                LeftEllipse.HorizontalOffset = left_hand_scaled.Position.X;
                LeftEllipse.VerticalOffset = left_hand_scaled.Position.Y;


                /*
                 * If the hand positions are (250) mm closer than the head position
                 */
                int threshold = 250;
                if (headDepthPoint.Depth > leftHandDepthPoint.Depth + threshold)
                {
                    LeftEllipseInner.Stroke = System.Windows.Media.Brushes.HotPink;
                }
                else
                {
                    LeftEllipseInner.Stroke = System.Windows.Media.Brushes.White;
                }
                if (headDepthPoint.Depth > rightHandDepthPoint.Depth + threshold)
                {
                    RightEllipseInner.Stroke = System.Windows.Media.Brushes.HotPink;
                }
                else
                {
                    RightEllipseInner.Stroke = System.Windows.Media.Brushes.White;
                }
            }
        }

    }
}
