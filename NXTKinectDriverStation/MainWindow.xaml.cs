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

namespace NXTKinectDriverStation
{
    using Microsoft.Kinect;
    using System.IO;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Private Fields
        private KinectSensor kinect;
        private byte[] colorPixels;
        private WriteableBitmap colorBitmap;
        private Skeleton[] skeletonData;
        
        private Robot robot;

        private Joystick[] joysticks = { new Joystick(new sbyte[6], 0), new Joystick(new sbyte[6], 0) };
        private GestureProcessor processor;
        #endregion

        #region Window Lifecycle
        public MainWindow()
        {
            InitializeComponent();
        }

        // Initialize the program
        private void Window_Initialized(object sender, EventArgs e)
        {
            foreach (KinectSensor potentialSensor in KinectSensor.KinectSensors)
                if (potentialSensor != null)
                {
                    this.kinect = potentialSensor;
                    break;
                }

            if (kinect != null)
            {
                // Kinect color stream
                kinect.ColorStream.Enable();
                colorPixels = new byte[kinect.ColorStream.FramePixelDataLength];
                colorBitmap = new WriteableBitmap(kinect.ColorStream.FrameWidth, 
                    kinect.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
                kinect.ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(kinect_ColorFrameReady);
                this.kinectImage.Source = colorBitmap;

                // Kinect Skeleton Stream
                kinect.SkeletonStream.Enable();
                kinect.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(kinect_SkeletonFrameReady);
                skeletonData = new Skeleton[kinect.SkeletonStream.FrameSkeletonArrayLength];

                try
                {
                    kinect.Start();
                    kinectConnectedText.Text = "";
                }
                catch (IOException)
                {
                    kinect = null;
                    kinectConnectedText.Text = "Kinect Not Connected";
                }
            }

            robot = new Robot();
            robot.Connect();

            if (robot.IsConnected())
                nxtConnectedText.Text = "";
            else
                nxtConnectedText.Text = "NXT Not Connected";

            processor = new GestureProcessor();
        }

        // Cleanup before closing
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (kinect != null)
            {
                kinect.Stop();
            }

            if (robot.IsConnected())
            {
                robot.Coast();
                robot.Disconnect();
            }
        }
        #endregion

        #region Button Handlers
        private void ExitButtonClicked(object sender, RoutedEventArgs e)
        {
            Close();
        }

        private void KinectButtonClicked(object sender, RoutedEventArgs e)
        {
            foreach (KinectSensor potentialSensor in KinectSensor.KinectSensors)
                if (potentialSensor != null)
                {
                    this.kinect = potentialSensor;
                    break;
                }

            if (this.kinect != null)
            {
                kinect.ColorStream.Enable();
                kinect.ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(kinect_ColorFrameReady);
                kinect.Start();
            }
        }

        private void NxtButtonClicked(object sender, RoutedEventArgs e)
        {
            robot.Connect();

            if (robot.IsConnected())
                nxtConnectedText.Text = "";
            else
                nxtConnectedText.Text = "NXT Not Connected";
        }
        #endregion

        #region Kinect Handlers
        void kinect_ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            using (ColorImageFrame frame = e.OpenColorImageFrame())
            {
                if (frame != null)
                {
                    // Copy the data to temporary storage
                    frame.CopyPixelDataTo(colorPixels);

                    this.colorBitmap.WritePixels(
                        new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight),
                        this.colorPixels,
                        this.colorBitmap.PixelWidth * sizeof(int),
                        0);
                }
            }
        }

        void kinect_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            using (SkeletonFrame skeleton = e.OpenSkeletonFrame())
            {
                if (skeleton != null && skeletonData != null)
                {
                    skeleton.CopySkeletonDataTo(skeletonData);

                    if (skeletonData[0] != null && robot.IsConnected())
                    {
                        processor.ProcessGestures(joysticks, skeletonData[0]);

                        sbyte left = joysticks[0].getAxis()[(uint)Joystick.Axis.Y];
                        sbyte right = joysticks[1].getAxis()[(uint)Joystick.Axis.Y];
                        leftOutput.Text = "Left: " + left.ToString();
                        rightOutput.Text = "Right: " + right.ToString();
                        robot.TankDrive(left, right);
                    }
                }
            }
        }
        #endregion
    }
}
