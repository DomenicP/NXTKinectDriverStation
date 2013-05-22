using System;
using NKH.MindSqualls;
using Microsoft.Kinect;

namespace MindsquallsNXT
{
    class Robot
    {
        #region Private Fields
        private NxtBrick m_brick;
        private NxtMotor m_leftMotor;
        private NxtMotor m_rightMotor;
        private NxtMotorSync m_motors;
        #endregion

        #region Constructor
        public Robot()
        {
            m_brick = new NxtBrick(NxtCommLinkType.Bluetooth, 3);
            m_leftMotor = new NxtMotor();
            m_rightMotor = new NxtMotor();

            m_brick.MotorB = m_leftMotor;
            m_brick.MotorC = m_rightMotor;

            m_motors = new NxtMotorSync(m_leftMotor, m_rightMotor);
        }
        #endregion

        #region Connection Methods
        public bool Connect()
        {
            m_brick.Connect();
            return m_brick.IsConnected;
        }

        public void Disconnect()
        {
            m_brick.Disconnect();
        }
        #endregion

        #region Driving Methods
        public void TankDrive(sbyte left, sbyte right)
        {
            m_leftMotor.Run(left, 0);
            m_rightMotor.Run(right, 0);
        }

        public void ArcadeDrive(sbyte move, sbyte turn)
        {
            m_motors.Run(move, 0, turn);
        }

        public void Brake()
        {
            m_motors.Brake();
        }

        public void Coast()
        {
            m_motors.Coast();
        }
        #endregion

        public ushort GetBatteryLevel()
        {
            return m_brick.BatteryLevel;
        }
    }

    class Joystick
    {
        public enum Axis
        {
            X,
            Y,
            Z,
            Twist,
            Throttle,
            Custom
        }

        public enum Buttons
        {
            Btn1 = 1,
            Btn2 = 2,
            Btn3 = 4,
            Btn4 = 8,
            Btn5 = 16,
            Btn6 = 32,
            Btn7 = 64,
            Btn8 = 128,
            Btn9 = 256,
            Btn10 = 512,
            Btn11 = 1024,
            Btn12 = 2048,
        };

        protected sbyte[] m_axis;
        protected ushort m_buttons;

        public Joystick(sbyte[] axis, ushort buttons)
        {
            m_axis = axis;
            m_buttons = buttons;
        }

        public void Set(sbyte[] axis, ushort buttons)
        {
            if (axis == null)
                throw new ArgumentException("Array argument must not be null");

            m_axis = axis;
            m_buttons = buttons;
        }

        public sbyte[] getAxis()
        {
            sbyte[] copy = new sbyte[m_axis.Length];
            m_axis.CopyTo(copy, 0);
            return copy;
        }

        public ushort getButtons()
        {
            return m_buttons;
        }
    }

    class GestureProcessor
    {
        public const double Z_PLANE_TOLERANCE = 0.3;

        public const double ARM_MAX_ANGLE = 105;
        public const double ARM_MIN_ANGLE = -90;

        delegate bool CheckAngle(double angle);

        CheckAngle IsLegForward = x => x < -110;
        CheckAngle IsLegBackward = x => x > -80;
        CheckAngle IsLegOut = x => x > -75;
        CheckAngle IsHeadLeft = x => x > 98;
        CheckAngle IsHeadRight = x => x < 82;

        /// <summary>
        /// Processes a skeleton into joystick data using the default FIRST gestures.
        /// </summary>
        /// <param name="joy">Vector of Joysticks to put the result in</param>
        /// <param name="skeleton">The skeleton to process</param>
        public void ProcessGestures(Joystick[] joy, Microsoft.Kinect.Skeleton skeleton)
        {

            // Check edge cases
            if (joy == null || joy.Length < 2 || joy[0] == null || joy[1] == null)
                return;

            sbyte[] leftAxis = new sbyte[6];
            sbyte[] rightAxis = new sbyte[6];
            sbyte[] nullAxis = new sbyte[6];
            bool dataWithinExpectedRange;
            ushort buttons = 0;

            double leftAngle = RadToDeg(AngleXY(skeleton.Joints[JointType.ShoulderLeft].Position,
                                                skeleton.Joints[JointType.WristLeft].Position,
                                                true));

            double rightAngle = RadToDeg(AngleXY(skeleton.Joints[JointType.ShoulderRight].Position,
                                                 skeleton.Joints[JointType.WristRight].Position));

            dataWithinExpectedRange = leftAngle < ARM_MAX_ANGLE && leftAngle > ARM_MIN_ANGLE &&
                                      rightAngle < ARM_MAX_ANGLE && rightAngle > ARM_MIN_ANGLE;

            double leftYAxis = CoerceToRange(leftAngle,
                                             -70,
                                             70,
                                             -127,
                                             128);

            double rightYAxis = CoerceToRange(rightAngle,
                                             -70,
                                             70,
                                             -127,
                                             128);

            dataWithinExpectedRange = dataWithinExpectedRange &&
                                      InSameZPlane(skeleton.Joints[JointType.ShoulderLeft].Position,
                                                   skeleton.Joints[JointType.WristLeft].Position,
                                                   Z_PLANE_TOLERANCE) &&
                                      InSameZPlane(skeleton.Joints[JointType.ShoulderRight].Position,
                                                   skeleton.Joints[JointType.WristRight].Position,
                                                   Z_PLANE_TOLERANCE);

            // Head buttons
            double headAngle = RadToDeg(AngleXY(skeleton.Joints[JointType.ShoulderCenter].Position,
                                                skeleton.Joints[JointType.Head].Position));
            if (IsHeadRight(headAngle))
                buttons |= (ushort)Joystick.Buttons.Btn1;
            if (IsHeadLeft(headAngle))
                buttons |= (ushort)Joystick.Buttons.Btn2;


            // Right Leg XY Button
            double rightLegAngle = RadToDeg(AngleXY(skeleton.Joints[JointType.HipRight].Position,
                                                    skeleton.Joints[JointType.AnkleRight].Position));
            if (IsLegOut(rightLegAngle))
                buttons |= (ushort)Joystick.Buttons.Btn3;

            // Left Leg XY Button
            double leftLegAngle = RadToDeg(AngleXY(skeleton.Joints[JointType.HipLeft].Position,
                                                   skeleton.Joints[JointType.AnkleLeft].Position,
                                                   true));
            if (IsLegOut(leftLegAngle))
                buttons |= (ushort)Joystick.Buttons.Btn4;

            // Right Leg YZ Buttons
            double rightLegYZ = RadToDeg(AngleYZ(skeleton.Joints[JointType.HipRight].Position,
                                                 skeleton.Joints[JointType.AnkleRight].Position));
            if (IsLegForward(rightLegYZ))
                buttons |= (ushort)Joystick.Buttons.Btn5;
            if (IsLegBackward(rightLegYZ))
                buttons |= (ushort)Joystick.Buttons.Btn6;

            // Left Leg YZ Buttons
            double leftLegYZ = RadToDeg(AngleYZ(skeleton.Joints[JointType.HipLeft].Position,
                                                skeleton.Joints[JointType.AnkleLeft].Position));
            if (IsLegForward(leftLegYZ))
                buttons |= (ushort)Joystick.Buttons.Btn7;
            if (IsLegBackward(leftLegYZ))
                buttons |= (ushort)Joystick.Buttons.Btn8;

            if (dataWithinExpectedRange)
            {
                // Invert joystick axis to match a real joystick
                // (pushing away creates negative values)
                leftAxis[(uint)Joystick.Axis.Y] = (sbyte)-leftYAxis;
                rightAxis[(uint)Joystick.Axis.Y] = (sbyte)-rightYAxis;

                //Use "Button 9" as Kinect control enabled signal
                buttons |= (ushort)Joystick.Buttons.Btn9;

                joy[0].Set(leftAxis, buttons);
                joy[1].Set(rightAxis, buttons);
            }
            else
            {
                joy[0].Set(nullAxis, 0);
                joy[1].Set(nullAxis, 0);

            }
        }

        /// <summary>
        /// Converts units from radians to degrees.
        /// </summary>
        /// <param name="rad">A value in radians.</param>
        /// <returns>The given value in degrees.</returns>
        private double RadToDeg(double rad)
        {
            return (rad * 180) / Math.PI;
        }

        /// <summary>
        /// Calculates the XY plane tangent between the given vectors.
        /// </summary>
        /// <param name="origin">The first point.</param>
        /// <param name="measured">The second point.</param>
        /// <param name="mirrored">Whether or not to invert the X axis.</param>
        /// <returns></returns>
        private double AngleXY(SkeletonPoint origin, SkeletonPoint measured, bool mirrored = false)
        {
            return Math.Atan2(measured.Y - origin.Y, (mirrored) ? (origin.X - measured.X) : (measured.X - origin.X));
        }

        /// <summary>
        /// Calculates the YZ plane tangent between the given vectors.
        /// </summary>
        /// <param name="origin">The first point.</param>
        /// <param name="measured">The second point.</param>
        /// <param name="mirrored">Whether or not to invert the Z axis.</param>
        /// <returns></returns>
        private double AngleYZ(SkeletonPoint origin, SkeletonPoint measured, bool mirrored = false)
        {
            return Math.Atan2(measured.Y - origin.Y, (mirrored) ? (origin.Z - measured.Z) : (measured.Z - origin.Z));
        }

        /// <summary>
        /// Determines whether the given points lie in the same XY plane along the Z axis.
        /// </summary>
        /// <param name="origin">The first point.</param>
        /// <param name="measured">The second point.</param>
        /// <param name="tolerance">The acceptable tolerance between the XY planes of the given points.</param>
        /// <returns>Whether or not the given points are close enough along the Z axis.</returns>
        private bool InSameZPlane(SkeletonPoint origin, SkeletonPoint measured, double tolerance)
        {
            return Math.Abs(measured.Z - origin.Z) < tolerance;
        }

        /// <summary>
        /// Converts an input value in the given input range into an output value along the given
        /// output range.
        /// 
        /// If the result would be outside of the given output range, it is constrained to the 
        /// output range.
        /// </summary>
        /// <param name="input">An input value within the given input range.</param>
        /// <param name="inputMin">The minimum expected input value.</param>
        /// <param name="inputMax">The maximum expected input value.</param>
        /// <param name="outputMin">The minimum expected output value.</param>
        /// <param name="outputMax">The maximum expected output value.</param>
        /// <returns>An output value within the given output range proportional to the input.</returns>
        private double CoerceToRange(double input, double inputMin, double inputMax, double outputMin, double outputMax)
        {
            // Determine the center of the input range
            double inputCenter = Math.Abs(inputMax - inputMin) / 2 + inputMin;
            double outputCenter = Math.Abs(outputMax - outputMin) / 2 + outputMin;

            // Scale the input range to the output range
            double scale = (outputMax - outputMin) / (inputMax - inputMin);

            // Apply the transformation
            double result = (input + -inputCenter) * scale + outputCenter;

            // Constrain to the result range
            return Math.Max(Math.Min(result, outputMax), outputMin);
        }
    }

    class Program
    {
        static bool finished = false;
        static Skeleton[] skeletonData;

        static void Main(string[] args)
        {

            Console.CancelKeyPress += new ConsoleCancelEventHandler(Console_CancelKeyPress);

            Robot robot = new Robot();
            KinectSensor sensor = null;
            Joystick[] joysticks = { new Joystick(new sbyte[6], 0), new Joystick(new sbyte[6], 0) };
            GestureProcessor processor = new GestureProcessor();

            // Connect to the robot
            if (robot.Connect())
            {
                Console.WriteLine("Connected to NXT; Battery Level: {0}", robot.GetBatteryLevel());
                // Connect to the Kinect
                foreach (KinectSensor s in KinectSensor.KinectSensors)
                    if (s.Status == KinectStatus.Connected)
                        sensor = s;

                if (sensor != null)
                {
                    Console.WriteLine("Connected to Kinect");
                    sensor.SkeletonStream.Enable();
                    skeletonData = new Skeleton[sensor.SkeletonStream.FrameSkeletonArrayLength];
                    sensor.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(sensor_SkeletonFrameReady);

                    sensor.Start();

                    Console.WriteLine("Entering Main Loop");
                    while (!finished)
                    {
                        if (skeletonData[0] != null)
                        {
                            // Process skeleton data
                            processor.ProcessGestures(joysticks, skeletonData[0]);
                        }
                        else
                        {
                            Console.WriteLine("Error: invalid skeleton data");
                        }

                        sbyte left = joysticks[0].getAxis()[(uint) Joystick.Axis.Y];
                        sbyte right = joysticks[1].getAxis()[(uint) Joystick.Axis.Y];
                        Console.WriteLine("Left: {0}, Right: {1}", left, right);
                        robot.TankDrive(left, right);
                        
                    }

                    sensor.Stop();
                }
                else
                {
                    Console.WriteLine("Error: Failed to connect to the Kinect Sensor");
                }
            }
            else
            {
                Console.WriteLine("Error: failed to connect to the NXT!");
            }

            // Disconnect from the robot
            robot.Disconnect();

            Console.WriteLine("Program Finished; press any key to exit...");
            Console.ReadKey();
        }

        static void sensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            //Console.WriteLine("Processing Skeleton Data");
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null && skeletonData != null)
                {
                    skeletonFrame.CopySkeletonDataTo(skeletonData);
                }
            }
        }

        static void Console_CancelKeyPress(object sender, ConsoleCancelEventArgs e)
        {
            finished = true;
        }
    }
}
