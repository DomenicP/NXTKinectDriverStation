using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NXTKinectDriverStation
{
    using NKH.MindSqualls;
    using System.IO;

    class Robot
    {
        #region Private Fields
        private NxtBrick m_brick;
        private NxtMotor m_leftMotor;
        private NxtMotor m_rightMotor;
        private NxtMotorSync m_motors;
        private bool m_connected;
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

            m_connected = false;
        }
        #endregion

        #region Connection Methods
        public void Connect()
        {
            try
            {
                m_brick.Connect();
            }
            catch (IOException)
            {
                // Do nothing ATM
            }
            m_connected = m_brick.IsConnected;
        }

        public void Disconnect()
        {
            if (m_connected)
                m_brick.Disconnect();
        }

        public bool IsConnected()
        {
            return m_connected;
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
            m_leftMotor.Coast();
            m_rightMotor.Coast();
        }
        #endregion
    }
}
