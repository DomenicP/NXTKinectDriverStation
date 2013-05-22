## NXT Kinect Driver Station

This is a small app that allows you to drive a LEGO NXT robot over Bluetooth via
the Microsoft Kinect. It provides a two motor tank drive using the arms as virtual
joysticks.

### Requirements  

- a Bluetooth capable PC running Windows 7
- Kinect SDK 1.7 installation
- Microsoft Visual Studio Express 2012 for Windows Desktop (or equivalent)
- MindSqualls .NET library v2.2 found [here][1]

### Usage

There are two projects in the solution: NXTKinectConsole, and NXTKinectDriverStation. They are mostly functionaly equivalent;
the main difference being that the first is a console application, while the second provides a GUI. The program is setup to drive
the robot in a tank-drive style: movement of the left arm coresponds to the left side of the drivetrain, while movement  of the right
arm controls the right side of the drivetrain. Folding the arms across the chest is treated as a neutral output.

---

This software uses code from the 2013 [FIRST Robotics Competition][2] Kinect Server. For details, see "License_for_KinectServer_code.txt"  
LEGO is a trademark and/or copyright of the LEGO Group.  
Microsoft and Kinect are trademarks and/or copyright of Microsoft Corporation.

[1]: MindSqualls "http://www.mindsqualls.net"
[2]: FRC "http://www.usfirst.org/frc"