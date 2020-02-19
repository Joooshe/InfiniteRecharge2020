/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SecondTread;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.cameraserver.CameraServer;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;

import com.frc2020.GripPipeline;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;

  public static DriveTrain driveTrain;
  public static Shooter shooter;
  public static SecondTread secondTread;
  public static Elevator elevator;

  public static Timer timer;

  public static UsbCamera usbCamera;
  public static MjpegServer mjpegServer1;
  public static CvSink cvSink;
  public static CvSource outputStream;
  public static MjpegServer mjpegServer2;
  public static Mat source;
  public static Mat output;

  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 240;

  private VisionThread visionThread;
  private double centerX = 0.0;

  private final Object imgLock = new Object();

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //m_oi = new OI();
    driveTrain = new DriveTrain();
    shooter = new Shooter();
    secondTread = new SecondTread();
    elevator = new Elevator();

    timer = new Timer();

    m_oi = new OI();


    CameraInit();

    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

/*
    new Thread(() -> {
      
      CameraInit();

      //Where you put your continuous code
      while(!Thread.interrupted()){
        cvSink.grabFrame(source);
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
      }

    }).start();
  */
    
    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
      camera.setResolution(640, 480);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Camera Test", 640, 480);

      Mat source = new Mat();
      Mat output = new Mat();

      while(!Thread.interrupted()) {
        cvSink.grabFrame(source);
        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
        outputStream.putFrame(output);
      }
    }).start();
    
  }

  public void CameraInit() {
    
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
        if(!pipeline.findContoursOutput().isEmpty()) {
          Rect r = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
          synchronized (imgLock) {
            centerX = r.x + (r.width / 2);
          }
        }
    });

    visionThread.start();

    /*
    // Creates UsbCamera and MjpegServer [1] and connects them
    usbCamera = new UsbCamera("USB Camera 0", 0);
    mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
    mjpegServer1.setSource(usbCamera);

    // Creates the CvSink and connects it to the UsbCamera
    cvSink = new CvSink("opencv_USB Camera 0");
    cvSink.setSource(usbCamera);

    // Creates the CvSource and MjpegServer [2] and connects them
    outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    mjpegServer2 = new MjpegServer("serve_Blur", 1182);
    mjpegServer2.setSource(outputStream);

    source = new Mat();
    output = new Mat();
    */
  }

  public void CameraPeriodic() {
    //cvSink.grabFrame(image)
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Dashboard();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    timer.reset();
    timer.start();
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    AutonomousCode();
  }

  public void AutonomousCode() {

    driveTrain.driveStraight();

    if(timer.get() >= 0.5) {
      driveTrain.stop();
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void Dashboard() {
    SmartDashboard.putNumber("Gyro", RobotMap.kgyro.getAngle());
    SmartDashboard.putNumber("X", m_oi.controller.getX(Hand.kLeft));
    SmartDashboard.putNumber("Y", m_oi.controller.getY(Hand.kLeft));
    SmartDashboard.putNumber("Z", m_oi.controller.getX(Hand.kRight));
  
    SmartDashboard.putNumber("Talon 1 Speed", RobotMap.kfrontLeft.get());
    SmartDashboard.putNumber("Talon 1 Bus Voltage", RobotMap.kfrontLeft.getBusVoltage());
    SmartDashboard.putNumber("Talon 2 Speed", RobotMap.kbackLeft.get());
    SmartDashboard.putNumber("Talon 2 Bus Voltage", RobotMap.kbackLeft.getBusVoltage());
    SmartDashboard.putNumber("Talon 3 Speed", RobotMap.kfrontRight.get());
    SmartDashboard.putNumber("Talon 3 Bus Voltage", RobotMap.kfrontRight.getBusVoltage());
    SmartDashboard.putNumber("Talon 4 Speed", RobotMap.kbackRight.get());
    SmartDashboard.putNumber("Talon 4 Bus Voltage", RobotMap.kbackRight.getBusVoltage());
  }

}
