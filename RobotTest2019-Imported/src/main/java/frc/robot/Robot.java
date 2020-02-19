/*Joshua: The robot file is where your robots brains are located. It is where all 
your commands and buttons interact with eachother to tell the motors and sensors 
what to do */

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.AnalogInput;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;

import javax.xml.transform.Source;

import com.TestGripPipeline;
import com.revrobotics.ColorMatch;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

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

  public static DriveTrainPID driveTrainPID;

  //Ultra sonic sensor object and constants
  public static AnalogInput m_ultrasonic;
  public static final double kValueToInches = 0.125; //ultra sonic conversion factor
  public static final double kP = 0.05; //proportional speed constant
  
  public static AnalogInput potent;
  
  public static AnalogInput lightSensor;

  public static ADXRS450_Gyro gyro;

  public static ColorSensorV3 m_colorSensor;
  public static final ColorMatch m_colorMatcher = new ColorMatch();
  public static final I2C.Port i2cport = I2C.Port.kOnboard;
  
  private static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 240;

  private VisionThread visionThread;
  private double centerX = 0.0;
  private TestGripPipeline gripPipeline;

  private final Object imgLock = new Object();

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();

    driveTrain = new DriveTrain();
    driveTrainPID = new DriveTrainPID();
    System.out.println("Test");
    System.out.println(driveTrainPID.getGyroAngle());
    //m_ultrasonic = new AnalogInput(RobotMap.m_ultrasonic);
    potent = new AnalogInput(RobotMap.potent);
    lightSensor = new AnalogInput(RobotMap.lightSensor);
    
    m_colorSensor = new ColorSensorV3(i2cport);
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    gyro = RobotMap.gyro; //whenever you make a gyro you need to calibrate it
    gyro.calibrate();

    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

    gripPipeline = new TestGripPipeline();

    /*
    UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
    mjpegServer1.setSource(usbCamera);

    // Creates the CvSink and connects it to the UsbCamera
    CvSink cvSink = new CvSink("opencv_USB Camera 0");
    cvSink.setSource(usbCamera);

    // Creates the CvSource and MjpegServer [2] and connects them
    CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
    mjpegServer2.setSource(outputStream);
    */
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();


    

    visionThread = new VisionThread(camera, new TestGripPipeline(), pipeline -> {
      if (!pipeline.findContoursOutput().isEmpty()) {
          Rect r = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
          synchronized (imgLock) {
              centerX = r.x + (r.width / 2);
          }
      }

  });
  visionThread.start();

    

    new Thread(() -> {
      // Creates UsbCamera and MjpegServer [1] and connects them
      CameraServer.getInstance().startAutomaticCapture();

      // Creates the CvSink and connects it to the UsbCamera
      CvSink cvSink = CameraServer.getInstance().getVideo();

      // Creates the CvSource and MjpegServer [2] and connects them
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", IMG_WIDTH, IMG_HEIGHT);

      while(!Thread.interrupted()) {
        Mat source = new Mat();
        Mat output1 = new Mat();
        Mat output2 = new Mat();

         if (cvSink.grabFrame(source) == 0) {
          // Send the output the error.
          outputStream.notifyError(cvSink.getError());
          // skip the rest of the current iteration
          continue;
        }
        gripPipeline.blur(source, output1);
        gripPipeline.hsvThreshold(output1, output2);
        cvSink.grabFrame(source);
        gripPipeline.process(source);
        outputStream.putFrame(output2);

      }

    }).start();

    /*
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    visionThread = new VisionThread(camera, new TestGripPipeline(), pipeline -> {
        if (!pipeline.findContoursOutput().isEmpty()) {
            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
            synchronized (imgLock) {
                centerX = r.x + (r.width / 2);
            }
        }
    });
    visionThread.start();
    */

  /*
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
        gripPipeline.process(source);
        outputStream.putFrame(source);
      }
    }).start();
    */
  
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
    simpleVisionTurn();
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
    //Dashboard();
  }

  //where we put all our dashboard commands
  public void Dashboard() {
    Shuffleboard.getTab("Test");
    
    SmartDashboard.putNumber("Center X", centerX);
    /*
    SmartDashboard.putNumber("Ultra Sonic Konstant", m_ultrasonic.getValue() * kValueToInches);
    SmartDashboard.putNumber("Ultra Sonic getVoltage", m_ultrasonic.getVoltage());
    SmartDashboard.putNumber("Ultra Sonic getValue", m_ultrasonic.getVoltage());
    */

    SmartDashboard.putNumber("Gyro Raw", gyro.getAngle());

    SmartDashboard.putNumber("Light", lightSensor.getVoltage());

    SmartDashboard.putNumber("Potentiometer", potent.getVoltage());

    calculateColor();

    //SmartDashboard.putData(value);
  }

  public void simpleVisionTurn() {
    double centerX;
    synchronized (imgLock) {
        centerX = this.centerX;
    }
    double turn = centerX - (IMG_WIDTH / 2);
    driveTrain.arcadeDrive(-0.6, turn * 0.005);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public static ADXRS450_Gyro getGyro() {
    return gyro;
  }

  public void calculateColor() {
    Color detectedColor = m_colorSensor.getColor();

    String colorString;

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  
  }

    public void CameraInit() {
      UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
      MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
      mjpegServer1.setSource(usbCamera);

      // Creates the CvSink and connects it to the UsbCamera
      CvSink cvSink = new CvSink("opencv_USB Camera 0");
      cvSink.setSource(usbCamera);

      // Creates the CvSource and MjpegServer [2] and connects them
      CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
      MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
      mjpegServer2.setSource(outputStream);
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    

}
