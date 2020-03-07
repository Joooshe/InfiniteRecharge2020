/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.subsystems.LineSensors;
import frc.robot.subsystems.Wrist;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.RobotMap;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static DriveTrain driveTrain;
  public static OI m_oi;
  public static HatchMechanism hatchMechanism;
  public static Wrist wrist;
  public static LineSensors lineSensors;
  public static Compressor compressor;
  public static boolean isExtended;

  public static NetworkTable table;

  public static boolean isForward; //true is facing cargo

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    RobotMap.gyro.calibrate();
    isForward = false;
    isExtended = false;

    driveTrain = new DriveTrain();

    hatchMechanism = new HatchMechanism();
    wrist = new Wrist();
    lineSensors = new LineSensors();
    compressor = new Compressor(0);

    m_oi = new OI();
    

    driveTrain.backLeftMaster.set(ControlMode.Position, 10*4096 );
    SmartDashboard.putString("Direction", Robot.isForward ? "Facing Cargo" : "Facing Hatch");
    
    //driveTrain.limeLight.turnOnLight();
    
    //CameraServer.getInstance().startAutomaticCapture(0);
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
        outputStream.putFrame(output);
      }
    }).start();
    */

    table = NetworkTableInstance.getDefault().getTable("limelight");
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
    compressor.setClosedLoopControl(true);
    Dashboard();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    //driveTrain.limeLight.turnOffLight();
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

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    driveTrain.backLeftMaster.set(ControlMode.Position, 10*4096 );
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    lineSensors.display();
    Scheduler.getInstance().run();
    
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void Dashboard() {
  
    SmartDashboard.putNumber("Gyro", RobotMap.gyro.getAngle());
    SmartDashboard.putNumber("X", m_oi.controller.getX(Hand.kLeft));
    SmartDashboard.putNumber("Y", m_oi.controller.getY(Hand.kLeft));
    SmartDashboard.putNumber("Z", m_oi.controller.getX(Hand.kRight));
    
    SmartDashboard.putNumber("Encoder Position In Rotations", driveTrain.backLeftMaster.getSelectedSensorPosition()/4096);
    SmartDashboard.putNumber("Encoder Position Raw Units", driveTrain.backLeftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("Encoder Velocity", driveTrain.backLeftMaster.getSelectedSensorVelocity());
    //2121
    driveTrain.displayMotorOutput();

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("PID TX", Robot.driveTrain.limeLight.getTX());

    SmartDashboard.putNumber("Controller Left Y", m_oi.getController().getY(Hand.kLeft));
    SmartDashboard.putNumber("Controller Left X", m_oi.getController().getX(Hand.kLeft));

    LiveWindow.updateValues();
  }
}
