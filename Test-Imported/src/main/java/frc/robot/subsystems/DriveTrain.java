/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

/**
 * Add your docs here.
 */
public class DriveTrain extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  public static final double kf = 3.0;

  private static double kP = 1.0;
  private static double kI = 0.0;
  private static double kD = 1.2;

  private static double kP2 = 1.0;
  private static double kI2 = 1.0;
  private static double kD2 = 1.0;

  public static PIDController PID2;
  public static PIDSource magEncoder;

  private static WPI_VictorSPX frontLeftVictor;
  public  static WPI_TalonSRX backLeftTalon;
  private static WPI_VictorSPX frontRightVictor;
  private static WPI_VictorSPX backRightVictor;

  private static MecanumDrive drive;

  private static ADXRS450_Gyro gyro;

  public static final ShuffleboardTab tab = Shuffleboard.getTab("PID");
  
  public static NetworkTableEntry P =
                      tab.add("P", 1)
                      .getEntry();
  public static NetworkTableEntry I = 
                      tab.add("I", 0)
                      .withWidget(BuiltInWidgets.kNumberSlider)
                      .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
                      .getEntry();
  public static NetworkTableEntry D = 
                          tab.add("D", 0.5)
                              .getEntry();


  public DriveTrain() {
    // Intert a subsystem name and PID values here
    super("DriveTrain", kP*kf, kI*kf, kD*kf);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    frontLeftVictor = RobotMap.frontLeftVictor;
    backLeftTalon = RobotMap.backLeftVictor;
    backRightVictor = RobotMap.backRightVictor;
    frontRightVictor = RobotMap.frontRightVictor;

    PID2 = new PIDController(kP2, kI2, kD2);

    backLeftTalon.configFactoryDefault();
    backLeftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    //https://robotpy.readthedocs.io/projects/ctre/en/stable/api_mot.html#talonsrx

    gyro = RobotMap.gyro;

    drive = RobotMap.driveTrain;

    disable();
    setInputRange(0, 360);
    getPIDController().setContinuous(true);
    setOutputRange(-0.45, 0.45);
    setAbsoluteTolerance(2);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveCommand());
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public void stop() {
    drive.driveCartesian(0, 0, 0);
    disable();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }


  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return gyro.getAngle();
  }

  public static void configurePID() {
/*
    NetworkTableEntry P = Shuffleboard.getTab("PID") 
    .add("P", 1)
    .getEntry();

    NetworkTableEntry I = Shuffleboard.getTab("PID")
    .add("I", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
    .getEntry();

    NetworkTableEntry D = Shuffleboard.getTab("PID")
    .add("D", 0.5)
    .getEntry();


    kP = P.getDouble(1.0);
    kI = I.getDouble(1.0);
    kD = D.getDouble(1.0);
    */
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    configurePID();
    frontLeftVictor.pidWrite(output);
    backLeftTalon.pidWrite(output);
    frontRightVictor.pidWrite(output);
    backRightVictor.pidWrite(output);
    SmartDashboard.putNumber("Output", output);
    //wheels are 6 inch
  }

  public void move(XboxController controller) {

    //TEST
    //SmartDashboard.putNumber("Potentiometer", potent.get());

    if (controller.getAButton()) {
      //gyro.reset();
      drive.driveCartesian(1, 0, 0);
    }
    if (controller.getBButton()) {
      //SmartDashboard.putString("lkhfalhflhkahjlasflhlhkaflhkaklaskjlfasklasfljk", "bjaskjasjhlasdlhjdsajhashjkhaksjuiqiuqkhebnbn");
    }
    if (Robot.isForward) {
      // first way we drove drive.driveCartesian(controller.getX(Hand.kLeft), -controller.getY(Hand.kLeft), controller.getX(Hand.kRight), gyro.getAngle());
      drive.driveCartesian(controller.getX(Hand.kLeft), -controller.getY(Hand.kLeft), controller.getX(Hand.kRight), gyro.getAngle());
    } else {
      // first way we drove drive.driveCartesian(-controller.getX(Hand.kLeft), +controller.getY(Hand.kLeft), controller.getX(Hand.kRight), gyro.getAngle());    
      //switches the front side of the robot
      drive.driveCartesian(-controller.getX(Hand.kLeft), controller.getY(Hand.kLeft), controller.getX(Hand.kRight), gyro.getAngle());
    }
  }


  public double decimalPlace(double num, int place) {
    return (((double)((int)(num*place)))/place);
  }

  public void reset() {
    gyro.reset();
  }

  public void forward() {
    if (Robot.isForward) {
      drive.driveCartesian(0, 1, 0);
    } else {
      drive.driveCartesian(0, -1, 0);
    }
  }

  public void forwardAtSpeed(double speed) {
    if (Robot.isForward) {
      drive.driveCartesian(0, speed, 0);
    } else {
      drive.driveCartesian(0, -speed, 0);
    }
  }

  public void turnLeft(double turnSpeed) {
    drive.driveCartesian(0, 0, turnSpeed);
  }

  public void turnRight(double turnSpeed) {
    drive.driveCartesian(0, 0, -turnSpeed);
  }

  public void backward() {
     if (Robot.isForward) {
      drive.driveCartesian(1, 0, 0);
    } else { 
      drive.driveCartesian(-1, 0, 0);
    }
  }

  public void backwardAtSpeed(double speed) {
    if (Robot.isForward) {
      drive.driveCartesian(0, -speed, 0); //the negative and positive signs may need to be changed depending on which way is forward and backwards
   } else { 
    drive.driveCartesian(0, speed, 0);
   }
 }
}
