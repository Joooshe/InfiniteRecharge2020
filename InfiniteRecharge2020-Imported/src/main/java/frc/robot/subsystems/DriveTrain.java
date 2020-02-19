/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.*;
import frc.robot.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem implements PIDOutput{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX frontLeft;
  private WPI_TalonSRX frontRight;
  private WPI_TalonSRX backLeft;
  private WPI_TalonSRX backRight;

  private SpeedControllerGroup leftSide;
  private SpeedControllerGroup rightSide;

  private DifferentialDrive driveTrain;

  private static double kP = 1.0;
  private static double kI = 1.0;
  private static double kD = 1.0;
  
  private static double kP2 = 1.0;
  private static double kI2 = 1.0;
  private static double kD2 = 1.0;

  public PIDController PID;
  public PIDController PID2;

  public static ADXRS450_Gyro gyro;

  private static double PIDMotorSpeed;

  public DriveTrain() {
    
    frontLeft = RobotMap.kfrontLeft;
    frontRight = RobotMap.kfrontRight;
    backLeft = RobotMap.kbackLeft;
    backRight = RobotMap.kbackRight;

    frontLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backLeft.configFactoryDefault();
    backRight.configFactoryDefault();

    backLeft.follow(frontLeft);
    backRight.follow(frontRight);

    frontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    frontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    driveTrain = RobotMap.kdriveTrain;

    gyro = RobotMap.kgyro;

    PID = new PIDController(kP, kI, kD, gyro, this);

    //frontRight.configClearPositionOnQuadIdx(clearPositionOnQuadIdx, timeoutMs);


    PID.setInputRange(0, 360);
    PID.setOutputRange(-0.45, 0.45);
    PID.setAbsoluteTolerance(3);
    PID.setContinuous();
  }

  public void rotateToAngle(double angle) {
    PIDMotorSpeed = 0.0;

    PID.reset();
    PID.setPID(kP, kI, kD);
    PID.setSetpoint(angle);
    PID.enable();

    PIDMotorSpeed = 0.0;
  }

  public void driveStraight() {
    PIDMotorSpeed = RobotMap.kM_Speed;
    
    double startAngle = gyro.getAngle();

    PID.reset();
    PID.setPID(kP, kI, kD);
    PID.setSetpoint(startAngle);
    PID.enable();

  }

  @Override
  public void pidWrite(double rotateOutput) {
    // TODO Auto-generated method stub
    driveTrain.arcadeDrive(PIDMotorSpeed, rotateOutput);
  }

  public void move(XboxController controller) {
    driveTrain.arcadeDrive(controller.getY(Hand.kLeft), controller.getX(Hand.kRight));
  }

  public void driveForward() {
    driveTrain.arcadeDrive(0.45, 0);
  }

  public void stop() {
    driveTrain.arcadeDrive(0, 0);
    PID.disable();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveCommand());
  }

  

 
}
