/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrainPID extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  private static final double kP = 1.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static WPI_VictorSPX frontLeftVictor;
  private static WPI_VictorSPX backLeftVictor;
  private static WPI_VictorSPX frontRightVictor;
  private static WPI_VictorSPX backRightVictor;

  private static MecanumDrive driveTrainPID;

  private static ADXRS450_Gyro gyro;

  public DriveTrainPID() {
    // Intert a subsystem name and PID values here
    super("DriveTrainPID", kP, kI, kD);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    frontLeftVictor = RobotMap.frontLeftVictor;
    backLeftVictor = RobotMap.backLeftVictor;
    backRightVictor = RobotMap.backRightVictor;
    frontRightVictor = RobotMap.frontRightVictor;

    gyro = RobotMap.gyro;

    driveTrainPID = RobotMap.driveTrain;

    disable();
    setAbsoluteTolerance(3);
    getPIDController().setContinuous(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public void resetGyro() {
    gyro.reset();
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return gyro.getAngle();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    driveTrainPID.driveCartesian(0, 0, output);
  }
}
