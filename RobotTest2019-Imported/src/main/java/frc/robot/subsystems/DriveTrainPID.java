/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

  private static Spark m_left;
  private static Spark m_right;

  private static DifferentialDrive driveTrain;

  private static ADXRS450_Gyro gyro;
  public DriveTrainPID() {
    // Intert a subsystem name and PID values here
    super("DriveTrainPID", kP, kI, kD);

    m_left = (RobotMap.m_left);
    m_right = (RobotMap.m_right);

    gyro = RobotMap.gyro;
    
    driveTrain = new DifferentialDrive(m_left, m_right);

    setAbsoluteTolerance(0.5);
    getPIDController().setContinuous(false);

    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }
  
  public void resetGyro() {
    gyro.reset();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
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

    driveTrain.arcadeDrive(0, output);

  }
}
