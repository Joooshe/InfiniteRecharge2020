/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.Command;

public class PidWithNoPID extends Command {
  private double kP = 1.0;
  private double kI = 0.0;
  private double kD = 0.0;

  private double integral;
  private double previous_error;
  private double setpoint;
  private double error;
  private double rcw;

  private ADXRS450_Gyro gyro;
  private DriveTrain driveTrain;

  public PidWithNoPID() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    //requires(Robot.driveTrain);

    gyro = Robot.getGyro();
    driveTrain = Robot.driveTrain;

    setpoint = 30;

  }

  public void setSetPoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public void PID() {
    error = setpoint - gyro.getAngle();
    integral += (error*.02);
    rcw = kP*error + kI * integral;
  }



  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    PID();
    driveTrain.arcadeDrive((double)0, rcw);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return gyro.getAngle()-error < 1;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
