/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


public class DriveToDistance extends Command {
  
  private double holdDistance;
  private double currentDistance;

  public DriveToDistance(double holdDistance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.holdDistance = holdDistance;

    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentDistance = Robot.m_ultrasonic.getValue() * Robot.kValueToInches;

    double robotSpeed = (holdDistance - currentDistance) * Robot.kP;

    Robot.driveTrain.drive(robotSpeed, 0);


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //checks to see if robot is within desire range including .25 inch error
    if(Math.abs(holdDistance - currentDistance) < 0.25) {
      return true;
    } else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
