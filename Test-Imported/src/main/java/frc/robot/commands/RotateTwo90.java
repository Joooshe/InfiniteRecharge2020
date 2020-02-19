/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class RotateTwo90 extends Command {
  
  private int count = 0;

  public RotateTwo90() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.enable();
    Robot.driveTrain.setSetpoint(90.0);
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    if (Robot.driveTrain.onTarget()) {
      count++;
    } else {
      count = 0;
    }

    SmartDashboard.putNumber("Count", count);

    return count >= 6 && Robot.driveTrain.onTarget();
    

    //return Robot.driveTrain.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.getPIDController().disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
