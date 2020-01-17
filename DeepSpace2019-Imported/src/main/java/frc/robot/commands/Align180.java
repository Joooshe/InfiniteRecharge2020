/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Align180 extends Command {
  private double angle;

  public Align180() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    angle = Robot.driveTrain.getGyro().getAngle();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(angle >= 0 && angle <= 180) { //checks to see if it will take less time to turn left or right
      if(!(angle >= 170)) { //this code is based off the assumption the gyro goes from 0 to 360
      Robot.driveTrain.turnRight(RobotMap.fastTurnSpeed); //left and right might have to be changed in driveTrain
      } else {
        Robot.driveTrain.turnRight(RobotMap.slowTurnSpeed);
      }
    } else {
      if(!(angle <= 190)) {
      Robot.driveTrain.turnLeft(RobotMap.fastTurnSpeed);
      } else {
        Robot.driveTrain.turnLeft(RobotMap.slowTurnSpeed);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return angle <= 183 && angle >= 177;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
