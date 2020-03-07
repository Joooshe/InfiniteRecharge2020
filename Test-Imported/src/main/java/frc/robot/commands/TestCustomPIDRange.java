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
import frc.team6499.CustomPID;

public class TestCustomPIDRange extends Command {
  private int count = 0;

  public TestCustomPIDRange() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.configCustomPID_TY();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.useCustomPID_TY();
    SmartDashboard.putNumber("CustomPID Output", Robot.driveTrain.customPID_TY.getOutput());
    System.out.println(Robot.driveTrain.customPID_TY.getOutput());
    System.out.println(Robot.driveTrain.customPID_TY.onTarget());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    //return false;
    
    
    if (Robot.driveTrain.customPID_TY.onTarget()) {
      count++;
    } else {
      count = 0;
    }

    SmartDashboard.putNumber("Count", count);

    boolean isFinished = count >= 7 && Robot.driveTrain.limeLight.getTV() == 1 && Robot.driveTrain.customPID_TY.onTarget();
    System.out.println("Is finished: " + isFinished);
    
    return isFinished; //count >= 7 && Robot.driveTrain.limeLight.getTV() == 1 && Robot.driveTrain.customPID_TY.onTarget();
    
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
    end();
  }
}
