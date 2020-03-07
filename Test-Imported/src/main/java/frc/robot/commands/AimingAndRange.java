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

public class AimingAndRange extends Command {
  private int count;
  
  public AimingAndRange() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    count = 0;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.configCustomPID_TY();
    Robot.driveTrain.configPIDLimeLight_TX();
    Robot.driveTrain.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.configCustomPID_TY();
    Robot.driveTrain.useCustomPID_TY();
    System.out.println(Robot.driveTrain.customPID_TY.getOutput());
    System.out.println(Robot.driveTrain.customPID_TY.onTarget());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Robot.driveTrain.onTarget() && Robot.driveTrain.customPID_TY.onTarget()) {
      count++;
    } else {
      count = 0;
    }

    SmartDashboard.putNumber("Count", count);

    boolean isFinished = count >= 7 && Robot.driveTrain.limeLight.getTV() == 1 && Robot.driveTrain.customPID_TY.onTarget() && Robot.driveTrain.onTarget();
    System.out.println("Is finished: " + isFinished);
    
    return isFinished; //count >= 7 && Robot.driveTrain.onTarget() && Robot.driveTrain.limeLight.getTV() == 1 && Robot.driveTrain.customPID_TY.onTarget();
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
