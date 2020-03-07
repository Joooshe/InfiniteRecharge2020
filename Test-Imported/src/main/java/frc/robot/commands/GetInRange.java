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

public class GetInRange extends Command {
  private double distanceError;
  private int count;

  public GetInRange() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);

    //distanceError = RobotMap.limeLightTable.getEntry("ty").getDouble(0.0);

    count = 0;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.configPIDLimeLight_TY();
    Robot.driveTrain.enable();
  }
  

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println(Robot.driveTrain.limeLight.getTY());
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

    return count >= 6 && Robot.driveTrain.onTarget() && Robot.driveTrain.limeLight.getTV() == 1;
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
