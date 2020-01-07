/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.FlaccidWristMove;

/**
 * Add your docs here.
 */
public class FlaccidWrist extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Victor flaccidVictor;

  public FlaccidWrist() {
    flaccidVictor = new Victor(RobotMap.flaccidWristMotor);
  }

  public void move(XboxController controller) {
    SmartDashboard.putNumber("Left Trigger", controller.getTriggerAxis(Hand.kLeft));
    SmartDashboard.putNumber("Right Trigger", controller.getTriggerAxis(Hand.kRight));
    if (controller.getTriggerAxis(Hand.kLeft) > 0) {
      flaccidVictor.set(controller.getTriggerAxis(Hand.kLeft));
    } else if (controller.getTriggerAxis(Hand.kRight) > 0) {
      flaccidVictor.set(-controller.getTriggerAxis(Hand.kRight));
    } else {
      flaccidVictor.set(0);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new FlaccidWristMove());
  }
}
