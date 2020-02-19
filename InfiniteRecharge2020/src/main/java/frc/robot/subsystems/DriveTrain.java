/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX frontLeft;
  private TalonSRX frontRight;
  private TalonSRX backLeft;
  private TalonSRX backRight;

  private SpeedControllerGroup leftSide;
  private SpeedControllerGroup rightSide;

  private DifferentialDrive driveTrain;

  public DriveTrain() {
    frontLeft = new TalonSRX(RobotMap.frontLeftDrive);
    frontRight = new TalonSRX(RobotMap.frontRightDrive);
    backLeft = new TalonSRX(RobotMap.backLeftDrive);
    backRight = new TalonSRX(RobotMap.backRightDrive);

    leftSide = new SpeedControllerGroup(frontLeft, frontRight);
    rightSide = new SpeedControllerGroup(backLeft, backRight);

    driveTrain = new DifferentialDrive(leftSide, rightSide);

  }

  public void move(XboxController controller) {
    driveTrain.arcadeDrive(controller.getY(Hand.kLeft), controller.getX(Hand.kRight));
  }

  public void stop() {
    driveTrain.arcadeDrive(0, 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveCommand());
  }
}
