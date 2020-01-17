/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.*;
import frc.robot.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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

  private WPI_TalonSRX frontLeft;
  private WPI_TalonSRX frontRight;
  private WPI_TalonSRX backLeft;
  private WPI_TalonSRX backRight;

  private SpeedControllerGroup leftSide;
  private SpeedControllerGroup rightSide;

  private DifferentialDrive driveTrain;

  public DriveTrain() {
    
    frontLeft = new WPI_TalonSRX(RobotMap.frontLeftDrive);
    frontRight = new WPI_TalonSRX(RobotMap.frontRightDrive);
    backLeft = new WPI_TalonSRX(RobotMap.backLeftDrive);
    backRight = new WPI_TalonSRX(RobotMap.backRightDrive);

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
