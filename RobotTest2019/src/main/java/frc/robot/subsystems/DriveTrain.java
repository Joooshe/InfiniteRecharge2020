/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Spark m_left;
  private Spark m_right;
  private DifferentialDrive drive;


  public DriveTrain () {
    m_left = new Spark(RobotMap.m_left);
    m_right = new Spark(RobotMap.m_right);

    drive = new DifferentialDrive(m_left, m_right);
    drive.setDeadband(0.05);
  }

  public void move(XboxController controller) {
    drive.arcadeDrive(controller.getY(Hand.kLeft), controller.getX(Hand.kRight));;
  }

  public void drive (double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  public void stop() {
    drive.arcadeDrive(0, 0);
  }



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveCommand());

  }
}
