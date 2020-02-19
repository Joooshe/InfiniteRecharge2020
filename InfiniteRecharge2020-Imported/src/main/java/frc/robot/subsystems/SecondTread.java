/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SecondTread extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static final double kSlowIntakeSpeed = 0.3;
  public static final double kFastIntakeSpeed = 0.6;

  public static WPI_TalonSRX m1;
  public static WPI_TalonSRX m2;

  public SecondTread() {
    m1 = RobotMap.m1SecondTread;
    m2 = RobotMap.m2SecondTread;
  }

  public void slowIntake() {
    m1.set(kSlowIntakeSpeed);
    m2.set(kSlowIntakeSpeed);
  }

  public void fastIntake() {
    m1.set(kFastIntakeSpeed);
    m2.set(kFastIntakeSpeed);
  }

  public void reverseIntake() {
    m1.set(-kSlowIntakeSpeed);
    m2.set(-kSlowIntakeSpeed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
