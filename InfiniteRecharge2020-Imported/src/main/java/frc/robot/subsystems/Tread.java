/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Tread extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static final double kSlowIntakeSpeed = 0.3;
  public static final double kFastIntakeSpeed = 0.6;

  public static WPI_VictorSPX m_Tread;

  public Tread() {
    m_Tread = RobotMap.m_Tread;
  }

  public void slowIntake() {
    m_Tread.set(kSlowIntakeSpeed);
  }

  public void fastIntake() {
    m_Tread.set(kFastIntakeSpeed);
  }

  public void reverseIntake() {
    m_Tread.set(-kSlowIntakeSpeed);
  }

  public void setSpeed(double speed) {
    m_Tread.set(speed);
  }

  public void stop() {
    m_Tread.set(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
