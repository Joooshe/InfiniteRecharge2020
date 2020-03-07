/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_VictorSPX m_intake;

  public DoubleSolenoid s_intake;

  public Intake() {
    m_intake = RobotMap.m_intake;

    s_intake = RobotMap.s_intake;
  }

  public void intake(double speed) {
    m_intake.set(speed);
  }

  public void release(double speed) {
    m_intake.set(-speed);
  }

  public void extend() {
    s_intake.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    s_intake.set(DoubleSolenoid.Value.kReverse);
  }

  public void stop() {
    m_intake.set(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
