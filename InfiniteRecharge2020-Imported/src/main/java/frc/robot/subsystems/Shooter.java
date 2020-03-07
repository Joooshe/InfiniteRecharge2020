/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Shooter extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static WPI_VictorSPX ml_Shooter;
  public static WPI_VictorSPX mr_Shooter;

  public static DoubleSolenoid sl_Shooter;
  public static DoubleSolenoid sr_Shooter;

  public Shooter() {
    ml_Shooter = RobotMap.ml_Shooter;
    mr_Shooter = RobotMap.ml_Shooter;

    sl_Shooter = RobotMap.sl_Shooter;
    sr_Shooter = RobotMap.sr_Shooter;
  }

  public void shoot(double speed) {
    ml_Shooter.set(speed);
    mr_Shooter.set(-speed);
  }

  public void stop() {
    ml_Shooter.set(0);
    mr_Shooter.set(0);
  }

  public void extend() {
    sl_Shooter.set(DoubleSolenoid.Value.kForward);
    sr_Shooter.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    sl_Shooter.set(DoubleSolenoid.Value.kReverse);
    sr_Shooter.set(DoubleSolenoid.Value.kReverse);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
