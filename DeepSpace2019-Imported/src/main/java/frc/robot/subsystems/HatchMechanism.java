/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.SolenoidOff;

/**
 * Add your docs here.
 */
public class HatchMechanism extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public DoubleSolenoid solenoid1;
  //public DoubleSolenoid solenoid2;

  public HatchMechanism() {
    solenoid1 = new DoubleSolenoid(RobotMap.solenoid1, RobotMap.solenoid2);
    //solenoid2 = new DoubleSolenoid(RobotMap.solenoid1, RobotMap.solenoid2);

  }

  public void extend() {
    solenoid1.set(DoubleSolenoid.Value.kForward);
    //solenoid2.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    solenoid1.set(DoubleSolenoid.Value.kReverse);
    //solenoid2.set(DoubleSolenoid.Value.kReverse);
  }

  public void stop() {
    solenoid1.set(DoubleSolenoid.Value.kOff);
    //solenoid2.set(DoubleSolenoid.Value.kOff);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
