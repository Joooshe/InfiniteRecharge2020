/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.CargoOff;

/**
 * Add your docs here.
 */
public class CargoShooter extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private Spark cargoMotor;

  public CargoShooter() {
    cargoMotor = new Spark(RobotMap.cargoMotor);
  }

  public void shoot() {
    cargoMotor.setSpeed(RobotMap.cargoMotorSpeed);
  }

  public void takeIn() {
    cargoMotor.setSpeed(-RobotMap.cargoMotorSpeed);    
  }

  public void stop() {
    cargoMotor.setSpeed(RobotMap.cargoMotorOff);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
