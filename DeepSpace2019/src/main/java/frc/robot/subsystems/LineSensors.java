/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class LineSensors extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public DigitalInput distanceHatch;
  public DigitalInput distanceCargo;
  public DigitalInput frontLineSensor;

  public LineSensors () {
    distanceHatch = new DigitalInput(RobotMap.distanceHatch);
    distanceCargo = new DigitalInput(RobotMap.distanceCargo);
    frontLineSensor = new DigitalInput(RobotMap.frontLinesensor);
  }

  public void display() {
    SmartDashboard.putBoolean("Distance Hatch", distanceHatch.get());
    SmartDashboard.putBoolean("Distance Cargo", distanceCargo.get());
    SmartDashboard.putBoolean("Front Linesensor", frontLineSensor.get());
  }

  public boolean getHatchSensor() {
    return distanceHatch.get();
  }

  public boolean getCargoSensor() {
    return distanceCargo.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
