/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Wrist extends PIDSubsystem {
  /**
   * Add your docs here.
   */
  private  Potentiometer pot;
  private  VictorSP victor;
  private static final double kP = 1.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double tolerance = 2.0;

  public Wrist() {
    // Intert a subsystem name and PID values here
    super("Wrist", kP , kI, kD);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    pot = new AnalogPotentiometer(1, 250, 0);
    victor = new VictorSP(RobotMap.wrist);
    setAbsoluteTolerance(tolerance);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return pot.get();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    victor.set(output);
  }
}
