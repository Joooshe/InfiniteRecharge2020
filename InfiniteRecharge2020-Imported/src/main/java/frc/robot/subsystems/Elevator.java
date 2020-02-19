/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorCommand;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem implements PIDOutput{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  //private WPI_TalonSRX m_Elevator;
  private WPI_VictorSPX m_Elevator;


  private final PIDController PID;

  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  private Encoder encoder;

  public Elevator() {
    m_Elevator = RobotMap.m_ElevatorVictor;

    encoder = new Encoder(5, 7);

    PID = new PIDController(kP, kI, kD, encoder, this);
  }

  //PID FUNCTIONS
  public void liftTo(double height) {
    PID.reset();
    PID.setPID(kP, kI, kD);
    PID.setSetpoint(height);
    PID.enable();
  }

  public void pidWrite(double rotateOutput) {
    m_Elevator.set(rotateOutput);
  }

  //NON-PID FUNCTIONS
  public void up() {
    m_Elevator.set(0.3);
  }

  public void down() {
    m_Elevator.set(-0.3);
  }

  public void up(double speed) {
    m_Elevator.set(speed);
  }

  public void down(double speed) {
    m_Elevator.set(-speed);
  }

  public void stop() {
    m_Elevator.set(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ElevatorCommand());
  }
}
