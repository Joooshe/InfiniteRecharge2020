/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
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
  private WPI_TalonSRX m_Elevator;


  private final PIDController PID;

  public static final boolean kSensorPhase = true;
  public static final int kPIDError = 409;
  public static final int kPIDPrimary = 0;
  public static final int kPIDAux = 1;
  public static final int kTimeOutMs = 50;

  private static final double kFConstant = 1.85;

  private static final double kP = 0.15;
  private static final double kI = 0.020;
  private static final double kD = 0.35;//3.50;
  private static final double kF = 0.185  * kFConstant;
  private static final int kIzone = 40;
  private static final int kRemote_0 = 0;
  public static final double kPeakNominal = 0.5;


  private Encoder encoder;

  public Elevator() {
    m_Elevator = RobotMap.m_Elevator;
    configureTalons();


    encoder = new Encoder(5, 7);
    PID = new PIDController(kP, kI, kD, kF, encoder, this);
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

  public void configureTalons() {
    //m_Elevatorm_Elevator.set(ControlMode.PercentOutput, 0);
    //m_Elevator.set(ControlMode.PercentOutput, 0);
    
   m_Elevator.configFactoryDefault();

  m_Elevator.setNeutralMode(NeutralMode.Brake);
  m_Elevator.set(ControlMode.PercentOutput, 0);

  m_Elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                              kPIDPrimary, 
                                              kTimeOutMs);
  m_Elevator.setSensorPhase(kSensorPhase);

  //configure back right talon to use the sensor sum feedback device that has been calculated from the configSensorTerm 
  //method

   m_Elevator.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 50, kTimeOutMs);
   m_Elevator.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 50, kTimeOutMs);
   m_Elevator.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 50, kTimeOutMs);
   m_Elevator.setStatusFramePeriod(StatusFrame.Status_10_Targets, 50, kTimeOutMs);

    //config peak and lowest outputs for motor
    m_Elevator.configNominalOutputForward(0, kTimeOutMs); 
    m_Elevator.configNominalOutputReverse(0, kTimeOutMs);
    m_Elevator.configPeakOutputForward(kPeakNominal, kTimeOutMs);
    m_Elevator.configPeakOutputReverse(-kPeakNominal, kTimeOutMs);
    
    //config allowable error

    m_Elevator.configAllowableClosedloopError(kPIDPrimary, kPIDError, kTimeOutMs);

    m_Elevator.config_kF(kPIDPrimary, kF, kTimeOutMs);
    m_Elevator.config_kP(kPIDPrimary, kP, kTimeOutMs);
    m_Elevator.config_kI(kPIDPrimary, kI, kTimeOutMs);
    m_Elevator.config_kD(kPIDPrimary, kD, kTimeOutMs);
    
    m_Elevator.config_IntegralZone(kPIDPrimary, kIzone, kTimeOutMs);
 }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ElevatorCommand());
  }
}
