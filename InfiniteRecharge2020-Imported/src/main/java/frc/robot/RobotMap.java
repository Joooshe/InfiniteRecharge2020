/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;


  //PORT CONSTANTS
  public static final int frontLeftDrive = 1;
  public static final int frontRightDrive = 3;
  public static final int backLeftDrive = 2;
  public static final int backRightDrive = 4;

  public static final int klShooterMotor = 5;
  public static final int krShooterMotor = 6;
  public static final int klShooterSolenoid_f = 10;
  public static final int klShooterSolenoid_r = 11;
  public static final int krShooterSolenoid_f = 12;
  public static final int krShooterSolenoid_r = 13;

  public static final int kTread = 7;

  public static final int km_Elevator = 8;

  public static final int kIntakeMotor = 9;
  public static final int kIntakeSolenoid_f = 0;
  public static final int kIntakeSolenoid_r = 1;

  //MOTOR CONSTANTS
  public static final WPI_TalonSRX kfrontLeft = new WPI_TalonSRX(RobotMap.frontLeftDrive);
  public static final WPI_TalonSRX kfrontRight = new WPI_TalonSRX(RobotMap.frontRightDrive);
  public static final WPI_TalonSRX kbackLeft = new WPI_TalonSRX(RobotMap.backLeftDrive);
  public static final WPI_TalonSRX kbackRight = new WPI_TalonSRX(RobotMap.backRightDrive);

  public static final WPI_VictorSPX ml_Shooter = new WPI_VictorSPX(klShooterMotor);
  public static final WPI_VictorSPX mr_Shooter = new WPI_VictorSPX(krShooterMotor);
  public static final DoubleSolenoid sl_Shooter = new DoubleSolenoid(klShooterSolenoid_f, klShooterSolenoid_r);
  public static final DoubleSolenoid sr_Shooter = new DoubleSolenoid(krShooterSolenoid_f, krShooterSolenoid_r);


  public static final WPI_VictorSPX m_Tread = new WPI_VictorSPX(kTread);

  public static final WPI_TalonSRX m_Elevator = new WPI_TalonSRX(km_Elevator);

  public static final WPI_VictorSPX m_intake = new WPI_VictorSPX(kIntakeMotor);
  public static final DoubleSolenoid s_intake = new DoubleSolenoid(kIntakeSolenoid_f, kIntakeSolenoid_r);

  public static final SpeedControllerGroup kLeftMotorGroup = new SpeedControllerGroup(kfrontLeft, kbackLeft);
  public static final SpeedControllerGroup kRightMotorGroup = new SpeedControllerGroup(kfrontRight, kbackRight);

  public static final DifferentialDrive kdriveTrain = new DifferentialDrive(kLeftMotorGroup, kRightMotorGroup);

  public static final ADXRS450_Gyro kgyro = new ADXRS450_Gyro();

  public static final double kM_Speed = 0.45;

}
