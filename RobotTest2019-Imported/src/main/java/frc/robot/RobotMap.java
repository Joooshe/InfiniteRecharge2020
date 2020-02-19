/*Joshua: When you map the port numbers on the system to certain variables
Ex: when you map pwm ports, CAN ports, DIO parts, or USB ports to certain numbers
This lets you keep track of the numbers easily by assigning them to names */

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
import edu.wpi.first.wpilibj.Spark;
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

//drive train motor port values


public static final int km_left = 0;
public static final int km_right = 1;


public static final Spark m_left = new Spark(km_left);
public static final Spark m_right = new Spark(km_right);

//ultra sonic sensor port values
//public static final int m_ultrasonic = 0;
public static final int potent = 2;
public static final int lightSensor = 3;

public static final SPI.Port kGyro = SPI.Port.kOnboardCS0;
public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro(kGyro);

//port values for wrist
public static final int pot = 0; //analog
public static final int wristTalon = 2; //pwm


}
