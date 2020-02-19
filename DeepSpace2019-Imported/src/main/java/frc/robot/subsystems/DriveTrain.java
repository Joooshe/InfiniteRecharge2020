/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;


/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_VictorSPX frontLeftVictor;
  private WPI_VictorSPX backLeftVictor;
  private WPI_VictorSPX frontRightVictor;
  private WPI_VictorSPX backRightVictor;
  private MecanumDrive drive;
  private AHRS gyro;

  //TESTING
  private Potentiometer potent;

  public DriveTrain() {
    frontLeftVictor = RobotMap.frontLeftVictor;
    backLeftVictor = RobotMap.backLeftVictor;
    backRightVictor = RobotMap.backRightVictor;
    frontRightVictor = RobotMap.frontRightVictor;

    gyro = new AHRS(Port.kUSB);
    drive = RobotMap.driveTrain;//new MecanumDrive(frontLeftVictor, backLeftVictor, frontRightVictor, backRightVictor);
    drive.setDeadband(0.1);

    //TESTING
    potent = new AnalogPotentiometer(RobotMap.pot, 250, 0);

  }
  
  public void move(XboxController controller) {

    //TEST
    //SmartDashboard.putNumber("Potentiometer", potent.get());

    if (controller.getAButton()) {
      //gyro.reset();
      //drive.driveCartesian(1, 0, 0);
    }
    if (controller.getBButton()) {
      //SmartDashboard.putString("lkhfalhflhkahjlasflhlhkaflhkaklaskjlfasklasfljk", "bjaskjasjhlasdlhjdsajhashjkhaksjuiqiuqkhebnbn");
    }
    if (Robot.isForward) {
      // first way we drove drive.driveCartesian(controller.getX(Hand.kLeft), -controller.getY(Hand.kLeft), controller.getX(Hand.kRight), gyro.getAngle());
      drive.driveCartesian(controller.getX(Hand.kLeft), -controller.getY(Hand.kLeft), controller.getX(Hand.kRight), gyro.getAngle());
    } else {
      // first way we drove drive.driveCartesian(-controller.getX(Hand.kLeft), +controller.getY(Hand.kLeft), controller.getX(Hand.kRight), gyro.getAngle());    
      //switches the front side of the robot
      drive.driveCartesian(-controller.getX(Hand.kLeft), controller.getY(Hand.kLeft), controller.getX(Hand.kRight), gyro.getAngle());
    }
  }


  public double decimalPlace(double num, int place) {
    return (((double)((int)(num*place)))/place);
  }

  public void reset() {
    gyro.reset();
  }

  public void stop() {
    drive.stopMotor();
  }

  public void forward() {
    if (Robot.isForward) {
      drive.driveCartesian(0, 1, 0);
    } else {
      drive.driveCartesian(0, -1, 0);
    }
  }

  public void forwardAtSpeed(double speed) {
    if (Robot.isForward) {
      drive.driveCartesian(0, speed, 0);
    } else {
      drive.driveCartesian(0, -speed, 0);
    }
  }

  public void turnLeft(double turnSpeed) {
    drive.driveCartesian(0, 0, turnSpeed);
  }

  public void turnRight(double turnSpeed) {
    drive.driveCartesian(0, 0, -turnSpeed);
  }

  public void backward() {
     if (Robot.isForward) {
      drive.driveCartesian(1, 0, 0);
    } else { 
      drive.driveCartesian(-1, 0, 0);
    }
  }

  public void backwardAtSpeed(double speed) {
    if (Robot.isForward) {
     drive.driveCartesian(0, -speed, 0); //the negative and positive signs may need to be changed depending on which way is forward and backwards
   } else { 
     drive.driveCartesian(0, speed, 0);
   }
 }

  public AHRS getGyro(){
    return gyro;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveCommand());
  }
}
