/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;

import frc.team6499.LimeLightPIDSource;
import frc.team6499.CustomPID;
import frc.team6499.GyroPIDConstants;
import frc.team6499.LimeLightPIDConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class DriveTrain extends PIDSubsystem{
  /**
   * Add your docs here.
   */
  public static final double kf = 3.0;

  //talonsrx constants
  public static final boolean kSensorPhase = true;
  public static final int kPIDError = 10;
  public static final int kPIDPrimary = 0;
  public static final int kPIDAux = 1;
  public static final int kTimeOutMs = 50;

  public static final double limeLightPIDErrorTX = 0.50;
  public static final double limeLightPIDErrorTY = 0.35;

  /*
  private static double kP_G = 1.0;
  private static double kI_G = 1.0;
  private static double kD_G = 1.0;

  
  //Good working PID values for setting motor position
  private static double kP3 = 1.0;
  private static double kI3 = 0.0;
  private static double kD3 = 10.0;
  private static double kF3 = 0.0;
  */
  private static final double kFConstant = 1.85;


  //for constant wheel velocity
  private static final double kP_E = 0.15;
  private static final double kI_E = 0.020;
  private static final double kD_E = 0.35;//3.50;
  private static final double kF_E = 0.185  * kFConstant;
  private static final int kIzone = 40;
  private static final int kRemote_0 = 0;


  //PID Constants as value holders
  private static double kP = 0.0;
  private static double kI = 0.0;
  private static double kD = 0.0;
  private static double kF = 0.0;

  public static PIDController secondaryPID;
  public static PIDSource magEncoder;

  private static WPI_VictorSPX frontLeftVictor;
  public  static WPI_TalonSRX backLeftMaster;
  private static WPI_VictorSPX frontRightVictor;
  private static WPI_TalonSRX backRightMaster;

  private static MecanumDrive drive;

  private static ADXRS450_Gyro gyro;
  public LimeLightPIDSource limeLight;
  public CustomPID customPID_TY;

  public static final double kDeadband = 0.17;

  public static final ShuffleboardTab tab = Shuffleboard.getTab("PID");
  
  public static final String[] ACCEPTABLE_PID_TYPES = {"gyro", "limelightTX", "limelightTY"};
  public static final String DEFAULT_PID_TYPE = "limelight";
  public static String pidType;

  public static NetworkTable table;
  
  public DriveTrain() {
    // Insert a subsystem name and PID values here
    super("DriveTrain", kP*kf, kI*kf, kD*kf);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    frontLeftVictor = RobotMap.frontLeftVictor;
    backLeftMaster = RobotMap.backLeftTalon;
    backRightMaster = RobotMap.backRightTalon;
    frontRightVictor = RobotMap.frontRightVictor;
  
    limeLight = new LimeLightPIDSource();
    //secondaryPID = new PIDController(kP, kI, kD, limeLight, this);

    customPID_TY = new CustomPID(LimeLightPIDConstants.kP_TY, LimeLightPIDConstants.kI_TY, LimeLightPIDConstants.kD_TY, limeLight);

    pidType = DEFAULT_PID_TYPE;
    configurePIDConstants();

    table = RobotMap.limeLightTable;//NetworkTableInstance.getDefault().getTable("limelight");
   

    //https://robotpy.readthedocs.io/projects/ctre/en/stable/api_mot.html#talonsrx

    gyro = RobotMap.gyro;

    drive = RobotMap.driveTrain;
    drive.setDeadband(kDeadband);
  }

  public void setPositionRotation(int targetPosRot) {
   backLeftMaster.set(ControlMode.Position, targetPosRot);
    //backLeftMaster.set(ControlMode.PercentOutput, 50);
    //Robot.driveTrain.backLeftMaster.set(0.8);

  }

  public void setEncoderSpeed(int targetSpeed) {
    backLeftMaster.set(ControlMode.Velocity, targetSpeed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveCommand());
  }

  public void displayMotorOutput() {
    SmartDashboard.putNumber("backLeftMaster Output", backLeftMaster.get());
    SmartDashboard.putNumber("backLRightMaster Output", backRightMaster.get());
    SmartDashboard.putNumber("frontLeftVictor Output", frontLeftVictor.get());
    SmartDashboard.putNumber("frontRightVictor Output", frontRightVictor.get());

    SmartDashboard.putNumber("Controller Output", Robot.m_oi.getController().getX(Hand.kRight));
  }

  public void configCustomPID_TY() {
    customPID_TY.setSetPoint(0.0);
    customPID_TY.setInputRange(50, -50);
    customPID_TY.setOutputRange(0.35, -0.35);
    customPID_TY.setAbsoluteTolerance(0.40);
  }

  public boolean useCustomPID_TY() {
    customPID_TY.calculatePID();
    if(limeLight.getTV() == 1) {
      drive.driveCartesian(0, customPID_TY.getOutput(), 0);
    }
    return customPID_TY.onTarget();
  }

  public boolean setPIDType(String type) {
    boolean acceptable = false;

    for(String i: ACCEPTABLE_PID_TYPES) {
      if(type.equals(i)) {
        acceptable = true;
        this.pidType = type;
      }
    }

    return acceptable;
  }

  public void configPIDGyro(double setPoint) {
    disable();
    setInputRange(0, 360);
    getPIDController().setContinuous(true);
    setOutputRange(-0.45, 0.45);
    setAbsoluteTolerance(2);
    setPIDType("gyro");
    configurePIDConstants();
    setSetpoint(setPoint);
  }

  public void configPIDLimeLight_TX() {
    disable();
    setInputRange(-50, 50); // might need to change these
    getPIDController().setContinuous(false);
    setOutputRange(-0.25, 0.25);
    setAbsoluteTolerance(limeLightPIDErrorTX);
    setPIDType("limelightTX");
    configurePIDConstants();
    setSetpoint(0.0);
  }

  public void configPIDLimeLight_TY() {
    disable();
    setInputRange(-50, 50); // might need to change these
    getPIDController().setContinuous(false);
    setOutputRange(-0.35, 0.35);
    setAbsoluteTolerance(limeLightPIDErrorTY);
    setPIDType("limelightTY");
    configurePIDConstants();
    setSetpoint(0.0);
  }

  public void configurePIDConstants() {
    if(pidType.equals("limelightTX")) {
      this.kP = LimeLightPIDConstants.kP_TX;
      this.kI = LimeLightPIDConstants.kI_TX;
      this.kD = LimeLightPIDConstants.kD_TX;
      this.kF = LimeLightPIDConstants.kF_TX;
    } else if(pidType.equals("limelightTY")) {
      this.kP = LimeLightPIDConstants.kP_TY;
      this.kI = LimeLightPIDConstants.kI_TY;
      this.kD = LimeLightPIDConstants.kD_TY;
      this.kF = LimeLightPIDConstants.kF_TY;
    } else if(pidType.equals("gyro")) {
      this.kP = GyroPIDConstants.kP;
      this.kI = GyroPIDConstants.kI;
      this.kD = GyroPIDConstants.kD;
      this.kF = GyroPIDConstants.kF;
    }
    this.getPIDController().setP(kP);
    this.getPIDController().setI(kI);
    this.getPIDController().setD(kD);
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public void stop() {
    drive.driveCartesian(0, 0, 0);
    disable();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }


  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    if(pidType.equals("limelightTX")) {
      return limeLight.getTXForPID();
      
    } else if(pidType.equals("limelightTY")) {
      return limeLight.getTY();

    } else if(pidType.equals("gyro")) {
      return gyro.getAngle();
      
    } else {
      return 0.0;
    }
  }

  public double visionTurning(double setValue) {
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double v = tv.getDouble(0.0);
    double area = ta.getDouble(0.0);
  
    double kP4 = 1.0;
    double error = setValue - x;
    
    
     if(v == 0) {
       drive.driveCartesian(0, 0, RobotMap.slowTurnSpeed);
     } else {
      drive.driveCartesian(0, 0, 0.3*error);
     }

    return error;
    //max distance 9 1/2 feet
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
    SmartDashboard.putNumber("PID Output", output);
    //wheels are 6 inch
    if(pidType.equals("limelightTX")) {
       //drive.driveCartesian(0, 0, -output);
       customPID_TY.calculatePID();
       frontLeftVictor.pidWrite(-output); //+ customPID_TY.getOutput());
       backLeftMaster.pidWrite(-output); //+ customPID_TY.getOutput());
       frontRightVictor.pidWrite(-output); //+ customPID_TY.getOutput());
       backRightMaster.pidWrite(-output); // + customPID_TY.getOutput());
       
      /*
        frontLeftVictor.set(ControlMode.PercentOutput, -output);
        backLeftMaster.set(ControlMode.PercentOutput, -output);
        frontRightVictor.set(ControlMode.PercentOutput, -output);
        backRightMaster.set(ControlMode.PercentOutput, -output);
      */
    } else if(pidType.equals("limelightTY")) {
       drive.driveCartesian(0, output, 0);
      
      /*
        frontLeftVictor.pidWrite(output);
        backLeftMaster.pidWrite(output);
        frontRightVictor.pidWrite(output);
        backRightMaster.pidWrite(output);
        */


    } else if(pidType.equals("gyro")) {
       drive.driveCartesian(0, 0, output);
      
      /*
        frontLeftVictor.pidWrite(output);
        backLeftMaster.pidWrite(output);
        frontRightVictor.pidWrite(output);
        backRightMaster.pidWrite(output);
        */

    } else {
      drive.driveCartesian(0, 0, 0);

      /*
        frontLeftVictor.pidWrite(0);
        backLeftMaster.pidWrite(0);
        frontRightVictor.pidWrite(0);
        backRightMaster.pidWrite(0);
      */
    }

    SmartDashboard.putNumber("Output", output);
  }

  /*
  public void pidWrite(double output) {
    if(pidType.equals("limelightTX")) {
      drive.driveCartesian(ySpeed, xSpeed, output);;

    } else if(pidType.equals("limelightTY")) {
      limeLight.setReturnType("ty");

    } else if(pidType.equals("gyro")) {
      return gyro.getAngle();
      
    } else {
      return 0.0;
    }
  }
  */

  public void move(XboxController controller) {

    //TEST
    //SmartDashboard.putNumber("Potentiometer", potent.get());

    if (Robot.isForward) {
      // first way we drove drive.driveCartesian(controller.getX(Hand.kLeft), -controller.getY(Hand.kLeft), controller.getX(Hand.kRight), gyro.getAngle());
      drive.driveCartesian(controller.getX(Hand.kLeft), -controller.getY(Hand.kLeft), controller.getX(Hand.kRight), gyro.getAngle());
    } else {
      // first way we drove drive.driveCartesian(-controller.getX(Hand.kLeft), +controller.getY(Hand.kLeft), controller.getX(Hand.kRight), gyro.getAngle());    
      //switches the front side of the robot
      //drive.driveCartesian(ySpeed, xSpeed, zRotation);
      drive.driveCartesian(-getControllerLeftX(controller), getControllerLeftY(controller), getControllerRightX(controller), gyro.getAngle());
    }
  }

  public double getControllerLeftX(XboxController controller) {
    if(Math.abs(controller.getX(Hand.kLeft)) > kDeadband) {
      return controller.getX(Hand.kLeft);
    } else {
      return 0;
    }
  }

  public double getControllerRightX(XboxController controller) {
    if(Math.abs(controller.getX(Hand.kRight)) > kDeadband) {
      return controller.getX(Hand.kRight);
    } else {
      return 0;
    }
  }
  public double getControllerLeftY(XboxController controller) {
    if(Math.abs(controller.getY(Hand.kLeft)) > kDeadband) {
      return controller.getY(Hand.kLeft);
    } else {
      return 0;
    }
  }


  public double decimalPlace(double num, int place) {
    return (((double)((int)(num*place)))/place);
  }

  public void reset() {
    gyro.reset();
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
 /*
      NetworkTableEntry P = Shuffleboard.getTab("PID") 
      .add("P", 1)
      .getEntry();
  
      NetworkTableEntry I = Shuffleboard.getTab("PID")
      .add("I", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
      .getEntry();
  
      NetworkTableEntry D = Shuffleboard.getTab("PID")
      .add("D", 0.5)
      .getEntry();
  
  
      kP = P.getDouble(1.0);
      kI = I.getDouble(1.0);
      kD = D.getDouble(1.0);
      */

 public void configureTalons() {
    //backRightMaster.set(ControlMode.PercentOutput, 0);
    //backLeftMaster.set(ControlMode.PercentOutput, 0);

    frontLeftVictor.follow(backLeftMaster);
    frontRightVictor.follow(backRightMaster);

    backLeftMaster.configFactoryDefault();
    backRightMaster.configFactoryDefault();

    backLeftMaster.setNeutralMode(NeutralMode.Brake);
    backRightMaster.setNeutralMode(NeutralMode.Brake);

    backLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                                kPIDPrimary, 
                                                kTimeOutMs);
    backLeftMaster.setSensorPhase(kSensorPhase);

    backRightMaster.configRemoteFeedbackFilter(backLeftMaster.getDeviceID(),
                                              RemoteSensorSource.TalonSRX_SelectedSensor,
                                              kRemote_0,
                                              kTimeOutMs);
                                              
    backRightMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, kTimeOutMs);
    backRightMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, kTimeOutMs);
    
    backRightMaster.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, kTimeOutMs);
    backRightMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, kTimeOutMs);

    //configure back right talon to use the sensor sum feedback device that has been calculated from the configSensorTerm 
    //method
    backRightMaster.configSelectedFeedbackSensor( FeedbackDevice.SensorSum, kPIDPrimary, kTimeOutMs); 
    backRightMaster.configSelectedFeedbackCoefficient(0.5, kPIDPrimary, kTimeOutMs);
    

    backRightMaster.configSelectedFeedbackSensor( FeedbackDevice.SensorDifference, kPIDPrimary, kTimeOutMs);
    backRightMaster.configSelectedFeedbackCoefficient(1, kPIDPrimary, kTimeOutMs);

    backLeftMaster.setInverted(false);
    backLeftMaster.setSensorPhase(false);
    backRightMaster.setInverted(true);
    backRightMaster.setSensorPhase(false);

    backRightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 50, kTimeOutMs);
    backRightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 50, kTimeOutMs);
    backRightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 50, kTimeOutMs);
    backRightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 50, kTimeOutMs);
    backLeftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 50, kTimeOutMs);

    //config peak and lowest outputs for motor
    backLeftMaster.configNominalOutputForward(0, kTimeOutMs); 
    backLeftMaster.configNominalOutputReverse(0, kTimeOutMs);
    backLeftMaster.configPeakOutputForward(1, kTimeOutMs);
    backLeftMaster.configPeakOutputReverse(-1, kTimeOutMs);
    
    //config allowable error

    backLeftMaster.configAllowableClosedloopError(kPIDPrimary, kPIDError, kTimeOutMs);

    backLeftMaster.config_kF(kPIDPrimary, kF_E, kTimeOutMs);
    backLeftMaster.config_kP(kPIDPrimary, kP_E, kTimeOutMs);
    backLeftMaster.config_kI(kPIDPrimary, kI_E, kTimeOutMs);
    backLeftMaster.config_kD(kPIDPrimary, kD_E, kTimeOutMs);
    
    backLeftMaster.config_IntegralZone(kPIDPrimary, kIzone, kTimeOutMs);
 }
}
