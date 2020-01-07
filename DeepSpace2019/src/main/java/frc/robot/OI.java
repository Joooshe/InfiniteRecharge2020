/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import frc.robot.commands.SolenoidRetract;
//import frc.robot.commands.SolenoidExtend;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);
  private XboxController controller;
  private Button bA;
  private Button bB;
  private Button bX;
  private Button bY;
  private Button leftPaddle;
  private Button bLeftBumper;
  private Button bRightBumper;
  private Button rightPaddle;

  public static final int buttonA = 1;
  public static final int buttonB = 2;
  public static final int buttonX = 3;
  public static final int buttonY = 4;
  public static final int buttonLeftBumper = 5;
  public static final int buttonRightBumper = 6;
  public static final int buttonLeftPaddle = 9;

  public OI() {
    controller = new XboxController(0);
    bA = new JoystickButton(controller, buttonA);
    bB = new JoystickButton(controller, buttonB);
    bX = new JoystickButton(controller, buttonX);
    bY = new JoystickButton(controller, buttonY);
    bLeftBumper = new JoystickButton(controller , buttonLeftBumper);
    bRightBumper = new JoystickButton(controller , buttonRightBumper);
    leftPaddle = new JoystickButton(controller, buttonLeftPaddle);

    bY.whenPressed(new HatchPickup());
    bX.whenPressed(new Align270());
    bB.whenPressed(new Align90());
    bA.whenPressed(new SwitchDirection());
    
    bRightBumper.whenPressed(new CargoShoot());
    bRightBumper.whenReleased(new CargoOff());
    bLeftBumper.whenPressed(new CargoTakeIn());
    bLeftBumper.whenReleased(new CargoOff());

    leftPaddle.whenPressed(new ToggleSolenoid());
  }

  public XboxController getController() {
    return controller;
  }
  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
