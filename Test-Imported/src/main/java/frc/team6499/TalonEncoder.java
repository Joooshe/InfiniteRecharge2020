/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team6499;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

/**
 * Add your docs here.
 */
public class TalonEncoder implements PIDSource {

    public TalonEncoder() {

    }

    public PIDSourceType getPIDSourceType() {
        return new PIDSourceType();
    }

    public double pidGet() {
        return Robot.driveTrain.backLeftTalon.getSelectedSensorPosition();
    }

    public void setPIDSourceType() {

    }
}
