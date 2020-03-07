/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team6499;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class LimeLightPIDSource implements PIDSource {

    public static PIDSourceType limeLightVision;
    public static NetworkTable table;
    public static String returnType;

    public static final String[] ACCEPTABLE_RETURN_TYPES = {"tv", "tx", "ty", "ta", "ts", "tl", "tshort", "tlong", "thor", "tvert", "getpipe", "camtran"};
    public static final String DEFAULT_RETURN_TYPE = "tx";

    private double lastT;
    private double currentT;

    public LimeLightPIDSource() {
        table = RobotMap.limeLightTable;
        returnType = DEFAULT_RETURN_TYPE;
        currentT = table.getEntry(returnType).getDouble(0.0);
    }

    public PIDSourceType getPIDSourceType() {
        return limeLightVision;
    }

    public boolean setReturnType(String returnType) {
        boolean acceptableValue = false;

        for(String i : ACCEPTABLE_RETURN_TYPES) {
            if(returnType.equals(i)) {
                acceptableValue = true;
                this.returnType = returnType;
            }
        }

        return acceptableValue;
    }

    public void configT() {
        if(getTV() == 1) {
            lastT = currentT;
            currentT = table.getEntry(returnType).getDouble(0.0);
        }
    }

    public String getReturnType() {
        return this.returnType;
    }

    public double pidGet() {
        return getT(returnType);
    }

    public double getT(String returnType) {
            if(returnType.equals("tv")) {
                return getTV();
            } else if(returnType.equals("tx")) {
                return getTXForPID();
            } else if(returnType.equals("ty")) {
                return getTY();
            } else if(returnType.equals("ts")) {
                return getTS();
            } else {
                return 0.0;
            }
    }

    public void setPIDSourceType(PIDSourceType pidSource) {
        limeLightVision = pidSource;
    }

    public double getTX() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getTXForPID() {
        if(getTV() == 0) {
            return 30.0;

        } else {
           return getTX(); 

        }
    }
    
    public double getTY() {
        return table.getEntry("ty").getDouble(0.0);
    }

    public double getTV() {
        return table.getEntry("tv").getDouble(0.0);
    }

    public double getTA() {
        return table.getEntry("ta").getDouble(0.0);
    }

    public double getTS() {
        return table.getEntry("ts").getDouble(0.0);
    }

    public void turnOffLight() {
        table.getEntry("ledMode").setNumber(1);
    }

    public void turnOnLight() {
        table.getEntry("ledMode").setNumber(3);
    }

}
