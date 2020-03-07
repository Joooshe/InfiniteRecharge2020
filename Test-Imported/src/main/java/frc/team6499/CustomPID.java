package frc.team6499;

import edu.wpi.first.wpilibj.PIDOutput;

public class CustomPID {
    public double kP;
    public double kI;
    public double kD;
    public LimeLightPIDSource limeLightSource;

    public double error = 0.0;

    public double integral = 0.0;
    public double derivative = 0.0;
    public double output = 1.0;
    public double previous_error = 0.0;

    public double setpoint = DEFAULT_SETPOINT;
    public static final double DEFAULT_SETPOINT = 0.0;

    public double tolerance = DEFAULT_TOLERANCE;
    public static final double DEFAULT_TOLERANCE = 0.0;

    public double inputRangeMax = DEFAULT_INPUT_MAX;
    public static final double DEFAULT_INPUT_MAX = 1.0;

    public double inputRangeMin = DEFAULT_INPUT_MIN;
    public static final double DEFAULT_INPUT_MIN = -1.0;

    public double outputRangeMax = DEFAULT_OUTPUT_MAX;
    public static final double DEFAULT_OUTPUT_MAX = 1.0;

    public double outputRangeMin = DEFAULT_OUTPUT_MIN;
    public static final double DEFAULT_OUTPUT_MIN = -1.0;

    public CustomPID(double kP, double kI, double kD, LimeLightPIDSource limeLightSource) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.limeLightSource = limeLightSource;
    }

    public void setSetPoint(double setPoint) {
        this.setpoint = setPoint;
    }

    public void setInputRange(double max, double min) {
        inputRangeMax = max;
        inputRangeMin = min;
    }

    public void setOutputRange(double max, double min) {
        outputRangeMax = max;
        outputRangeMin = min;
    }

    public void setAbsoluteTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public void setKP(double kP) {
        this.kP = kP;
    }

    public void setKI(double kI) {
        this.kI = kI;
    }

    public void setKD(double kD) {
        this.kD = kD;
    }

    public double getPIDValue() {
        double pidValue = limeLightSource.getTY();

        if(pidValue > inputRangeMax) {
            pidValue = inputRangeMax;
        } else if (pidValue < inputRangeMin) {
            pidValue = inputRangeMin;
        }

        return pidValue;
    }

    public void calculatePID() {
        this.previous_error = error;
        error = setpoint - getPIDValue(); // Error = Target - Actual
        this.integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        derivative = (error - this.previous_error) / .02;
        this.output = kP*error + kI*this.integral + kD*derivative;
       
    }

    public double getOutput() {
        double returnOutput = output;

        if(output > outputRangeMax) {
            returnOutput = outputRangeMax;
        } else if(output < outputRangeMin) {
            returnOutput = outputRangeMin;
        }

        return returnOutput;
    }

    public boolean onTarget() {
        if((Math.abs(limeLightSource.getTY()) < Math.abs(setpoint) + Math.abs(tolerance) ) && (Math.abs(limeLightSource.getTY()) > Math.abs(setpoint) - Math.abs(tolerance))) {
            return true;
        } else {
            return false;
        }
    }

}