package frc.robot.subsystems.util;

import java.util.ArrayList;

public abstract class Motor {
    enum IdleMode {
        COAST,
        BRAKE,
    }
    public abstract void setIdle(IdleMode idle);
    public abstract void invertMotor(boolean flipped);
    public abstract void setSpeed(double speed, boolean isJoystick);
    public abstract void setCurrentLimit(double limit);
    public abstract void setEncoderLimit(double low, double high);
    public abstract void disable();
    public abstract void checkElecErrors();
    public abstract ArrayList<String> getErrors();

    Object oi;
    double lowLimit = Double.MIN_VALUE;
    double highLimit = Double.MAX_VALUE;
    ArrayList<String> allErrors = new ArrayList<String>();
}
