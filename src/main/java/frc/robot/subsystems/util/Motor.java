package frc.robot.subsystems.util;

import java.util.ArrayList;

import frc.robot.OI;

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

    public abstract String getRegisterString(String subsystem, String name);

    public abstract void endTwitch();

    public abstract void startTwitch();

    public abstract boolean checkEncoderErrors();

    public abstract ArrayList<String> findErrors();

    public abstract ArrayList<String> findTotalErrors();

    OI oi;
    double lowLimit = Double.MAX_VALUE * -1;
    double highLimit = Double.MAX_VALUE;
    ArrayList<String> allErrors = new ArrayList<String>();
}
