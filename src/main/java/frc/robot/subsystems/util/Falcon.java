package frc.robot.subsystems.util;

import java.util.ArrayList;

import javax.swing.text.Highlighter.Highlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kOI;
import frc.robot.OI;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.Motor.MotorHelper;

public class Falcon extends Motor {
    WPI_TalonFX motor;
    public int canID;
    public ErrorHandler errorHandler;
    public double currentLimit;
    private double startingEncoder;
    private double endingEncoder;
    private String subsystem;
    private String name;

    public Falcon(WPI_TalonFX motor) {
        errorHandler = ErrorHandler.getInstance();
        this.motor = motor;
        this.oi = OI.getInstance();
        this.canID = motor.getDeviceID();
        this.currentLimit = motor.getSupplyCurrent();
    }

    @Override
    public void setSubsystem(String subsystem) {
        this.subsystem = subsystem;
    }

    public void setName(String name) {
        this.name = name;
    }

    @Override
    public String getSubsystem() {
        return subsystem;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void startTwitch() {
        startTwitch(.1);
    }

    @Override
    public void startTwitch(double speed) {
        startingEncoder = motor.getSelectedSensorPosition();
        setSpeed(speed, false);
    }

    @Override
    public void endTwitch() {
        disable();
        endingEncoder = motor.getSelectedSensorPosition();
    }

    @Override
    public void checkEncoderErrors() {
        if (endingEncoder < startingEncoder + 40) {
            errorHandler.add("The motor isn't spinning");
        }
    }

    @Override
    public String getRegisterString(String subsystem, String swerveName) {
        return subsystem + " " + swerveName + " " + this.canID + " 0 " + "0.0 " + "0 " + ((motor.getInverted()) ? 1 : 0)
                + " 0 " + this.currentLimit + " " + this.lowLimit + " " + this.highLimit + " 0 " + "0";
    }

    @Override
    public void setIdle(IdleMode idle) {
        if (idle == IdleMode.COAST) {
            motor.setNeutralMode(NeutralMode.Coast);
        } else {
            motor.setNeutralMode(NeutralMode.Brake);
        }
    }

    @Override
    public void invertMotor(boolean flipped) {
        motor.setInverted(flipped);
    }

    @Override
    public void setSpeed(double speed, boolean isJoystick) {

        double newSpeed;
        if (isJoystick) {
            newSpeed = oi.getDriveTrainTranslationY();
            if (newSpeed < DiagnosticConstants.JOYSTICK_SPEED_BOUNDARIES && newSpeed > DiagnosticConstants.JOYSTICK_SPEED_BOUNDARIES * -1) {
                newSpeed = 0;
            }
        } else {
            newSpeed = speed;
        }

        double position = motor.getSelectedSensorPosition();

        if (((newSpeed > 0 && position >= highLimit) || (newSpeed < 0 && position <= lowLimit))) {
            newSpeed = 0;
        }

        motor.set(newSpeed);
    }

    @Override
    public void setCurrentLimit(double currentLimit) {
        motor.configSupplyCurrentLimit(MotorHelper.createCurrentLimt((int) currentLimit));
    }

    @Override
    public void setEncoderLimit(double lowLimit, double highLimit) {
        if (lowLimit == DiagnosticConstants.LOW_LIMIT_VALUE) {
            this.lowLimit = Double.MAX_VALUE * -1;
        } else {
            this.lowLimit = lowLimit * DiagnosticConstants.ENCODER_MULTIPLIER;
        }

        if (highLimit == DiagnosticConstants.HIGH_LIMIT_VALUE) {
            this.highLimit = Double.MAX_VALUE;
        } else {
            this.highLimit = highLimit * DiagnosticConstants.ENCODER_MULTIPLIER;
        }
    }

    @Override
    public void disable() {
        motor.disable();
    }

    public void checkElecErrors() {
    }

}
