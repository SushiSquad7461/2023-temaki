package frc.robot.subsystems.util;

import java.util.ArrayList;

import javax.swing.text.Highlighter.Highlight;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kOI;
import frc.robot.OI;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Falcon extends Motor {
    WPI_TalonFX motor;
    public int canID;
    public double currentLimit;

    public Falcon(WPI_TalonFX motor) {
        this.motor = motor;
        this.oi = OI.getInstance();
        this.canID = motor.getDeviceID();
        this.currentLimit = motor.getSupplyCurrent();
    }

    @Override
    public String getRegisterString(String subsystem, String neoName){
        return subsystem + " " + neoName + " "+ this.canID + " 0 "  + "0 " + "0 " + ((motor.getInverted()) ? 1 : 0) + " " +this.currentLimit + " " + this.lowLimit + " " +this.highLimit + "0 " + "0";
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
            if (newSpeed < .08 && newSpeed > -.08) {
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
        // System.out.println(motor.getSelectedSensorPosition() +"pos");
        // System.out.println(lowLimit);

    }

    @Override
    public void setCurrentLimit(double currentLimit) {
        SupplyCurrentLimitConfiguration CurrentLimit = new SupplyCurrentLimitConfiguration(true, currentLimit,
                currentLimit, 0.1);
        motor.configSupplyCurrentLimit(CurrentLimit);
    }

    // public void setCurrentLimit(double currentLimit1, double currentLimit2,
    // double threshhold) {
    // SupplyCurrentLimitConfiguration CurrentLimit = new
    // SupplyCurrentLimitConfiguration(true, currentLimit1,
    // currentLimit2,threshhold);
    // motor.configSupplyCurrentLimit(CurrentLimit);
    // }

    @Override
    public void setEncoderLimit(double lowLimit, double highLimit) {
        if (lowLimit == -2) {
            this.lowLimit = Double.MAX_VALUE * -1;
        } else {
            this.lowLimit = lowLimit * 1000;
        }

        if (highLimit == 0) {
            this.highLimit = Double.MAX_VALUE;
        } else {
            this.highLimit = highLimit * 1000;
        }
    }

    @Override
    public void disable() {
        motor.disable();
    }

    public void checkElecErrors() {
    }

    public ArrayList<String> getErrors() {
        return null;
    }
}
