package frc.robot.subsystems.util;

import java.util.ArrayList;
import java.util.Arrays;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kOI;
import frc.robot.OI;

public class Neo extends Motor {
    CANSparkMax motor;

    public int canID;
    public double currentLimit;
    private double startingEncoder;
    private double endingEncoder;

    public Neo(CANSparkMax motor) {
        this.motor = motor;
        this.oi = OI.getInstance();
        this.canID = motor.getDeviceId();
        this.currentLimit = motor.getOutputCurrent();
    }

    @Override
    public void startTwitch() {
        startingEncoder = motor.getEncoder().getPosition();
        setSpeed(.1, false);
    }

    @Override
    public void endTwitch() {
        disable();
        endingEncoder = motor.getEncoder().getPosition();
    }

    @Override
    public boolean checkEncoderErrors() {
        if (endingEncoder > startingEncoder + 40) {
            return true;
        }
        return false;
    }

    @Override
    public ArrayList<String> findErrors() {
        checkElecErrors();
        return getErrors();
    }

    @Override
    public ArrayList<String> findTotalErrors() {
        ArrayList<String> output = findErrors();
        if (!checkEncoderErrors()) {
            output.add("The motor isn't working");
        }
        return output;
    }

    @Override
    public String getRegisterString(String subsystem, String neoName) {
        return subsystem + " " + neoName + " " + this.canID + " 0 " + "0.0 " + "0 " + ((motor.getInverted()) ? 1 : 0)
                + " 0 " + this.currentLimit + " " + this.lowLimit + " " + this.highLimit + " 0 " + "0";
    }

    public ArrayList<String> getErrors() {
        ArrayList<String> ret = new ArrayList<>(allErrors);
        allErrors.removeAll(allErrors);
        return ret;
    }

    @Override
    public void setIdle(IdleMode idle) {
        REVLibError errorChecker = REVLibError.kOk;
        if (idle == IdleMode.COAST) {
            errorChecker = motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        } else {
            errorChecker = motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        }

        if (errorChecker != REVLibError.kOk) {
            allErrors.add("\n coast/brake of " + motor.getDeviceId() + " is " + errorChecker.toString());
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
            if (newSpeed < .09 && newSpeed > -.09) {
                newSpeed = 0;
            }
        } else {
            newSpeed = speed;
        }

        double position = motor.getEncoder().getPosition();
        if (((newSpeed > 0 && position >= highLimit) || (newSpeed < 0 && position <= lowLimit))) {
            newSpeed = 0;
        }

        motor.set(newSpeed);
    }

    @Override
    public void setCurrentLimit(double currentLimit) {
        REVLibError errorChecker = REVLibError.kOk;
        if (currentLimit != 0) {
            errorChecker = motor.setSmartCurrentLimit((int) (currentLimit));
        }

        if (errorChecker != REVLibError.kOk) {
            allErrors.add("\n current limit of " + motor.getDeviceId() + " is " + errorChecker.toString());
        }
    }

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

    public void disable() {
        motor.disable();
    }

    @Override
    public void checkElecErrors() {
        if (motor.getFault(CANSparkMax.FaultID.kBrownout)) {
            allErrors.add("\n" + motor.getDeviceId() + " brownout");
        }

        if (motor.getFault(CANSparkMax.FaultID.kMotorFault)) {
            allErrors.add("\n" + motor.getDeviceId() + " motor fault");
        }

        if (motor.getFault(CANSparkMax.FaultID.kOvercurrent)) {
            allErrors.add("\n" + motor.getDeviceId() + " over current");
        }

        if (motor.getFault(CANSparkMax.FaultID.kStall)) {
            allErrors.add("\n" + motor.getDeviceId() + " stalling");
        }

        if (motor.getFault(CANSparkMax.FaultID.kHasReset)) {
            allErrors.add("\n" + motor.getDeviceId() + " has reset");
        }

        if (motor.getFault(CANSparkMax.FaultID.kCANRX)) {
            allErrors.add("\n" + motor.getDeviceId() + " can rx");
        }

        if (motor.getFault(CANSparkMax.FaultID.kCANTX)) {
            allErrors.add("\n" + motor.getDeviceId() + " can tx");
        }

    }

}
