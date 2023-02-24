package frc.robot.subsystems.util;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

public class Neo extends Motor {
    CANSparkMax motor;
    public int canID;
    public double currentLimit;

    public Neo(CANSparkMax motor) {
        this.motor = motor;
        this.canID = motor.getDeviceId();
        this.currentLimit = motor.getOutputCurrent();
    }

    public ArrayList<String> getErrors() {
        ArrayList<String> ret = allErrors;
        allErrors = new ArrayList<String>();
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

        if (errorChecker != REVLibError.kOk){
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
            //newSpeed = oi.getDriveTrainTranslationY()
            newSpeed = 0;
        } else {
            newSpeed = speed;
        }
       
        double position = motor.getEncoder().getPosition();
        if (highLimit != 0){
            if (!((speed < 0 && position >= lowLimit) || position <= highLimit)){
                newSpeed = 0;
            }
        }
        motor.set(newSpeed);
    }

    @Override
    public void setCurrentLimit (double currentLimit) {
        REVLibError errorChecker = REVLibError.kOk;
        if (currentLimit != 0){
            errorChecker = motor.setSmartCurrentLimit((int)(currentLimit));
        }

        if (errorChecker != REVLibError.kOk){
            allErrors.add("\n current limit of " + motor.getDeviceId() + " is " + errorChecker.toString());
        }
    }

    @Override
    public void setEncoderLimit(double lowLimit, double highLimit) {
        this.lowLimit = lowLimit;
        if (highLimit != 0) {
            motor.getEncoder().setPosition(0);
            this.highLimit = highLimit;
        }
    }

    public void disable() {
        motor.disable();
    }

    public void checkElecErrors() { //add to string array in motor test
        if (motor.getFault(CANSparkMax.FaultID.kBrownout)){
            allErrors.add("\n" + motor.getDeviceId() + " brownout");
        }

        if (motor.getFault(CANSparkMax.FaultID.kMotorFault)){
            allErrors.add("\n" + motor.getDeviceId() + " motor fault");
        }

        if (motor.getFault(CANSparkMax.FaultID.kOvercurrent)){
            allErrors.add("\n" + motor.getDeviceId() + " over current");
        }

        if (motor.getFault(CANSparkMax.FaultID.kStall)){
            allErrors.add("\n" + motor.getDeviceId() + " stalling");
        }

        if (motor.getFault(CANSparkMax.FaultID.kHasReset)){
            allErrors.add("\n" + motor.getDeviceId() + " has reset");
        }

        
        if (motor.getFault(CANSparkMax.FaultID.kCANRX)){
            allErrors.add("\n" + motor.getDeviceId() + " can rx");
        }

        if (motor.getFault(CANSparkMax.FaultID.kCANRX)){
            allErrors.add("\n" + motor.getDeviceId() +  CANSparkMax.FaultID.kCANRX.toString());
        }

    }

}
