package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.kArm;

public class AlphaArm extends Arm {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final DutyCycleEncoder encoder;
    private final TunableNumber armP;
    private final TunableNumber armI;
    private final TunableNumber armD;
    private final TunableNumber armF;
    private final TunableNumber targetPos;
    private static AlphaArm instance;

    private SparkMaxPIDController leftMotorPid;

    public static AlphaArm getInstance() {
        if (instance == null) {
            instance = new AlphaArm();
        }
        return instance;
    }

    private AlphaArm() {
        armP = new TunableNumber("Arm P", kArm.kP, Constants.TUNING_MODE);
        armI = new TunableNumber("Arm I", kArm.kI, Constants.TUNING_MODE);
        armD = new TunableNumber("Arm D", kArm.kD, Constants.TUNING_MODE);
        armF = new TunableNumber("Arm F", kArm.kF, Constants.TUNING_MODE);
        targetPos = new TunableNumber("Target Pos", 0, Constants.TUNING_MODE);

        leftMotor = MotorHelper.createSparkMax(kArm.LEFT_MOTOR_ID, MotorType.kBrushless, kArm.LEFT_INVERSION,
                    kArm.LEFT_CURRENT_LIMIT, kArm.LEFT_IDLE_MODE, armP.get(), armI.get(), armD.get(), armF.get());
        rightMotor = MotorHelper.createSparkMax(kArm.RIGHT_MOTOR_ID, MotorType.kBrushless, kArm.RIGHT_INVERSION, 
                     kArm.RIGHT_CURRENT_LIMIT, kArm.LEFT_IDLE_MODE, armP.get(), armI.get(), armD.get(), armF.get());
        encoder = new DutyCycleEncoder(kArm.ENCODER_CHANNEL);

        leftMotorPid = leftMotor.getPIDController();
        rightMotor.follow(leftMotor, true);

        resetArm();
    }

    // returns encoder position in degrees
    public double getPosition() {
        return encoder.get() * 360.0;
    }

    public double getLeftMotorVelocity() {
        return leftMotor.getEncoder().getVelocity();
    }

    public double getRightMotorVelocity() {
        return leftMotor.getEncoder().getVelocity();
    }

    public double getError() {
        return targetPos.get() - getPosition();
    }

    public void runArm(double speed) {
        leftMotor.set(speed);
    }

    public void stopArm() {
        leftMotor.set(0);
    }

    public void setPosition(double degrees) {
        if (degrees < 0 || degrees > kArm.MAX_POSITION) {
            throw new IllegalArgumentException("Position input should be between 0 and " + kArm.MAX_POSITION + " degrees.");
        }

        double rotation = (degrees / 360.0) * kArm.GEAR_RATIO;
        leftMotorPid.setReference(rotation, CANSparkMax.ControlType.kPosition);
    }

    public boolean isAtPos() {
        return Math.abs(getError()) < kArm.ERROR;
    }

    public void resetArm() {
        leftMotor.getEncoder().setPosition(encoder.get() * kArm.GEAR_RATIO);
    }
    
    
    // update arm PIDF values
    public void update() {
        leftMotorPid.setP(armP.get());
        leftMotorPid.setI(armI.get());
        leftMotorPid.setD(armD.get());
        leftMotorPid.setFF(armF.get());
        setPosition(targetPos.get());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Absolute Encoder Pos", getPosition());
        SmartDashboard.putNumber("Left Motor velocity", getLeftMotorVelocity());
        SmartDashboard.putNumber("Right Motor Velocity", getRightMotorVelocity());
        SmartDashboard.putNumber("Arm Target Pose", targetPos.get());
        SmartDashboard.putNumber("Arm Error", getError());
        update();
    }
}
