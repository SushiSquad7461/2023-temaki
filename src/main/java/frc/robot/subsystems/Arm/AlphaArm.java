package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
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
    // TunableNumber leftMotorVelocity;
    private static AlphaArm instance;
    private final ArmFeedforward armFeedforward;

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
        // leftMotorVelocity = new TunableNumber("Left Motor Velocity", 0, Constants.TUNING_MODE);
        targetPos = new TunableNumber("Target Pos", 0, Constants.TUNING_MODE);

        armFeedforward = new ArmFeedforward(Constants.kArm.kS, Constants.kArm.kG, Constants.kArm.kV, Constants.kArm.kA);

        leftMotor = MotorHelper.createSparkMax(kArm.LEFT_MOTOR_ID, MotorType.kBrushless, kArm.LEFT_INVERSION,
                    kArm.LEFT_CURRENT_LIMIT, kArm.LEFT_IDLE_MODE, armP.get(), armI.get(), armD.get(), armF.get());
        rightMotor = MotorHelper.createSparkMax(kArm.RIGHT_MOTOR_ID, MotorType.kBrushless, kArm.RIGHT_INVERSION, 
                     kArm.RIGHT_CURRENT_LIMIT, kArm.LEFT_IDLE_MODE, armP.get(), armI.get(), armD.get(), armF.get());
        encoder = new DutyCycleEncoder(kArm.ENCODER_CHANNEL);

        leftMotorPid = leftMotor.getPIDController();
        leftMotor.getEncoder().setPositionConversionFactor(360 / Constants.kArm.GEAR_RATIO);
        leftMotor.getEncoder().setVelocityConversionFactor((360 / Constants.kArm.GEAR_RATIO) / 60);

        rightMotor.follow(leftMotor, true);

        // resetArm();
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


    public void setPosition(double degree) {
        //if (degree < 0 || degree > kArm.MAX_POSITION) {
            leftMotorPid.setReference(degree, ControlType.kPosition, 0, armFeedforward.calculate(Units.degreesToRadians(degree - 111), 0));
        //}
    }

    public boolean isAtPos() {
        return Math.abs(getError()) < kArm.ERROR;
    }

    public void resetArm() {
        leftMotor.getEncoder().setPosition(encoder.get() * kArm.GEAR_RATIO);
    }
    
    
    // update arm PIDF values
    public void update() {
        if (armP.hasChanged()) {
            leftMotorPid.setP(armP.get());
        }
        if (armI.hasChanged()) {
            leftMotorPid.setI(armI.get());
        }
        if (armD.hasChanged()) {
            leftMotorPid.setD(armD.get());
        }
        if (armF.hasChanged()) {
            leftMotorPid.setFF(armF.get());
        }
        // if (targetPos.hasChanged()) {
            setPosition(targetPos.get());
        // }
        // if (leftMotorVelocity.hasChanged()) {
        //     runArm(leftMotorVelocity.get());
        // }
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
