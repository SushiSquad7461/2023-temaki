package frc.robot.subsystems.Arm;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.kArm;
import frc.robot.Constants.kPorts;

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
        targetPos = new TunableNumber("Target Pos", 0, Constants.TUNING_MODE);

        armFeedforward = new ArmFeedforward(
            Constants.kArm.kS, 
            Constants.kArm.kG, 
            Constants.kArm.kV, 
            Constants.kArm.kA
        );

        leftMotor = MotorHelper.createSparkMax(
            kPorts.LEFT_MOTOR_ID, 
            MotorType.kBrushless, 
            kArm.LEFT_INVERSION,
            kArm.LEFT_CURRENT_LIMIT, 
            kArm.LEFT_IDLE_MODE, 
            armP.get(), 
            armI.get(), 
            armD.get(),
            armF.get()
        );

        rightMotor = MotorHelper.createSparkMax(
            kPorts.RIGHT_MOTOR_ID, 
            MotorType.kBrushless, 
            kArm.RIGHT_INVERSION, 
            kArm.RIGHT_CURRENT_LIMIT, 
            kArm.LEFT_IDLE_MODE, 
            armP.get(),
            armI.get(), 
            armD.get(), 
            armF.get()
        );

        encoder = new DutyCycleEncoder(kPorts.ENCODER_CHANNEL);

        leftMotorPid = leftMotor.getPIDController();
        leftMotor.getEncoder().setPositionConversionFactor(
            360.0 / Constants.kArm.GEAR_RATIO
        ); // degrees

        leftMotor.getEncoder().setVelocityConversionFactor(
            (360.0 / Constants.kArm.GEAR_RATIO) / 60.0
        ); //degrees per second

        rightMotor.follow(leftMotor, true);
        resetArm();
    }

    // returns encoder position in degrees
    public double getReletivePosition() {
        return leftMotor.getEncoder().getPosition() * 5;
    }

    public double getReletiveVelocity() {
        return leftMotor.getEncoder().getPosition() * 5;
    }

    public double getAbsolutePosition() {
        return (encoder.get() * 360.0) - kArm.ENCODER_ANGLE_OFFSET;
    }

    private double getLeftMotorVelocity() {
        return leftMotor.getEncoder().getVelocity();
    }

    private double getRightMotorVelocity() {
        return leftMotor.getEncoder().getVelocity();
    }

    public double getError(double target) {
        return target - getAbsolutePosition();
    }

    public void runArm(double speed) {
        leftMotor.set(speed);
    }

    public void stopArm() {
        leftMotor.set(0);
    }


    public void setPosition(double degree) {
        if (degree < 0) {
            degree = 0;
        } else if (degree > kArm.MAX_POSITION) {
            degree = kArm.MAX_POSITION;
        }

        targetPos.setDefault(degree);

        leftMotorPid.setReference(
            degree, 
            CANSparkMax.ControlType.kPosition,
            0, 
            armFeedforward.calculate(
                Units.degreesToRadians(degree - kArm.FEEDFORWARD_ANGLE_OFFSET),
                0
            )
        );
    }

    private boolean isAtPos(double degrees) {
        return Math.abs(getError(degrees)) < kArm.ERROR;
    }

    public void resetArm() {
        leftMotor.getEncoder().setPositionConversionFactor(
            360.0 / Constants.kArm.GEAR_RATIO
        ); // degrees

        leftMotor.getEncoder().setVelocityConversionFactor(
            (360.0 / Constants.kArm.GEAR_RATIO) / 60.0
        );
        leftMotor.getEncoder().setPosition(getAbsolutePosition());
    }
    
    /**
     * Update tunnable numbers for PIDF and target pos.
     */
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

        if (targetPos.hasChanged()) {
            setPosition(targetPos.get());
        }
    }

    public Command raiseArmToScore() {
        return moveArm(75); // 75 L2
    }

    public Command raiseArmPickupCone() {
        return moveArm(100);
    }

    public Command lowerArm() {
        return moveArm(0);
    }

    private Command moveArm(double degrees) {
        return run(() -> {
            setPosition(degrees);
        }).until(() -> isAtPos(degrees));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Relative Encoder Pos", leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Absolute Encoder Pos", getAbsolutePosition());
        SmartDashboard.putNumber("Arm Target Pose", targetPos.get());
        SmartDashboard.putNumber("Left Motor Voltage", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Motor Voltage", rightMotor.getOutputCurrent());
        update();
    }
}
