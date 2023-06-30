package frc.robot.subsystems.arm;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.TunableNumber;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kArm;
import frc.robot.Constants.kArm.ArmPos;
import frc.robot.Constants.kPorts;

/**
 * Defines arm ABC for 2023 robots.
 */
public abstract class Arm extends SubsystemBase {
    protected final TunableNumber armP;
    protected final TunableNumber armI;
    protected final TunableNumber armD;
    protected final TunableNumber armF;
    protected final TunableNumber targetPos;

    protected final CANSparkMax leftMotor;
    protected final CANSparkMax rightMotor;
    private SparkMaxPIDController leftMotorPid;

    private final DutyCycleEncoder absoluteEncoder;

    protected Arm() {
        armP = new TunableNumber("Arm P", kArm.KP, Constants.TUNING_MODE);
        armI = new TunableNumber("Arm I", kArm.KI, Constants.TUNING_MODE);
        armD = new TunableNumber("Arm D", kArm.KD, Constants.TUNING_MODE);
        armF = new TunableNumber("Arm F", kArm.KF, Constants.TUNING_MODE);

        targetPos = new TunableNumber(
            "Target Pos",
            kArm.ArmPos.LOWERED.getAngle(), 
            Constants.TUNING_MODE
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

        leftMotorPid = leftMotor.getPIDController();

        absoluteEncoder = new DutyCycleEncoder(kPorts.ENCODER_CHANNEL);

        leftMotor.getEncoder().setPositionConversionFactor(
            360.0 / Constants.kArm.GEAR_RATIO
        ); // degrees

        leftMotor.getEncoder().setVelocityConversionFactor(
            (360.0 / Constants.kArm.GEAR_RATIO) / 60.0
        ); //degrees per second

        rightMotor.follow(leftMotor, true);

        resetArm();
    }

    /**
     * Update tunnable numbers for PIDF and target pos.
     */
    protected void update() {
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
            moveArm(targetPos.get());
        }
    }

    protected double getError(double target) {
        return target - getAbsolutePosition();
    }

    protected double getEncoderPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    protected boolean isAtPos(double angle) {
        return Math.abs(getError(angle)) < kArm.ERROR;
    }

    /**
     * Gets through bore encoder position.
     *
     * @return position in degrees
     */
    protected double getAbsolutePosition() {
        return (absoluteEncoder.get() * 360.0) - kArm.ENCODER_ANGLE_OFFSET;
    }

    /**
     * Sets arm to a certain position.
     *
     * @param target position of the arm
     */
    public void setPosition(double target, double armFF) {
        targetPos.setDefault(target);

        leftMotorPid.setReference(
            target, 
            CANSparkMax.ControlType.kPosition,
            0, 
            armFF 
        );
    }

    public abstract void moveArm(double target);

    /**
     * Moves arm to a certain position.
     */
    public abstract Command moveArm(ArmPos angle);

    /**
     * Resets the arm encoder.
     */
    public void resetArm() {
        leftMotor.getEncoder().setPosition(getAbsolutePosition());
    }

    /**
     * Normalizes the arm angle.
     */
    public double normalizeAngle(double angle) {
        if (angle < 0) {
            return 0;
        } else if (angle > kArm.MAX_POSITION) {
            return kArm.MAX_POSITION;
        }
        return angle;
    }

}
