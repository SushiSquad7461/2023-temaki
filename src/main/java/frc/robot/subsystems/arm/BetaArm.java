package frc.robot.subsystems.arm;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.TunableNumber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.kArm;
import frc.robot.Constants.kArm.ArmPos;
import frc.robot.Constants.kPorts;

/**
 * Implementation of beta arm.
 */
public class BetaArm extends Arm {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final DoubleSolenoid solenoid;
    private final TunableNumber armP;
    private final TunableNumber armI;
    private final TunableNumber armD;
    private final TunableNumber armF;
    private final TunableNumber targetPos;
    private static BetaArm instance;
    private final ArmFeedforward armFeedforwardRetracted;
    private final ArmFeedforward armFeedforwardExtended;
    private SparkMaxPIDController leftMotorPid;
    private final DutyCycleEncoder absoluteEncoder;

    /**
     * Gets current instance of arm. implements singelton.
     */
    public static BetaArm getInstance() {
        if (instance == null) {
            instance = new BetaArm();
        }
        return instance;
    }

    private BetaArm() {
        armP = new TunableNumber("Arm P", kArm.KP, Constants.TUNING_MODE);
        armI = new TunableNumber("Arm I", kArm.KI, Constants.TUNING_MODE);
        armD = new TunableNumber("Arm D", kArm.KD, Constants.TUNING_MODE);
        armF = new TunableNumber("Arm F", kArm.KF, Constants.TUNING_MODE);
        targetPos = new TunableNumber("Target Pos", 0, Constants.TUNING_MODE);

        armFeedforwardRetracted = new ArmFeedforward(
            Constants.kArm.KS, 
            Constants.kArm.KGR, 
            Constants.kArm.KV, 
            Constants.kArm.KA
        );

        armFeedforwardExtended = new ArmFeedforward(
            Constants.kArm.KS,
            Constants.kArm.KGE,
            Constants.kArm.KV,
            Constants.kArm.KA
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

        // absoluteEncoder = leftMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder = new DutyCycleEncoder(kPorts.ENCODER_CHANNEL);

        solenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, 
            kPorts.PNEUMATIC_FORWARD_CHANNEL_ARM, 
            kPorts.PNEUMATIC_REVERSE_CHANNEL_ARM
        );

        solenoid.set(Value.kReverse);

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

    public double getAbsolutePosition() {
        return (absoluteEncoder.get() * 360.0) - kArm.ENCODER_ANGLE_OFFSET;
    }

    public double getError(double target) {
        return target - getAbsolutePosition();
    }

    public void toggleSolenoid() {
        solenoid.toggle();
    }

    /**
     * Sets the position of the arm to a certain angle.
     */
    public void setPosition(double degree) {
        if (degree < 0) {
            degree = 0;
        } else if (degree > kArm.MAX_POSITION) {
            degree = kArm.MAX_POSITION;
        }

        targetPos.setDefault(degree);

        if(solenoid.get() == Value.kForward) {
            leftMotorPid.setReference(
                degree, 
                CANSparkMax.ControlType.kPosition,
                0, 
                armFeedforwardExtended.calculate(
                    Units.degreesToRadians(degree - kArm.FEEDFORWARD_ANGLE_OFFSET),
                    0
                )
            );
        } else {
            leftMotorPid.setReference(
                degree, 
                CANSparkMax.ControlType.kPosition,
                0, 
                armFeedforwardRetracted.calculate(
                    Units.degreesToRadians(degree - kArm.FEEDFORWARD_ANGLE_OFFSET),
                    0
                )
            );
        }
    }

    private boolean isAtPos(double degrees) {
        return Math.abs(getError(degrees)) < kArm.ERROR;
    }

    /**
     * Resets the arm encoder.
     */
    public void resetArm() {
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

    @Override
    public Command moveArm(ArmPos angle) {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                    if (angle != ArmPos.L3_SCORING)
                        retractArm();
                    if(angle == ArmPos.LOWERED)
                        new WaitCommand(5).schedule();
                }
            ),
            angle == ArmPos.LOWERED ? new WaitCommand(3): new WaitCommand(0), 
            moveArm(angle.getAngle()),
            new InstantCommand(
                () -> {
                    if (angle == ArmPos.L3_SCORING) {
                        extendArm();
                    }
                }
            )
        );
    }

    public void extendArm() {
        if (solenoid.get() != Value.kForward) {
            solenoid.toggle();
        }
    }

    public void retractArm() {
        if (solenoid.get() != Value.kReverse) {
            solenoid.toggle();
        }
    }

    private Command moveArm(double degrees) {
        return run(() -> {
            setPosition(degrees);
        }).until(() -> isAtPos(degrees));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Relative Encoder Pos", leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm Target Pose", targetPos.get());
        SmartDashboard.putNumber("Left Motor Voltage", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Motor Voltage", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("ur moms absolute encoder", getAbsolutePosition());

        update();
    }
}
