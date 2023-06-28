package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.kArm;
import frc.robot.Constants.kArm.ArmPos;
import frc.robot.Constants.kPorts;

/**
 * Implementation of beta arm.
 */
public class BetaArm extends Arm {
    private final DoubleSolenoid solenoidLeft;
    private final DoubleSolenoid solenoidRight;

    private ArmFeedforward armFeedforwardRetracted;
    private ArmFeedforward armFeedforwardExtended;

    private static BetaArm instance;
    private static DigitalInput limitSwitch;
    private boolean override;

    private double target;

    /**
     * Gets current instance of arm implements singelton.
     */
    public static BetaArm getInstance() {
        if (instance == null) {
            instance = new BetaArm();
        }
        return instance;
    }

    private BetaArm() {
        override = false;
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

        solenoidLeft = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, 
            kPorts.PNEUMATIC_FORWARD_CHANNEL_ARM, 
            kPorts.PNEUMATIC_REVERSE_CHANNEL_ARM
        );

        solenoidRight = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, 
            kPorts.PNEUMATIC_FORWARD_CHANNEL_ARM2, 
            kPorts.PNEUMATIC_REVERSE_CHANNEL_ARM2
        );

        limitSwitch = new DigitalInput(kPorts.ARM_LIMIT_SWITCH);

        solenoidLeft.set(Value.kReverse);
        solenoidRight.set(Value.kReverse);
    }

    @Override
    public void moveArm(double target) {
        this.target = normalizeAngle(target);
        updateArmPID(target);
    }

    public void updateArmPID(double target) {
        double armFF = solenoidLeft.get() == Value.kForward 
            ? armFeedforwardExtended.calculate(
                Units.degreesToRadians(getEncoderPosition() - kArm.FEEDFORWARD_ANGLE_OFFSET),
                0
            ) 
            :
            armFeedforwardRetracted.calculate(
                Units.degreesToRadians(getEncoderPosition() - kArm.FEEDFORWARD_ANGLE_OFFSET),
                0
            );

        setPosition(target, armFF);
    }

    @Override
    public Command moveArm(ArmPos angle) {
        return new SequentialCommandGroup(
            run(
                () -> {
                    if (angle != ArmPos.L3_SCORING) {
                        retractArm();
                    }
                }
            ).until(() -> armClosed()),
            run(() -> {
                moveArm(angle.getAngle());
            }).until(() -> isAtPos(angle.getAngle())),
            runOnce(
                () -> {
                    if (angle == ArmPos.L3_SCORING) {
                        moveArm(angle.getAngle()); // Make sure to update with new kG
                        extendArm();
                    }
                }
            )
        );
    }

    private void extendArm() {
        if (solenoidLeft.get() != Value.kForward) {
            toggleSolenoid();
        }
    }

    private void retractArm() {
        if (solenoidLeft.get() != Value.kReverse) {
            toggleSolenoid();
        }
    }

    public void override() {
        override = true;
    }

    public void cancelOverride(){
        override = false;
    }

    private boolean armClosed() {
        if(override) {
            return true;
        }
        return !limitSwitch.get();
    }

    /**
     * Toggles solenoids on the arm.
     */
    public void toggleSolenoid() {
        solenoidLeft.toggle();
        solenoidRight.toggle(); 
    }

    @Override
    public void periodic() {
        if (Math.abs(getAbsolutePosition() - leftMotor.getEncoder().getPosition()) > 0.3) {
            resetArm();
        }

        updateArmPID(target);
        update();
        SmartDashboard.putBoolean("Arm limit switch", limitSwitch.get());
        SmartDashboard.putNumber("Arm Absolute Encoder", getAbsolutePosition());
        // SmartDashboard.putNumber("Arm Absolute Encoder Feedforawrd angle", getAbsolutePosition() - kArm.FEEDFORWARD_ANGLE_OFFSET);
        SmartDashboard.putNumber("Arm Relativ Encoder", leftMotor.getEncoder().getPosition());
    }
}
