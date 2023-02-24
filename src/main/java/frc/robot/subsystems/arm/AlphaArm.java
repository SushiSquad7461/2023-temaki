package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.kArm;
import frc.robot.Constants.kArm.ArmPos;

/**
 * Implements arm ABC for alpha robot.
 */
public class AlphaArm extends Arm {
    private final ArmFeedforward armFeedforward;

    private static AlphaArm instance;

    /**
     * Gets current instance of arm. implements singelton.
     */
    public static AlphaArm getInstance() {
        if (instance == null) {
            instance = new AlphaArm();
        }
        return instance;
    }

    private AlphaArm() {
        armFeedforward = new ArmFeedforward(
            Constants.kArm.KS, 
            Constants.kArm.KG, 
            Constants.kArm.KV, 
            Constants.kArm.KA
        );
    }

    /**
     * Moves arm to an ArmPos angle.
     */
    public Command moveArm(ArmPos angle) {
        return run(() -> {
            moveArm(angle.getAngle());
        }).until(() -> isAtPos(angle.getAngle()));
    }

    @Override
    public void moveArm(double target) {
        target = normalizeAngle(target);

        double armFF = armFeedforward.calculate(
            Units.degreesToRadians(target - kArm.FEEDFORWARD_ANGLE_OFFSET),
            0
        );
        setPosition(target, armFF);
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
