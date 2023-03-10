package frc.robot.subsystems.manipulator;

import SushiFrcLib.Motor.MotorHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kManipulator;
import frc.robot.Constants.kPorts;

/**
 * Controls the manipulator subsytem.
 */
public abstract class Manipulator extends SubsystemBase {
    protected CANSparkMax motor;

    protected Manipulator() {
        motor = MotorHelper.createSparkMax(kPorts.MANIPULATOR_MOTOR_ID, MotorType.kBrushless);
        motor.setSmartCurrentLimit(kManipulator.CURRENT_LIMITING);
        motor.setInverted(true);
        motor.burnFlash();
    }

    /**
     * Draw cone into claw.
     */
    public Command cone() {
        return runOnce(() -> {
            motor.set(kManipulator.SPEED * -1.0);
        });
    }

    /**
     * Reverse cone out of claw.
     */
    public Command coneReverse() {
        return runOnce(() -> {
            motor.set(kManipulator.SPEED);
        });
    }

    /**
     * Reverse the indexer.
     */
    public Command cube() {
        return runOnce(() -> {
            motor.set(kManipulator.SPEED);
        });
    }

    /**
     * Reverse cube out of claw.
     */
    public Command cubeReverse() {
        return runOnce(() -> {
            motor.set(kManipulator.SPEED * -1);
        });
    }

    /**
     * Stops indexer.
     */
    public Command stop() {
        return runOnce(() -> {
            motor.set(0);
        });
    }

    /**
     * Holds cube in maniupulator.
     */
    public Command holdCube() {
        return runOnce(() -> {
            motor.set(-0.01);
        });
    }
}
