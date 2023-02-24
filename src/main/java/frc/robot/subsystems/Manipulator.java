package frc.robot.subsystems;

import SushiFrcLib.Motor.MotorHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kManipulator;
import frc.robot.Constants.kPorts;

/**
 * Controls the manipulator subsytem.
 */
public class Manipulator extends SubsystemBase {
    private CANSparkMax motor;
    private static Manipulator instance;

    /**
     * Gets the current manipular instance. for singleton support.
     */
    public static Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }
        return instance;
    }

    private Manipulator() {
        motor = MotorHelper.createSparkMax(kPorts.MANIPULATOR_MOTOR_ID, MotorType.kBrushless);
        motor.setSmartCurrentLimit(kManipulator.CURRENT_LIMITING);
        motor.setInverted(true);
        motor.burnFlash();

    }

    /**
     * Runs indexer in positive direction.
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
            motor.set(kManipulator.SPEED*3);
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
            motor.set(kManipulator.SPEED * -3.0);
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

    public Command holdCube() {
        return runOnce(() -> {
            motor.set(-0.01);
        });
    }

    public void periodic() {
        SmartDashboard.putNumber("Manipulator Current", motor.getOutputCurrent());
    }
}
