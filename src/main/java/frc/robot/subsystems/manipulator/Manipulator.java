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
public class Manipulator extends SubsystemBase {
    private CANSparkMax motor;

    private static Manipulator instance;

    /**
     * singleton get instance method.
     */
    public static Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }
        return instance;
    }

    /**
     * Instaniate manipulator class.
     */
    private Manipulator() {
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
            motor.set(kManipulator.CONE_REVERSE_SPEED);
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
            motor.set(kManipulator.CUBE_REVERSE_SPEED);
        });
    }

    /**
     * Cube shoot.
     */
    public Command cubeShoot() {
        return runOnce(() -> {
            motor.set(kManipulator.CUBE_SHOOT_SPEEED);
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
            motor.set(kManipulator.SPEED * kManipulator.CUBE_HOLD_MULTIPLIER);
        });
    }

    @Override
    public void periodic() {}
}
