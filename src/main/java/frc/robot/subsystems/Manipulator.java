package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kManipulator;
import frc.robot.Constants.kPorts;

public class Manipulator extends SubsystemBase {
    private CANSparkMax motor;
    private static Manipulator instance;

    public static Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }
        return instance;
    }

    private Manipulator() {
        motor = MotorHelper.createSparkMax(kPorts.MANIPULATOR_MOTOR_ID, MotorType.kBrushless);
    }


    /**
     * Runs indexer in positive direction.
     */
    public Command run() {
        return runOnce(() -> {
            motor.set(kManipulator.SPEED);
        });
    }

    public Command reverse() {
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

}
