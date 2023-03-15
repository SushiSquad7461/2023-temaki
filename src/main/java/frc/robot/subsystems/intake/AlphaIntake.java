package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kIntake;
import frc.robot.Constants.kPorts;
import frc.robot.subsystems.util.MotorTest;

/**
 * Class that controls a cube intake.
 */
public class AlphaIntake extends Intake {
    private CANSparkMax motorIntake;

    private static AlphaIntake instance;
    private static MotorTest motorTest;
    /**
     * Gets instance for singleton.
     */
    public static AlphaIntake getInstance() {
        if (instance == null) {
            instance = new AlphaIntake();
        }
        return instance;
    }

    public void registerMotors() {
        motorTest.registerMotor(motorIntake, getName(), "intakeMotor");
    }

    private AlphaIntake() {
        super();
        motorIntake = new CANSparkMax(kPorts.INTAKE_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        motorIntake.setInverted(true);
        motorIntake.burnFlash();
        motorTest = MotorTest.getInstance();
    }

    /**
     * Makes sure intake is extended and turns on motor.
     */
    @Override
    public Command runIntake() {
        return runOnce(() -> {
            motorIntake.set(kIntake.MOTOR_SPEED);
        });
    }

    /**
     * Makes sure intake is retracted and turns of motor. 
     */
    @Override
    public Command stopIntake() {
        return runOnce(() -> {
            motorIntake.set(0);
        });
    }

    /**
     * Reverses intake.
     */
    @Override
    public Command reverseIntake() {
        return runOnce(() -> {
            motorIntake.set(-kIntake.MOTOR_SPEED);
        });
    }
}

