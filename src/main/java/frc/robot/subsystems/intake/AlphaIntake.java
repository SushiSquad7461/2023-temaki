package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake;
import frc.robot.Constants.kPorts;

/**
 * Class that controls a cube intake.
 */
public class AlphaIntake extends Intake {
    private CANSparkMax motorIntake;
    private static AlphaIntake instance;

    /**
     * Gets instance for singleton.
     */
    public static AlphaIntake getInstance() {
        if (instance == null) {
            instance = new AlphaIntake();
        }
        return instance;
    }


    private AlphaIntake() {
        super();
        motorIntake = new CANSparkMax(kPorts.INTAKE_MOTOR_ID, MotorType.kBrushless);
        motorIntake.setInverted(true);
        motorIntake.burnFlash();
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

