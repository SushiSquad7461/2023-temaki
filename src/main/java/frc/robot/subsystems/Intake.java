package frc.robot.subsystems;

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
public class Intake extends SubsystemBase {
    private CANSparkMax motorIntake;
    private final DoubleSolenoid solenoidRight;

    private static Intake instance;

    /**
     * Gets instance for singleton.
     */
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }


    private Intake() {
        motorIntake = new CANSparkMax(kPorts.INTAKE_MOTOR_ID, MotorType.kBrushless);
        motorIntake.setInverted(true);
        motorIntake.burnFlash();

        solenoidRight = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, 
            kPorts.PNEUMATIC_FORWARD_CHANNEL_RIGHT, 
            kPorts.PNEUMATIC_REVERSE_CHANNEL_RIGHT
        );

        solenoidRight.set(Value.kReverse);
    }

    /**
     * Makes sure intake is extended and turns on motor.
     */
    public Command runIntake() {
        return runOnce(() -> {
            motorIntake.set(kIntake.MOTOR_SPEED);
        });
    }

    /**
     * Extends intake.
     */
    public Command extendIntake() {
        return runOnce(() -> {
            if (solenoidRight.get() == Value.kReverse) {
                toggleIntake();
            }
        });
    }

    /**
     * Makes sure intake is retracted and turns of motor. 
     */
    public Command stopIntake() {
        return runOnce(() -> {
            motorIntake.set(0);
        });
    }

    /**
     * Retracts intake.
     */
    public Command retractIntake() {
        return runOnce(() -> {
            if (solenoidRight.get() == Value.kForward) {
                toggleIntake();
            }
        });
    }

    /**
     * Reverses intake.
     */
    public Command reverseIntake() {
        return runOnce(() -> {
            motorIntake.set(-kIntake.MOTOR_SPEED);
        });
    }

    private void toggleIntake() {
        solenoidRight.toggle();
    }
}
