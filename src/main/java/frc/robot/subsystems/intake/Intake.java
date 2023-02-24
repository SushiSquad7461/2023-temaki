package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kPorts;

/**
 * Class that controls a cube intake.
 */
public abstract class Intake extends SubsystemBase {
    private final DoubleSolenoid solenoid;
    private boolean intakeIn;

    /**
     * Creates a solenoid intake.
     */
    public Intake() {
        solenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, 
            kPorts.PNEUMATIC_FORWARD_CHANNEL, 
            kPorts.PNEUMATIC_REVERSE_CHANNEL
        );

        solenoid.set(Value.kReverse);
        intakeIn = true;
    }

    public boolean isIn() {
        return intakeIn;
    }

    /**
     * Makes sure intake is extended and turns on motor.
     */
    public abstract Command runIntake();

    /**
     * Extends intake.
     */
    public Command extendIntake() {
        return runOnce(() -> {
            if (solenoid.get() == Value.kReverse) {
                toggleIntake();
            }
        });
    }

    /**
     * Makes sure intake is retracted and turns of motor. 
     */
    public abstract Command stopIntake();

    /**
     * Retracts intake.
     */
    public Command retractIntake() {
        return runOnce(() -> {
            if (solenoid.get() == Value.kForward) {
                toggleIntake();
            }
        });
    }

    /**
     * Reverses intake.
     */
    public abstract Command reverseIntake();

    private void toggleIntake() {
        solenoid.toggle();
        intakeIn = !intakeIn;
    }
}
