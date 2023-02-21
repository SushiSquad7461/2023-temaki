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
public abstract class Intake extends SubsystemBase {
    private final DoubleSolenoid solenoidRight;
    /**
     * Gets instance for singleton.
     */


    public Intake() {
        solenoidRight = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, 
            kPorts.PNEUMATIC_FORWARD_CHANNEL, 
            kPorts.PNEUMATIC_REVERSE_CHANNEL
        );

        solenoidRight.set(Value.kReverse);
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
            if (solenoidRight.get() == Value.kReverse) {
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
            if (solenoidRight.get() == Value.kForward) {
                toggleIntake();
            }
        });
    }

    /**
     * Reverses intake.
     */
    public abstract Command reverseIntake();

    private void toggleIntake() {
        solenoidRight.toggle();
    }
}
