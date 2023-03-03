package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake;
import frc.robot.Constants.kPorts;
import frc.robot.subsystems.util.MotorTest;
import frc.robot.subsystems.util.Neo;

/**
 * Class that controls a cube intake.
 */
public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor;
    private final DoubleSolenoid solenoidLeft;
    private final DoubleSolenoid solenoidRight;
    private MotorTest motorTest;

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
        intakeMotor = new CANSparkMax(kPorts.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.setInverted(true);
        intakeMotor.burnFlash();

        solenoidLeft = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, 
            kPorts.PNEUMATIC_FORWARD_CHANNEL_LEFT, 
            kPorts.PNEUMATIC_REVERSE_CHANNEL_LEFT
        );
        solenoidRight = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, 
            kPorts.PNEUMATIC_FORWARD_CHANNEL_RIGHT, 
            kPorts.PNEUMATIC_REVERSE_CHANNEL_RIGHT
        );

        solenoidLeft.set(Value.kReverse);
        solenoidRight.set(Value.kReverse);

        Neo neoIntake = new Neo(intakeMotor);
        motorTest = MotorTest.getInstance();
        motorTest.register(neoIntake, null, "IntakeSubsystem", "intake");
        motorTest.register(null, solenoidLeft, "IntakeSubsytems", "solenoidLeft");
        motorTest.register(null, solenoidRight, "IntakeSubsytems", "solenoidRight");
    }

    /**
     * Makes sure intake is extended and turns on motor.
     */
    public Command runIntake() {
        return runOnce(() -> {
            intakeMotor.set(kIntake.MOTOR_SPEED);
        });
    }

    /**
     * Extends intake.
     */
    public Command extendIntake() {
        return runOnce(() -> {
            if (solenoidLeft.get() == Value.kReverse) {
                toggleIntake();
            }
        });
    }

    /**
     * Makes sure intake is retracted and turns of motor. 
     */
    public Command stopIntake() {
        return runOnce(() -> {
            intakeMotor.set(0);
        });
    }

    /**
     * Retracts intake.
     */
    public Command retractIntake() {
        return runOnce(() -> {
            if (solenoidLeft.get() == Value.kForward) {
                toggleIntake();
            }
        });
    }

    /**
     * Reverses intake.
     */
    public Command reverseIntake() {
        return runOnce(() -> {
            intakeMotor.set(-kIntake.MOTOR_SPEED);
        });
    }

    private void toggleIntake() {
        solenoidLeft.toggle();
        solenoidRight.toggle();
    }
}
