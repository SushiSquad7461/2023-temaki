package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake;
import frc.robot.Constants.kPorts;

public class Intake extends SubsystemBase {
    private CANSparkMax motorIntake;
    private final DoubleSolenoid solenoidLeft;
    private final DoubleSolenoid solenoidRight;

    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }


    private Intake() {
        motorIntake = new CANSparkMax(kPorts.INTAKE_MOTOR_ID, MotorType.kBrushless);
        solenoidLeft = new DoubleSolenoid(PneumaticsModuleType.REVPH, kPorts.PNEUMATIC_FORWARD_CHANNEL_LEFT, kPorts.PNEUMATIC_REVERSE_CHANNEL_LEFT);
        solenoidRight = new DoubleSolenoid(PneumaticsModuleType.REVPH, kPorts.PNEUMATIC_FORWARD_CHANNEL_RIGHT, kPorts.PNEUMATIC_REVERSE_CHANNEL_RIGHT);

        solenoidLeft.set(Value.kReverse);
        solenoidRight.set(Value.kReverse);
    }

    public Command extendIntake() {
        return runOnce(() -> {
            solenoidLeft.toggle();
            solenoidRight.toggle();

            SmartDashboard.putString("In sdakj", "IN");
            motorIntake.set(kIntake.MOTOR_SPEED * -1);
        });
    }

    public Command retrakeIntake() {
        return runOnce(() -> {
            SmartDashboard.putString("In sdakj", "OUT");
            solenoidLeft.toggle();
            solenoidRight.toggle();
            motorIntake.set(0);
        });
    }
}
