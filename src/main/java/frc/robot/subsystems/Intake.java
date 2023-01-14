package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    CANSparkMax motorIntake;
    DoubleSolenoid solenoid1;
    DoubleSolenoid solenoid2;

    public Intake() {
        motorIntake = new CANSparkMax(0, MotorType.kBrushless);
        solenoid1 = new DoubleSolenoid(null, 0, 1);
        solenoid2 = new DoubleSolenoid(null, 0, 1);
    }

    private void extendIntake() {
        solenoid1.toggle();
        solenoid2.toggle();
        motorIntake.set(1);
    }

    private void retrakeIntake() {
        solenoid1.toggle();
        solenoid2.toggle();
        motorIntake.set(-1);
    }
}
