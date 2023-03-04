package frc.robot.subsystems.manipulator;

import SushiFrcLib.Motor.MotorHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kManipulator;
import frc.robot.Constants.kPorts;

/**
 * Controls the manipulator subsytem.
 */
public abstract class Manipulator extends SubsystemBase {
    private CANSparkMax motor;
    private static BetaManipulator instance;

    protected Manipulator() {
        motor = MotorHelper.createSparkMax(kPorts.MANIPULATOR_MOTOR_ID, MotorType.kBrushless);
        motor.setSmartCurrentLimit(kManipulator.CURRENT_LIMITING);
        motor.setInverted(true);
        motor.burnFlash();
    }

    public abstract Command cone();

    /**
     * Reverse cone out of claw.
     */
    public abstract Command coneReverse();

    /**
     * Reverse the indexer.
     */
    public abstract Command cube();

    /**
     * Reverse cube out of claw.
     */
    public abstract Command cubeReverse();

    /**
     * Stops indexer.
     */
    public abstract Command stop();

    public abstract Command holdCube();
}
