package frc.robot.subsystems.manipulator;

import SushiFrcLib.Motor.MotorHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kManipulator;
import frc.robot.Constants.kPorts;

/**
 * Controls the manipulator subsytem.
 */
public class BetaManipulator extends Manipulator {
    private CANSparkMax motor;
    private static BetaManipulator instance;
    private DigitalInput beamBreak;

    /**
     * Gets the current manipular instance. for singleton support.
     */
    public static BetaManipulator getInstance() {
        if (instance == null) {
            instance = new BetaManipulator();
        }
        return instance;
    }

    private BetaManipulator() {
        super();
        beamBreak = new DigitalInput(1);
    }


    public Command release() {
        return beamBreak.get() ? coneReverse() : cubeReverse();
    }

    public void periodic() {
        SmartDashboard.putNumber("Manipulator Current", motor.getOutputCurrent());
    }
}
