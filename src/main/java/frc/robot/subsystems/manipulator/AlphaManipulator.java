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
public class AlphaManipulator extends Manipulator {
    private CANSparkMax motor;
    private static AlphaManipulator instance;

    /**
     * Gets the current manipular instance. for singleton support.
     */
    public static AlphaManipulator getInstance() {
        if (instance == null) {
            instance = new AlphaManipulator();
        }
        return instance;
    }

    private AlphaManipulator() {
        super();
    }

}
