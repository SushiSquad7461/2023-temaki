package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.kCommandTimmings;
import frc.robot.Constants.kPorts;

/**
 * Controls the manipulator subsytem.
 */
public class BetaManipulator extends Manipulator {
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
        beamBreak = new DigitalInput(kPorts.MANIPULATOR_BEAM_BREAk);
    }

    /**
     * Relase cube or cone.
     */
    public void release() {
        if (isBeamBreakBlocked()) {
            new SequentialCommandGroup(
                cubeReverse(), 
                new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME), 
                stop()
            ).schedule();
        } else {
            new SequentialCommandGroup(
                coneReverse(),
                new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME), 
                stop()
            ).schedule();
        }
    }

    public void periodic() {
        SmartDashboard.putNumber("Manipulator Current", motor.getOutputCurrent());
        // SmartDashboard.putBoolean("Beam Break", isBeamBreakBlocked());
    }

    public boolean isBeamBreakBlocked() {
        return !beamBreak.get();
    }
}
