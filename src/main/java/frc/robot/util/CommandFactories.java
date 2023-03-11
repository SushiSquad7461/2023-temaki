package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kArm.ArmPos;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;

/**
 * Util commands.
 */
public class CommandFactories {
    /**
     * Resets robot to starting position.
     */
    public static Command resetRobot(
        Intake intake, 
        Indexer indexer, 
        Arm arm, 
        Manipulator manipulator
    ) {
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                intake.retractIntake(),
                intake.stopIntake()
            ),
            indexer.stopIndexer(),
            arm.moveArm(ArmPos.LOWERED),
            manipulator.stop()
        );
    } 
}
