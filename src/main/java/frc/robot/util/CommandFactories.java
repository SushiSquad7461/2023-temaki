package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kCommandTimmings;
import frc.robot.Constants.kArm.ArmPos;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.arm.AlphaArm;

public class CommandFactories {
    public static Command getConeScore(Intake intake, AlphaArm arm, Manipulator manipulator) {
        return new SequentialCommandGroup(
            intake.extendIntake(),
            new WaitCommand(kCommandTimmings.PNEUMATIC_WAIT_TIME),
            arm.moveArm(ArmPos.L2_SCORING),
            manipulator.coneReverse(),
            new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME),
            manipulator.stop(),
            arm.moveArm(ArmPos.LOWERED),
            new WaitCommand(kCommandTimmings.PNEUMATIC_WAIT_TIME),
            intake.retractIntake()
        );
    }


    public static Command getCubeScore(Intake intake, AlphaArm arm, Manipulator manipulator) {
        return new SequentialCommandGroup(
            intake.extendIntake(),
            new WaitCommand(kCommandTimmings.PNEUMATIC_WAIT_TIME),
            arm.moveArm(ArmPos.L2_SCORING),
            manipulator.cubeReverse(),
            new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME),
            manipulator.stop(),
            arm.moveArm(ArmPos.LOWERED),
            new WaitCommand(kCommandTimmings.PNEUMATIC_WAIT_TIME),
            intake.retractIntake()
        );
    }
}
