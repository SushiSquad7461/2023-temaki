// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kArm.ArmPos;
import frc.robot.Constants.kCommandTimmings;
import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm.AlphaArm;
import frc.robot.util.CommandFactories;

/**
 * This class is where the bulk of the robot (subsytems, commands, etc.) should be declared. 
 */
public class RobotContainer {
    private final Swerve swerve;
    private final Intake intake;
    private final OI oi;
    private final AutoCommands autos;
    private final AlphaArm arm;
    private final Indexer indexer;
    private final Manipulator manipulator;
    private boolean intakeToggled = false;


    /**
     * Instaite subsystems and commands.
     */
    public RobotContainer() {
        SmartDashboard.putString("Robot Name", Constants.ROBOT_NAME.toString());

        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        arm = AlphaArm.getInstance();
        oi = OI.getInstance();
        indexer = Indexer.getInstance();
        manipulator = Manipulator.getInstance();

        autos = new AutoCommands(swerve);

        configureButtonBindings();
    }

    private void toggleIntake() {
        intakeToggled = !intakeToggled;
        if (intakeToggled) {
            (
                new SequentialCommandGroup(
                    intake.extendIntake(), 
                    intake.runIntake()
                )
            ).schedule();
        } else {
            (
                new SequentialCommandGroup(
                    intake.retractIntake(), 
                    new ParallelCommandGroup(
                        indexer.runIndexer(), 
                        manipulator.cube()
                    ), 
                    new WaitCommand(1.5), 
                    new ParallelCommandGroup(
                        intake.stopIntake(), 
                        indexer.stopIndexer(), 
                        manipulator.holdCube()

                    )
                )
            ).schedule();
        }
    }

    private void toggleIntakeReversal() {
        intakeToggled = !intakeToggled;
        if (intakeToggled) {
            (
                new SequentialCommandGroup(
                    intake.extendIntake(),
                    new ParallelCommandGroup( 
                        intake.reverseIntake(),
                        manipulator.cubeReverse()
                    )
                )
            ).schedule();
        } else {
            (
                new SequentialCommandGroup(
                    manipulator.stop(),
                    intake.retractIntake(), 
                    intake.stopIntake()
                )
            ).schedule();
        }
    }

    private void configureButtonBindings() {
        swerve.setDefaultCommand(
            new TeleopSwerveDrive(
                swerve, 
                () -> oi.getDriveTrainTranslationX(),
                () -> oi.getDriveTrainTranslationY(),
                () -> oi.getDriveTrainRotation(),
                true, 
                false
            )
        );

        // Toggle intake
        oi.getDriverController().leftBumper().onTrue(
            new InstantCommand(
                () -> {
                    toggleIntake();
                }
            )
        );

        oi.getDriverController().leftTrigger().onTrue(
            new InstantCommand(
                () -> {
                    toggleIntakeReversal();
                }
            )
        );

        // Move to april tag id 2
        oi.getDriverController().rightBumper().onTrue(
            swerve.moveToNearestAprilTag(null)
        );

        // // Reset odo
        oi.getDriverController().povUp().onTrue(
            swerve.resetOdometryToBestAprilTag()
        );

        oi.getDriverController().povLeft().onTrue(
            swerve.moveToNearestAprilTag(new Translation2d(0.9, 0.6))
        );

        oi.getDriverController().povRight().onTrue(
            swerve.moveToNearestAprilTag(new Translation2d(0.9, -0.6))
        );

        // Lower arm
        oi.getOperatorController().a().onTrue(new SequentialCommandGroup(
            arm.moveArm(ArmPos.LOWERED),
            new WaitCommand(kCommandTimmings.PNEUMATIC_WAIT_TIME),
            intake.retractIntake()
        ));

        // Raise arm to score at L2
        oi.getOperatorController().y().onTrue(new SequentialCommandGroup(
            intake.extendIntake(),
            new WaitCommand(kCommandTimmings.PNEUMATIC_WAIT_TIME),
            arm.moveArm(ArmPos.L2_SCORING)
        ));

        // Score item to relese cube
        oi.getOperatorController().x().onTrue(CommandFactories.getCubeScore(intake, arm, manipulator));

        // Score item to relese cone
        oi.getOperatorController().b().onTrue(CommandFactories.getConeScore(intake, arm, manipulator));

        // cone pick up from substation
        oi.getOperatorController().povUp().onTrue(new SequentialCommandGroup(
            intake.extendIntake(),
            new WaitCommand(kCommandTimmings.PNEUMATIC_WAIT_TIME),
            arm.moveArm(ArmPos.CONE_PICKUP_ALLIGMENT)
        ));

        // pickup cone
        oi.getOperatorController().povDown().onTrue(new SequentialCommandGroup(
            manipulator.cone(),
            arm.moveArm(ArmPos.CONE_PICKUP_LOWERED),
            new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME),
            arm.moveArm(ArmPos.CONE_PICKUP_ALLIGMENT),
            manipulator.stop()
        ));
    }

    public Command getAutonomousCommand() {
        return autos.getAuto();

    }
}
