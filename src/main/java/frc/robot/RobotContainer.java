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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm.AlphaArm;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.BetaArm;
import frc.robot.subsystems.indexer.AlphaIndexer;
import frc.robot.subsystems.indexer.BetaIndexer;
import frc.robot.subsystems.indexer.Indexer;

/**
 * This class is where the bulk of the robot (subsytems, commands, etc.) should be declared. 
 */
public class RobotContainer {
    private final Swerve swerve;
    private final Intake intake;
    private final OI oi;
    private final AutoCommands autos;
    private final Arm arm;
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

        switch (Constants.ROBOT_NAME) {
            case ALPHA:
                arm = AlphaArm.getInstance();
                indexer = AlphaIndexer.getInstance();
                break;
            default:
                arm = BetaArm.getInstance();
                indexer = BetaIndexer.getInstance();
                break;
        }

        oi = OI.getInstance();
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


        oi.getDriverController().y().onTrue(
            new SequentialCommandGroup(
                intake.reverseIntake(),
                indexer.runIndexer(),
                manipulator.cubeReverse()
            )
        ).onFalse(new SequentialCommandGroup(
            intake.stopIntake(),
            indexer.stopIndexer(),
            manipulator.stop()
        ));

        // Move to april t
        // oi.getDriverController().rightBumper().onTrue(
        //     swerve.moveToAprilTag(2, null)
        // );

        // // Reset odo
        // oi.getDriverController().povUp().onTrue(
        //     swerve.resetOdometryToBestAprilTag()
        // );

        // oi.getDriverController().povLeft().onTrue(
        //     swerve.moveToAprilTag(2, new Translation2d(0.65, 0.5))
        // );

        // Lower arm
        oi.getOperatorController().a().onTrue(new SequentialCommandGroup(
            arm.moveArm(ArmPos.LOWERED),
            new WaitCommand(kCommandTimmings.PNEUMATIC_WAIT_TIME)
            //intake.retractIntake()
        ));

        // Raise arm to score at L2
        oi.getOperatorController().y().onTrue(new SequentialCommandGroup(
            //intake.extendIntake(),
            new WaitCommand(kCommandTimmings.PNEUMATIC_WAIT_TIME),
            arm.moveArm(ArmPos.L3_SCORING)
        ));

        // Score item to relese cube
        oi.getOperatorController().x().onTrue(new SequentialCommandGroup(
            manipulator.cubeReverse(),
            new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME),
            manipulator.stop()
        ));

        // Score item to relese cone
        oi.getOperatorController().b().onTrue(new SequentialCommandGroup(
            manipulator.coneReverse(),
            new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME),
            manipulator.stop()
        ));

        // raise arm for cone
        oi.getOperatorController().povUp().onTrue(new SequentialCommandGroup(
            //intake.extendIntake(),
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
