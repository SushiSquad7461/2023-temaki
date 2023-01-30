// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm.AlphaArm;

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
                    new WaitCommand(1.0), 
                    new ParallelCommandGroup(
                        intake.stopIntake(), 
                        indexer.stopIndexer(), 
                        manipulator.holdCube()
                    )
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
        oi.getDriverController().leftBumper().onTrue(new InstantCommand(() -> {
            toggleIntake();
        }));

        // Lower arm
        oi.getOperatorController().a().onTrue(new SequentialCommandGroup(
            intake.extendIntake(),
            new WaitCommand(0.7),
            arm.lowerArm(),
            new WaitCommand(0.7),
            intake.retractIntake()
        ));

        // Raise arm to score
        oi.getOperatorController().y().onTrue(new SequentialCommandGroup(
            intake.extendIntake(),
            new WaitCommand(0.7),
            arm.raiseArmToScore(),
            new WaitCommand(0.7),
            intake.retractIntake()
        ));

        // Score item to relese cube
        oi.getOperatorController().x().onTrue(new SequentialCommandGroup(
            manipulator.cubeReverse(),
            new WaitCommand(1),
            manipulator.stop()
        ));

        // Score item to relese cone
        oi.getOperatorController().b().onTrue(new SequentialCommandGroup(
            manipulator.coneReverse(),
            new WaitCommand(1.5),
            manipulator.stop()
        ));

        // raise arm for cone
        oi.getOperatorController().povUp().onTrue(new SequentialCommandGroup(
            intake.extendIntake(),
            new WaitCommand(0.7),
            arm.raiseArmPickupCone(),
            new WaitCommand(0.7),
            intake.retractIntake()
        ));

        // pickup cone
        oi.getOperatorController().povDown().onTrue(new SequentialCommandGroup(
            manipulator.cone(),
            arm.raiseArmToScore(),
            new WaitCommand(1),
            arm.raiseArmPickupCone(),
            manipulator.stop()
        ));
    }

    public Command getAutonomousCommand() {
        return autos.getAuto();
    }
}
