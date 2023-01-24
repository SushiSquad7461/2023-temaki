// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm.AlphaArm;

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


    /**
     * Instaite subsystems and commands.
     */
    public RobotContainer() {
        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        arm = AlphaArm.getInstance();
        oi = OI.getInstance();
        indexer = Indexer.getInstance();
        manipulator = Manipulator.getInstance();

        autos = new AutoCommands(swerve);

        configureButtonBindings();
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

        oi.getDriverController().x().onTrue(new ParallelCommandGroup(intake.extendAndRunIntake()));
        oi.getDriverController().b().onTrue(new SequentialCommandGroup(intake.retractAndStopIntake(), new ParallelCommandGroup(indexer.runIndexer(), manipulator.run()), new WaitCommand(1), indexer.stopIndexer()));
        oi.getDriverController().a().onTrue(new SequentialCommandGroup(intake.extendIntake(), new WaitCommand(1.5), arm.raiseArmToScore(), manipulator.reverse(), new WaitCommand(2), manipulator.stop(), arm.lowerArm(), new WaitCommand(1), intake.retractAndStopIntake()));

        oi.getDriverController().leftTrigger().onTrue(new SequentialCommandGroup(intake.extendIntake(), new WaitCommand(0.5), arm.raiseArmPickupCone(), manipulator.reverse()));
        oi.getDriverController().leftBumper().onTrue(new SequentialCommandGroup(arm.raiseArmToScore(), new WaitCommand(1), arm.raiseArmPickupCone()));
        oi.getDriverController().rightTrigger().onTrue(new SequentialCommandGroup(intake.extendIntake(), arm.lowerArm(), new WaitCommand(0.2), intake.retractAndStopIntake()));


    }

    public Command getAutonomousCommand() {
        return autos.getAuto();
    }
}
