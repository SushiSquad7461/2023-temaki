// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
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

    /**
     * Instaite subsystems and commands.
     */
    public RobotContainer() {
        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        arm = AlphaArm.getInstance();
        oi = OI.getInstance();
        autos = new AutoCommands(swerve);
        Indexer.getInstance();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        swerve.setDefaultCommand(
            new TeleopSwerveDrive(
                swerve, 
                () -> oi.getDriveTrainTranslationX(),
                () -> oi.getDriveTrainTranslationX(),
                () -> oi.getDriveTrainRotation(),
                true, 
                false
            )
        );

        oi.getDriverController().x().onTrue(intake.extendAndRunIntake());
        oi.getDriverController().b().onTrue(intake.retrakeAndStopIntake());
    }

    public Command getAutonomousCommand() {
        return autos.getAuto();
    }
}
