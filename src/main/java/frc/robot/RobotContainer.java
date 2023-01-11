// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot (subsytems, commands, etc.) should be declared. 
 */
public class RobotContainer {
    Swerve swerve;
    private final AutoCommands autos;

    /**
     * Instaite subsystems and commands.
     */
    public RobotContainer() {
        swerve = Swerve.getInstance();
        autos = new AutoCommands(swerve);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        swerve.setDefaultCommand(
            new TeleopSwerveDrive(swerve, true, false)
        );
    }

    public Command getAutonomousCommand() {
        return autos.getAuto();
    }
}
