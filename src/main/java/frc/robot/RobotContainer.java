// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import SushiFrcLib.Constants.SushiConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.kOI;
import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.commands.autoBalance;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot (subsytems, commands, etc.) should be declared. 
 */
public class RobotContainer {
    Swerve swerve;
    XboxController driveController;

    /**
     * Instaite subsystems and commands.
     */
    public RobotContainer() {
        driveController = new XboxController(SushiConstants.OI.DRIVER_PORT); 
        swerve = Swerve.getInstance();
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        swerve.setDefaultCommand(
            new TeleopSwerveDrive(
                swerve,
                driveController,
                kOI.DRIVE_TRANSLATION_Y,
                kOI.DRIVE_TRANSLATION_X,
                kOI.DRIVE_ROTATE,
                true,
                false
            )
        );

        new JoystickButton(driveController, XboxController.Button.kA.value).onTrue(new autoBalance());

    }

    public Command getAutonomousCommand() {
        return null;
    }
}
