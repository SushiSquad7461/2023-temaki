// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kOI;
import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot (subsytems, commands, etc.) should be declared. 
 */
public class RobotContainer {
    Swerve swerve;
    XboxController driveController;
    private final SendableChooser<SequentialCommandGroup> autoChooser;
    private final AutoCommands autos;

    /**
     * Instaite subsystems and commands.
     */
    public RobotContainer() {
        driveController = new XboxController(0); 
        swerve = Swerve.getInstance();
        autos = new AutoCommands(swerve);
        autoChooser = new SendableChooser<>();

        Set<String> keys = autos.autos.keySet();
        autoChooser.setDefaultOption((String) keys.toArray()[0], autos.autos.get(keys.toArray()[0]));
        keys.remove((String) keys.toArray()[0]);
    
        for (String i : autos.autos.keySet()) {
            autoChooser.addOption(i, autos.autos.get(i));
        }
    
        SmartDashboard.putData("Auto Selector", autoChooser);

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
    }

    public Command getAutonomousCommand() {
        SmartDashboard.putData("Auto Selector", autoChooser);
        return autoChooser.getSelected();
    }
}
