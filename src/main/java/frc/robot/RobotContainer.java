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
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm.AlphaArm;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.BetaArm;
import frc.robot.subsystems.indexer.AlphaIndexer;
import frc.robot.subsystems.indexer.BetaIndexer;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.AlphaIntake;
import frc.robot.subsystems.intake.BetaIntake;
import frc.robot.subsystems.intake.Intake;

/**
 * This class is where the bulk of the robot (subsytems, commands, etc.) should be declared. 
 */
public class RobotContainer {
    public final Swerve swerve;
    public final Intake intake;
    public final OI oi;
    // public final AutoCommands autos;
    // public final Arm arm;
    public final Indexer indexer;
    public final Manipulator manipulator;
    public final Arm arm;

    /**
     * Instaite subsystems and commands.
     */
    public RobotContainer() {
        SmartDashboard.putString("Robot Name", Constants.ROBOT_NAME.toString());
        swerve = Swerve.getInstance();
        oi = OI.getInstance();
        manipulator = Manipulator.getInstance();

        switch (Constants.ROBOT_NAME) {
          case ALPHA:
              arm = AlphaArm.getInstance();
              indexer = AlphaIndexer.getInstance();
              intake = AlphaIntake.getInstance();
              break;
          default:
              arm = BetaArm.getInstance();
              indexer = BetaIndexer.getInstance();
              intake = BetaIntake.getInstance();
              break;
        }

        // autos = new AutoCommands(swerve, indexer, intake, manipulator, arm);

    }
}
