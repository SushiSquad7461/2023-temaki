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
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoBalancePID;
import frc.robot.commands.TeleopSwerveDrive;
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
import frc.robot.subsystems.manipulator.AlphaManipulator;
import frc.robot.subsystems.manipulator.BetaManipulator;
import frc.robot.subsystems.manipulator.Manipulator;

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

    /**
     * Instaite subsystems and commands.
     */
    public RobotContainer() {
        SmartDashboard.putString("Robot Name", Constants.ROBOT_NAME.toString());

        swerve = Swerve.getInstance();
        oi = OI.getInstance();

        switch (Constants.ROBOT_NAME) {
          case ALPHA:
              arm = AlphaArm.getInstance();
              indexer = AlphaIndexer.getInstance();
              intake = AlphaIntake.getInstance();
              manipulator = AlphaManipulator.getInstance();
              configureAlphaButtonBindings();
              break;
          default:
              arm = BetaArm.getInstance();
              indexer = BetaIndexer.getInstance();
              intake = BetaIntake.getInstance();
              manipulator = BetaManipulator.getInstance();
              configureBetaButtonBindings();
              break;
        }

        autos = new AutoCommands(swerve, indexer, intake, manipulator, arm);

        configureButtonBindings();
    }

    /**
     * Configures button bindings for alpha.
     */
    public void configureAlphaButtonBindings() {
        // raise arm for cone
        oi.getOperatorController().povUp().onTrue(new SequentialCommandGroup(
            intake.extendIntake(),
            arm.moveArm(ArmPos.CONE_PICKUP_ALLIGMENT),
            manipulator.cone()
        ));

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
        oi.getOperatorController().x().onTrue(new SequentialCommandGroup(
            manipulator.cubeReverse(),
            new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME),
            manipulator.stop()
        ));

        oi.getOperatorController().b().onTrue(new SequentialCommandGroup(
            manipulator.coneReverse(),
            new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME),
            manipulator.stop()
        ));
    }

    /**
     * Configures button bindings for beta.
     */
    public void configureBetaButtonBindings() {
        
        // Score item to relese cube
        oi.getOperatorController().x().onTrue(new SequentialCommandGroup(
            ((BetaManipulator) manipulator).release(),
            new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME),
            manipulator.stop()
        ));

        oi.getDriverController().x().onTrue(
            new SequentialCommandGroup(
                indexer.reverseIndexer(),
                intake.extendIntake(),
                ((BetaIntake) intake).cubeShoot()
            )
        ).onFalse(
            new SequentialCommandGroup(
                indexer.stopIndexer(),
                intake.retractIntake(),
                intake.stopIntake()
            )
        );

        oi.getDriverController().y().onTrue(
            new SequentialCommandGroup(
                ((BetaIntake) intake).coneIntake(),
                indexer.runIndexer(),
                manipulator.cone()
            )
        ).onFalse(new ParallelCommandGroup(
            intake.stopIntake(),
            new SequentialCommandGroup(
                manipulator.cone(),
                new WaitCommand(0),
                indexer.stopIndexer(),
                manipulator.stop()
            )
        ));

        oi.getOperatorController().leftTrigger().onTrue(
            new InstantCommand(
                () -> ((BetaArm) arm).toggleSolenoid()
            )
        );

        // raise arm for cone
        oi.getOperatorController().povUp().onTrue(new SequentialCommandGroup(
            arm.moveArm(ArmPos.CONE_PICKUP_ALLIGMENT),
            manipulator.cone()
        ));

        // Lower arm
        oi.getOperatorController().a().onTrue(new SequentialCommandGroup(
            arm.moveArm(ArmPos.LOWERED)
        ));

        // Raise arm to score at L2
        oi.getOperatorController().y().onTrue(new SequentialCommandGroup(
            arm.moveArm(ArmPos.L3_SCORING)
        ));
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

        oi.getDriverController().rightTrigger().onTrue(new AutoBalance());

        oi.getDriverController().b().onTrue(new InstantCommand(() -> {
            swerve.turnOnLocationLock(180);
        })).onFalse(new InstantCommand(() -> {
            swerve.turnOfLocationLock();
        }));

        oi.getDriverController().a().onTrue(new InstantCommand(() -> {
            swerve.turnOnLocationLock(0);
        })).onFalse(new InstantCommand(() -> {
            swerve.turnOfLocationLock();
        }));


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

        oi.getOperatorController().povLeft().onTrue(new SequentialCommandGroup(
            arm.moveArm(ArmPos.L2_SCORING)
        ));


        // // Score item to relese cone
        // oi.getOperatorController().b().onTrue(CommandFactories.getConeScore(intake, arm, manipulator));

        // pickup cone
        oi.getOperatorController().povDown().onTrue(new SequentialCommandGroup(
            manipulator.cone(),
            arm.moveArm(ArmPos.CONE_PICKUP_LOWERED),
            new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME),
            arm.moveArm(ArmPos.CONE_PICKUP_ALLIGMENT),
            manipulator.stop()
        ));

        oi.getDriverController().rightBumper().whileTrue(
            swerve.moveToNearestAprilTag(null)
        );

        // // Reset odo
        oi.getDriverController().povDown().onTrue(
            swerve.resetOdometryToBestAprilTag()
        );

        // TODO: add alliance based substation selection
        oi.getDriverController().povUp().onTrue(
            new SequentialCommandGroup(
                swerve.moveToAprilTag(4, new Translation2d(1.1, -0.2))
            )
        );

        oi.getDriverController().povLeft().whileTrue(
            swerve.moveToNearestAprilTag(new Translation2d(0.8, 0.6))
        );

        oi.getDriverController().povRight().whileTrue(
            swerve.moveToNearestAprilTag(new Translation2d(0.8, -0.6))
        );
    }

    private void toggleIntake() {
        if (intake.isIn()) {
            (
                new SequentialCommandGroup(
                    intake.extendIntake(), 
                    intake.runIntake(),
                    indexer.runIndexer()
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
        if (intake.isIn()) {
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

    public Command getAutonomousCommand() {
        return autos.getAuto();

    }
}
