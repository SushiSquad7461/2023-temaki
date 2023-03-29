package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kArm.ArmPos;
import frc.robot.commands.AutoBalance;
import frc.robot.Constants.kCommandTimmings;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.manipulator.Manipulator;
import java.util.HashMap;

/**
 * This class generates auto commands.
 */
public class AutoCommands {
    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final Manipulator manipulator;
    private final Arm arm;
    private final SwerveAutoBuilder autoBuilder;
    private final SendableChooser<Command> autoChooser;
    HashMap<String, Command> eventMap = new HashMap<String, Command>();
    private final double chargeSpeed;
    private final double autoBalanceWait;

    /**
     * Define all auto commands.
     */

    public AutoCommands(Swerve swerve, Indexer indexer, Intake intake, Manipulator manipulator, Arm arm) {
        this.swerve = swerve;
        this.indexer = indexer;
        this.intake = intake;
        this.manipulator = manipulator;
        this.arm = arm;
        chargeSpeed = 2.0;
        autoBalanceWait = 0.5;

        eventMap.put("intakeDown", new SequentialCommandGroup(intake.extendIntake(), intake.runIntake(), new ParallelCommandGroup(
            indexer.runIndexer()
            // manipulator.cube()
        )));

        eventMap.put("intakeUp", new SequentialCommandGroup(
            intake.retractIntake(),
            new ParallelCommandGroup(
                indexer.runIndexer(), 
                manipulator.cube()
            ), 
            new WaitCommand(2.0), 
            new ParallelCommandGroup(
                intake.stopIntake(), 
                indexer.stopIndexer(), 
                manipulator.holdCube()
            )
        ));

        eventMap.put("scoreCube", new SequentialCommandGroup(
            arm.moveArm(ArmPos.L3_SCORING),
            new WaitCommand(kCommandTimmings.PNEUMATIC_WAIT_TIME),
            manipulator.cubeReverse(),
            new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME)
        ));

        eventMap.put("scoreCone", new SequentialCommandGroup(
            manipulator.cone(),
            new WaitCommand(0.1),
            arm.moveArm(ArmPos.L3_SCORING),
            new WaitCommand(0.5),
            manipulator.coneReverse(),
            new WaitCommand(0.1),
            manipulator.stop()
        ));

        eventMap.put("raiseArm", new SequentialCommandGroup(
            arm.moveArm(ArmPos.L3_SCORING)
        ));

        eventMap.put("lowerArm", new SequentialCommandGroup(
            arm.moveArm(ArmPos.LOWERED)
        ));

        autoBuilder = new SwerveAutoBuilder(
            swerve::getPose, // Pose2d supplier
            swerve::resetOdometryAndGyro, // Pose2d consumer, used to reset odometry at the beginning of auto
            kSwerve.SWERVE_KINEMATICS, // SwerveDriveKinematics
            kSwerve.TRANSLATION_CONTROLLER, // PID constants to correct for translation error (used to create the X and Y PID controllers)
            kSwerve.ANGLE_CONTROLLER, // PID constants to correct for rotation error (used to create the rotation controller)
            swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true // The drive subsystem. Used to properly set the requirements of path following commands
            swerve
        );

        autoChooser = new SendableChooser<>();

        autoChooser.addOption("nothing", new InstantCommand(() -> {
            System.out.println("YOUR A CLOWN");
        }));

        autoChooser.addOption("2 Piece Loading Zone", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("2_Piece_Loading_Zone")),
            new WaitCommand(0.5),
            scoreCube()
        ));

        autoChooser.addOption("Red 2 Piece Loading Zone", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_2_Piece_Loading_Zone")),
            new WaitCommand(0.5),
            scoreCube()
        ));

        autoChooser.addOption("2 Piece Loading Zone + Bal", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("2_Piece_Loading_Zone")),
            new WaitCommand(0.5),
            scoreCube(),
            makeAuto(("2_piece_bal"), chargeSpeed),
            new WaitCommand(autoBalanceWait),
            new AutoBalance()
        ));

        autoChooser.addOption("Red 2 Piece Loading Zone + Bal", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_2_Piece_Loading_Zone")),
            new WaitCommand(0.5),
            scoreCube(),
            makeAuto(("Red_2_piece_bal"), chargeSpeed),
            new WaitCommand(autoBalanceWait),
            new AutoBalance()
        ));

        autoChooser.addOption("3 Piece Piece Loading Zone", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("2_Piece_Loading_Zone")),
            new WaitCommand(0.5),
            scoreCube(),
            makeAuto(("3_Piece_Loading_Zone")),
            new WaitCommand(0.2),
            scoreCube()
        ));

        autoChooser.addOption("1 piece + Bal", new SequentialCommandGroup(
            scoreCone(),
            arm.moveArm(ArmPos.LOWERED),
            makeAuto(("Charge"), chargeSpeed),
            new WaitCommand(autoBalanceWait),
            new AutoBalance()
        ));

        autoChooser.addOption("Red 1 piece + Bal", new SequentialCommandGroup(
            scoreCone(),
            arm.moveArm(ArmPos.LOWERED),
            makeAuto(("Red_Charge"), chargeSpeed),
            new WaitCommand(autoBalanceWait),
            new AutoBalance()
        ));

        autoChooser.addOption("2 piece burm", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("2_Piece_Loading_Zone_Burm"), 2.0),
            new WaitCommand(0.5),
            scoreCube()
        ));

        autoChooser.addOption("Red 2 piece burm", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_2_Piece_Loading_Zone_Burm"), 2.0),
            new WaitCommand(0.5),
            scoreCube()
        ));

        autoChooser.addOption("2 piece burm + bal", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("2_Piece_Loading_Zone_Burm"), 2.0),
            new WaitCommand(0.3),
            scoreCube(),
            makeAuto(("2_piece_bal_burm"), chargeSpeed),
            new WaitCommand(0.2),
            new AutoBalance()
        ));

        autoChooser.addOption("Red 2 piece burm + bal", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_2_Piece_Loading_Zone_Burm"), 2.0),
            new WaitCommand(0.3),
            scoreCube(),
            makeAuto(("Red_2_piece_bal_burm"), chargeSpeed),
            new WaitCommand(0.2),
            new AutoBalance()
        ));

        autoChooser.addOption("Red 3 Piece Piece Loading Zone", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_2_Piece_Loading_Zone")),
            new WaitCommand(0.5),
            scoreCube(),
            makeAuto(("Red_3_Piece_Loading_Zone")),
            new WaitCommand(0.2),
            scoreCube()
        ));

        putAutoChooser();
    }

    // private String getAutoPath(String pathName) {
    //     var table = NetworkTableInstance.getDefault().getTable("FMSInfo");
    //     boolean isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);
       
    //     System.out.println((isRedAlliance ? "Red_" : "") + pathName);
    //     return (isRedAlliance ? "Red_" : "") + pathName;
    //     // return pathName;
    // }

    private void putAutoChooser() {
        SmartDashboard.putData("Auto Selector", autoChooser); 
    }

    /**
     * Get currently selected auto.
     */
    public Command getAuto() {
        return autoChooser.getSelected();
    }

    private Command makeAuto(String path) {
        return autoBuilder.fullAuto(
            PathPlanner.loadPathGroup(path, kSwerve.MAX_SPEED, kSwerve.MAX_ACCELERATION)
        );
    }

    private Command makeAuto(String path, double speed) {
        return autoBuilder.fullAuto(
            PathPlanner.loadPathGroup(path, speed, kSwerve.MAX_ACCELERATION)
        );
    }

    private Command scoreCube() {
        return new SequentialCommandGroup(
            // arm.moveArm(ArmPos.L3_SCORING),
            // new WaitCommand(kCommandTimmings.PNEUMATIC_WAIT_TIME),
            manipulator.cubeReverse(),
            new WaitCommand(0.4)
        );
    }

    private Command scoreCone() {
        return new SequentialCommandGroup(
            manipulator.cone(),
            new WaitCommand(0.1),
            arm.moveArm(ArmPos.L3_SCORING),
            new WaitCommand(0.7),
            manipulator.coneReverse(),
            new WaitCommand(0.2),
            manipulator.stop()
        );
    }

    private Pose2d getInitialPose(PathPlannerTrajectory path) {
        return new Pose2d(
                path.getInitialPose().getTranslation(),
                path.getInitialState().holonomicRotation
        );
    }
}
