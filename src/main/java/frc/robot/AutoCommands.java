package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kArm.ArmPos;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kCommandTimmings;
import frc.robot.Constants.kSwerve;
import frc.robot.commands.AutoBalance;
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
    private final Manipulator manipulator;
    private final Arm arm;

    private final SwerveAutoBuilder autoBuilder;
    private final SendableChooser<Command> autoChooser;

    HashMap<String, Command> eventMap = new HashMap<String, Command>();

    /**
     * Define all auto commands.
     */

    public AutoCommands(
        Swerve swerve, 
        Indexer indexer, 
        Intake intake, 
        Manipulator manipulator, 
        Arm arm
    ) {
        this.manipulator = manipulator;
        this.arm = arm;

        eventMap.put("intakeDown",
            new SequentialCommandGroup(
                intake.extendIntake(), 
                intake.runIntake(), 
                indexer.runIndexer()
            )
        );

        eventMap.put("intakeUp", new SequentialCommandGroup(
            intake.retractIntake(),
            new ParallelCommandGroup(
                indexer.runIndexer(), 
                manipulator.cube()
            ), 
            new WaitCommand(2.5), 
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

        eventMap.put("raiseArmL2", new SequentialCommandGroup(
            arm.moveArm(ArmPos.L2_SCORING)
        ));

        eventMap.put("lowerArm", new SequentialCommandGroup(
            arm.moveArm(ArmPos.LOWERED)
        ));

        autoBuilder = new SwerveAutoBuilder(
            swerve::getPose, 
            swerve::resetOdometryAndGyro,
            kSwerve.SWERVE_KINEMATICS, 
            kSwerve.TRANSLATION_CONTROLLER, 
            kSwerve.ANGLE_CONTROLLER, 
            swerve::setModuleStates,
            eventMap,
            false, // Path mirroring based on alliance color
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
            makeAuto(("2_piece_bal"), kAuto.CHARGE_SPEED),
            new WaitCommand(kAuto.AUTO_BALANCE_WAIT),
            new AutoBalance()
        ));

        autoChooser.addOption("Red 2 Piece Loading Zone + Bal", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_2_Piece_Loading_Zone")),
            new WaitCommand(0.5),
            scoreCube(),
            makeAuto(("Red_2_piece_bal"), kAuto.CHARGE_SPEED),
            new WaitCommand(kAuto.AUTO_BALANCE_WAIT),
            new AutoBalance()
        ));

        autoChooser.addOption("3 Piece Loading Zone", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("2_Piece_Loading_Zone")),
            new WaitCommand(0.5),
            scoreCube(),
            makeAuto(("3_Piece_Loading_Zone")),
            new WaitCommand(0.2),
            scoreCube()
        ));

        autoChooser.addOption("Red 3 Piece Loading Zone", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_2_Piece_Loading_Zone")),
            new WaitCommand(0.3),
            scoreCube(),
            makeAuto(("Red_3_Piece_Loading_Zone")),
            new WaitCommand(0.2),
            scoreCube()
        )); 
        
        autoChooser.addOption("3 Piece Loading Zone L2", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("2_Piece_Loading_Zone")),
            new WaitCommand(0.7),
            scoreCube(),
            makeAuto(("3_Piece_Loading_Zone_L2")),
            new WaitCommand(0.2),
            scoreCube()
        ));
        
        autoChooser.addOption("Red 3 Piece Loading Zone L2", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_2_Piece_Loading_Zone")),
            new WaitCommand(0.7),
            scoreCube(),
            makeAuto(("Red_3_Piece_Loading_Zone_L2")),
            new WaitCommand(0.2),
            scoreCube()
        ));

        autoChooser.addOption("1 piece + Bal", new SequentialCommandGroup(
            scoreCone(),
            arm.moveArm(ArmPos.LOWERED),
            makeAuto(("Charge1"), 1.4),
            new WaitCommand(0.5),
            makeAuto(("Charge2"), 3),
            new AutoBalance()
        ));

        autoChooser.addOption("Red 1 piece + Bal", new SequentialCommandGroup(
            scoreCone(),
            arm.moveArm(ArmPos.LOWERED),
            makeAuto(("Red_Charge1"), 1.4),
            new WaitCommand(0.5),
            makeAuto(("Red_Charge2"), 3),
            new AutoBalance()
        ));

        autoChooser.addOption("Burm 1 piece + Bal", new SequentialCommandGroup(
            scoreCone(),
            arm.moveArm(ArmPos.LOWERED),
            makeAuto(("Charge1_burm"), 1.4),
            new WaitCommand(0.5),
            makeAuto(("Charge2_burm"), 3),
            new AutoBalance()
        ));

        autoChooser.addOption("Red Burm 1 piece + Bal", new SequentialCommandGroup(
            scoreCone(),
            arm.moveArm(ArmPos.LOWERED),
            makeAuto(("Red_Charge1_burm"), 1.4),
            new WaitCommand(0.5),
            makeAuto(("Red_Charge2_burm"), 3),
            new AutoBalance()
        ));

        autoChooser.addOption("2 piece burm", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("2_Piece_Burm"), kAuto.BURM_SIDE_SPEED),
            new WaitCommand(0.5),
            scoreCube()
        ));

        autoChooser.addOption("Red 2 piece burm", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_2_Piece_Burm"), kAuto.BURM_SIDE_SPEED),
            new WaitCommand(0.5),
            scoreCube()
        ));

        autoChooser.addOption("2 piece burm + bal", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("2_Piece_Burm"), kAuto.BURM_SIDE_SPEED),
            new WaitCommand(kAuto.CUBE_SCORE_WAIT),
            scoreCube(),
            makeAuto(("2_piece_bal_burm"), kAuto.CHARGE_SPEED),
            new AutoBalance()
        ));

        autoChooser.addOption("Red 2 piece burm + bal", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_2_Piece_Burm"), kAuto.BURM_SIDE_SPEED),
            new WaitCommand(kAuto.CUBE_SCORE_WAIT),
            scoreCube(),
            makeAuto(("Red_2_piece_bal_burm"), kAuto.CHARGE_SPEED),
            new AutoBalance()
        ));

        autoChooser.addOption("1 piece", scoreCone());

        autoChooser.addOption("Red 2.5 Piece Burm", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_2_Piece_Burm"), kAuto.BURM_SIDE_SPEED),
            new WaitCommand(kAuto.CUBE_SCORE_WAIT),
            scoreCube(),
            makeAuto(("Red_Get_Piece_Burm"), kAuto.BURM_SIDE_SPEED)
        ));

        autoChooser.addOption("2.5 Piece Burm", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("2_Piece_Burm"), kAuto.BURM_SIDE_SPEED),
            new WaitCommand(kAuto.CUBE_SCORE_WAIT),
            scoreCube(),
            makeAuto(("Get_Piece_Burm"), kAuto.BURM_SIDE_SPEED)
        ));

        autoChooser.addOption("2.5 Piece Burm + Bal", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("2_Piece_Burm"), 2.2),
            new WaitCommand(0.3),
            scoreCube(),
            makeAuto(("Get_Piece_Burm"), 2.2),
            makeAuto(("Reverse_Charge_Burm"), 2.2),
            new AutoBalance()
        ));

        autoChooser.addOption("Red 2.5 Piece Burm + Bal", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_2_Piece_Burm"), 2.2),
            new WaitCommand(0.3),
            scoreCube(),
            makeAuto(("Red_Get_Piece_Burm"), 2.2),
            makeAuto(("Red_Reverse_Charge_Burm"), 2.2),
            new AutoBalance()
        ));

        autoChooser.addOption("Red 2.5 Piece + Bal", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_2_Piece_Loading_Zone")),
            new WaitCommand(0.3),
            scoreCube(),
            makeAuto(("Red_Get_Piece")),
            makeAuto(("Red_Reverse_Charge"), 2.2),
            new AutoBalance()
        ));

        autoChooser.addOption("2.5 Piece + Bal", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("2_Piece_Loading_Zone")),
            new WaitCommand(0.3),
            scoreCube(),
            makeAuto(("Get_Piece")),
            makeAuto(("Reverse_Charge"), 2.2),
            new AutoBalance()
        ));

        autoChooser.addOption("Red 3 Piece Burm", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("Red_3_Piece_Burm1"), 2.2),
            new SequentialCommandGroup(
                arm.moveArm(ArmPos.AUTO_DROP),
                manipulator.cubeReverse(),
                new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME)
            ),
            makeAuto("Red_3_Piece_Burm2", 2.2),
            new WaitCommand(0.5),
            scoreCube()
        ));

        autoChooser.addOption("3 Piece Burm", new SequentialCommandGroup(
            scoreCone(),
            makeAuto(("3_Piece_Burm1"), 2.2),
            new SequentialCommandGroup(
                arm.moveArm(ArmPos.AUTO_DROP),
                manipulator.cubeReverse(),
                new WaitCommand(kCommandTimmings.MANIPULATOR_WAIT_TIME)
            ),
            makeAuto("3_Piece_Burm2", 2.2),
            new WaitCommand(0.5),
            scoreCube()
        ));

        putAutoChooser();
    }

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
            manipulator.cubeReverse(),
            new WaitCommand(0.3)
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
}
