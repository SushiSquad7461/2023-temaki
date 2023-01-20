package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;
import java.util.function.Consumer;

/**
 * This class generates auto commands.
 */
public class AutoCommands {
    private final Swerve swerve;
    private final SendableChooser<SequentialCommandGroup> autoChooser;

    /**
     * Define all auto commands.
     */
    public AutoCommands(Swerve swerve) {
        this.swerve = swerve;

        autoChooser = new SendableChooser<>();

        autoChooser.addOption("nothing", new SequentialCommandGroup(new InstantCommand(() -> {
            System.out.println("YOUR A CLOWN");
        })));

        autos.put("R2.5CubeLZ", new SequentialCommandGroup(
            getCommand("R_GridToCubeLZ", true),
            getCommand("R_CubeToGridLZ", false),
            getCommand("R_GridToCube2LZ", false)   
        ));

        autos.put("R2Cube+ChargeLZ", new SequentialCommandGroup(
            getCommand("R_GridToCubeLZ", true),
            getCommand("R_CubeToGridLZ", false),
            getCommand("R_GridToChargeLZ", false)
        ));

        autos.put("RCharge", new SequentialCommandGroup(
           getCommand("R_StartToCharge", true) 
        ));

        putAutoChooser();
    }

    private void putAutoChooser() {
        SmartDashboard.putData("Auto Selector", autoChooser); 
    }

    /**
     * Get currently selected auto.
     */
    public SequentialCommandGroup getAuto() {
        return autoChooser.getSelected();
    }

    private Command getCommand(String pathName, boolean isFirstPath) {
        PathPlannerTrajectory path = PathPlanner.loadPath(
                pathName,
                kSwerve.MAX_SPEED,
                kSwerve.MAX_ACCELERATION);

        Consumer<SwerveModuleState[]> display = s -> swerve.setModuleStates(s);

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if (isFirstPath) {
                        swerve.resetOdometry(getInitialPose(path));
                    }
                }, swerve),
                new PPSwerveControllerCommand(
                        path,
                        swerve::getPose,
                        kSwerve.SWERVE_KINEMATICS,
                        kSwerve.X_CONTROLLER,
                        kSwerve.Y_CONTROLLER,
                        kSwerve.ANGLE_CONTROLLER,
                        display,
                        swerve),
                new InstantCommand(() -> {
                    swerve.drive(new Translation2d(0, 0), 0, true, false);
                }
                )
        );
    }

    private Pose2d getInitialPose(PathPlannerTrajectory path) {
        return new Pose2d(
                path.getInitialPose().getTranslation(),
                path.getInitialState().holonomicRotation
        );
    }
}
