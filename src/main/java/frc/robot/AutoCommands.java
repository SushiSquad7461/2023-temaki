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
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;

/**
 * This class generates auto commands.
 */
public class AutoCommands {
    private final Swerve swerve;
    public final Map<String, SequentialCommandGroup> autos;

    private final SendableChooser<SequentialCommandGroup> autoChooser;

    /**
     * Define all auto commands.
     */
    public AutoCommands(Swerve swerve) {
        this.swerve = swerve;

        autos = new HashMap<String, SequentialCommandGroup>();
        autoChooser = new SendableChooser<>();

        autos.put("nothing", new SequentialCommandGroup(new InstantCommand(() -> {
            System.out.println("YOUR A CLOWN");
        })));

        autos.put("R2CubeLZ", new SequentialCommandGroup(
            getCommand("R_CubeToGridLZ", true),
            getCommand("R_GridToCubeLZ", false)   
        ));

        putAutoChooser();
    }

    private void putAutoChooser() {
        Set<String> keys = autos.keySet();
        autoChooser.setDefaultOption(
            (String) keys.toArray()[0], 
            autos.get(keys.toArray()[0])
        );
        keys.remove((String) keys.toArray()[0]);
    
        for (String i : autos.keySet()) {
            autoChooser.addOption(i, autos.get(i));
        }
    
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
