package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kSwerve;
import frc.robot.util.SwerveModule;

/**
 * Class that controls falcon swerve drivetrain.
 */
public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] swerveMods;
    private final Pigeon2 gyro;
    private final Field2d field;

    private static Swerve instance;

    /**
     * singleton get instance method.
     */
    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
        gyro = new Pigeon2(kPorts.PIGEON_ID);
        gyro.configFactoryDefault();
        zeroGyro();

        field = new Field2d();

        swerveMods = new SwerveModule[] {
            new SwerveModule(0, kSwerve.Mod0.CONSTANTS),
            new SwerveModule(1, kSwerve.Mod1.CONSTANTS),
            new SwerveModule(2, kSwerve.Mod2.CONSTANTS),
            new SwerveModule(3, kSwerve.Mod3.CONSTANTS)
        };

        swerveOdometry = new SwerveDriveOdometry(
            kSwerve.SWERVE_KINEMATICS, 
            Rotation2d.fromDegrees(0),
            getPositions()
        );

        SmartDashboard.putData("Field", field);
    }

    /**
     * Updates the swerve module values for the swerve.
     */
    public void drive(Translation2d translation, 
        double rotation, boolean fieldRelative, boolean isOpenLoop
    ) {
        SwerveModuleState[] swerveModuleStates = kSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getYaw()
            ) : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
            )
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kSwerve.MAX_SPEED);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /**
     * Used by SwerveControllerCommand in Auto.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kSwerve.MAX_SPEED);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        gyro.setYaw(pose.getRotation().getDegrees());
        swerveOdometry.resetPosition(pose.getRotation(), getPositions(), pose);
    }

    /**
     * Returns current swerve module states.
     */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }

        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    /**
     *  Gets the current rotation of the robot based on gyro.
     */
    public Rotation2d getYaw() {
        double yaw = gyro.getYaw();

        return kSwerve.GYRO_INVERSION 
            ? Rotation2d.fromDegrees(360 - yaw)
            : Rotation2d.fromDegrees(yaw);
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getPositions());
        field.setRobotPose(swerveOdometry.getPoseMeters());

        SmartDashboard.putNumber("gyro", getYaw().getDegrees());

        for (SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", 
                mod.getCanCoder().getDegrees()
            );
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getAngle());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
                mod.getState().angle.getDegrees()
            );
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getDriveSpeed());
        }
    }

    /**
     * Updates all encoders based on cancoder values. Done to reduce false readings.
     */
    public void updateEncoders() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }
}