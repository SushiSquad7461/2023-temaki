package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import SushiFrcLib.Sensors.gyro.Pigeon;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kSwerve;
import frc.robot.util.SwerveModule;
import frc.robot.util.Vision;
import frc.robot.util.VisionMeasurement;
import frc.robot.util.Vision;
import frc.robot.util.VisionMeasurement;

/**
 * Class that controls falcon swerve drivetrain.
 */
public class Swerve extends SubsystemBase {
    private final SwerveDrivePoseEstimator swerveOdometry;
    private final SwerveModule[] swerveMods;
    private final Pigeon gyro;
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
        gyro = new Pigeon(kPorts.PIGEON_ID, kSwerve.GYRO_INVERSION);
        gyro.zeroGyro();

        field = new Field2d();

        swerveMods = new SwerveModule[] {
            new SwerveModule(0, kSwerve.Mod0.CONSTANTS),
            new SwerveModule(1, kSwerve.Mod1.CONSTANTS),
            new SwerveModule(2, kSwerve.Mod2.CONSTANTS),
            new SwerveModule(3, kSwerve.Mod3.CONSTANTS)
        };

        swerveOdometry = new SwerveDrivePoseEstimator(
            kSwerve.SWERVE_KINEMATICS, 
            Rotation2d.fromDegrees(0),
            getPositions(),
            new Pose2d(),
            kSwerve.STATE_STANDARD_DEVIATION,
            kSwerve.VISION_STANDARD_DEVIATION
        );
        SmartDashboard.putData("Field", field);
    }

    /**
     * Returns a command that will drive 1 meter in front of the nearest April
     * Tag.
     */
    public Command moveToNearestAprilTag() {
        return moveToNearestAprilTag(new Translation2d(1, 0));
    }

    /**
     * Returns a command that will drive the specified offset from the nearest 
     * April Tag.
     * 
     * @param tagOffset The offset of the tag in tag space (x+ away from
     * tag, y+ left from tag)
     */
    public Command moveToNearestAprilTag(Translation2d tagOffset) {
        // TODO: Consider creating whole-robot PID constants
        PIDController yPid = new PIDController(10, 0, 0);
        PIDController xPid = new PIDController(10, 0, 0);
        PIDController thetaPid = new PIDController(12, 0, 0);

        thetaPid.enableContinuousInput(0, 2 * Math.PI);

        // TODO: tune this tolerance
        xPid.setTolerance(0.03);
        yPid.setTolerance(0.03);
        thetaPid.setTolerance(0.03);


        // Get closest tag
        Translation2d currentTranslation = swerveOdometry.getEstimatedPosition().getTranslation(); 
        double minDistance = Double.MAX_VALUE;
        AprilTag closestTag = null;
        for (AprilTag tag : Constants.kVision.APRIL_TAG_FIELD_LAYOUT.getTags()) {
            Translation2d tagTranslation = tag.pose.getTranslation().toTranslation2d();
            double distanceToTag = currentTranslation.getDistance(tagTranslation);
            if (distanceToTag < minDistance) {
                minDistance = distanceToTag;
                closestTag = tag;
            }
        }

        // Get forward vector of tag
        Rotation2d tagRot = closestTag.pose.getRotation().toRotation2d();
        tagOffset = tagOffset.rotateBy(tagRot);
        Translation2d tagTranslation = closestTag.pose.getTranslation().toTranslation2d();
        Translation2d inFrontOfTag = tagTranslation.plus(tagOffset);
        field.getObject("target").setPose(new Pose2d(inFrontOfTag, tagRot.unaryMinus()));

        // Set pid setpoints
        xPid.setSetpoint(inFrontOfTag.getX());
        yPid.setSetpoint(inFrontOfTag.getY());
        thetaPid.setSetpoint(tagRot.unaryMinus().getRadians());

        return run(() -> {
            Pose2d currentPose = swerveOdometry.getEstimatedPosition();

            SmartDashboard.putNumber("x tolerance", xPid.getPositionError());
            SmartDashboard.putNumber("y tolerance", yPid.getPositionError());
            SmartDashboard.putNumber("theta tolerance", thetaPid.getPositionError());

            drive(
                new Translation2d(
                    xPid.calculate(currentPose.getX()),
                    yPid.calculate(currentPose.getY())),
                thetaPid.calculate(currentPose.getRotation().getRadians()),
                true, false);
        }).until(() -> xPid.atSetpoint() && yPid.atSetpoint() && thetaPid.atSetpoint())
        .andThen(() -> { xPid.close(); yPid.close(); thetaPid.close(); });
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
                gyro.getAngle()
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
        return swerveOdometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        gyro.setAngle(pose.getRotation());
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

    /**
     * Get Swerve Module Positions in meters.
     */
    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        
        return positions;
    }


    @Override
    public void periodic() {
        swerveOdometry.update(gyro.getAngle(), getPositions());
        field.setRobotPose(swerveOdometry.getEstimatedPosition());

        // Loop through all measurements and add it to pose estimator
        List<VisionMeasurement> measurements = Vision.getVision().getMeasurements();
        if (measurements != null) {
            for (VisionMeasurement measurement : measurements) {
                // Skip measurement if it's more than a meter away
                if (measurement.robotPose.getTranslation().getDistance(swerveOdometry.getEstimatedPosition().getTranslation()) > 1.0) {
                    continue;
                }
        
                swerveOdometry.addVisionMeasurement(
                    measurement.robotPose,
                    measurement.timestampSeconds,
                    kSwerve.VISION_STANDARD_DEVIATION);//.times(measurement.ambiguity + 0.9)); 
            }
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