package frc.robot.subsystems;

import SushiFrcLib.Sensors.gyro.Pigeon;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kAutoAlign;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kVision;
import frc.robot.util.SwerveModule;
import frc.robot.util.Vision;
import frc.robot.util.VisionMeasurement;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/**
 * Class that controls falcon swerve drivetrain.
 */
public class Swerve extends SubsystemBase {
    private final SwerveDrivePoseEstimator swerveOdometry;
    private final SwerveModule[] swerveMods;
    private final Pigeon gyro;
    private final Field2d field;
    
    private static Swerve instance;

    private boolean locationLock;
    private PIDController locationLockPID;

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
        gyro = new Pigeon(kPorts.PIGEON_ID, kSwerve.GYRO_INVERSION, kPorts.PIGEON_CANIVORE_NAME);
        gyro.zeroGyro();
        
        field = new Field2d();

        field.getObject("April Tag Layout").setPoses(
            kVision.APRIL_TAG_FIELD_LAYOUT.getTags().stream().map(
                (tag) -> tag.pose.toPose2d()
            )
            .collect(Collectors.toList())
        );

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

        locationLock = false;
        locationLockPID = new PIDController(0.1d, 0, 0);

        SmartDashboard.putData("Field", field);
    }

    public Pigeon getGyro() {
        return gyro;
    }

    public void turnOnLocationLock(double angle) {
        var table = NetworkTableInstance.getDefault().getTable("FMSInfo");
        boolean isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);

        if (isRedAlliance) {
            angle += 180;
        }

        locationLock = true;
        locationLockPID.setSetpoint(angle);
        locationLockPID.calculate(gyro.getAngle().getDegrees());
    }

    public void turnOfLocationLock() {
        locationLock = false;
    }

    /**
     * Returns a command that will drive the specified offset from the given 
     * April Tag.
     *
     * @param tagOffset The offset of the tag in tag space (x+ away from
     *      tag, y+ left from tag). A null value will default to 1 meter in front of
     *      the target.
     */
    public Command moveToAprilTag(int tagID, Translation2d tagOffset) {
        return moveToPose(
            () -> kVision.APRIL_TAG_FIELD_LAYOUT.getTagPose(tagID).get().toPose2d(),
            tagOffset
        );
    }

    public Command moveToDoubleSuby(Translation2d tagOffset) {
        var table = NetworkTableInstance.getDefault().getTable("FMSInfo");
        boolean isRedAlliance = table.getEntry("IsRedAlliance").getBoolean(true);
        return moveToAprilTag(isRedAlliance ? 7 : 4, tagOffset);
    }

    /**
     * Returns a command that will drive the specified offset from the nearest 
     * April Tag.
     *
     * @param tagOffset The offset of the tag in tag space (x+ away from
     *      tag, y+ left from tag). A null value will default to 1 meter in front of
     *      the target.
     */
    public Command moveToNearestAprilTag(Translation2d tagOffset) {
        return moveToPose(() -> getClosestAprilTag(), tagOffset);
    }

    private Pose2d getClosestAprilTag() {
        Translation2d currentTranslation = swerveOdometry
            .getEstimatedPosition()
            .getTranslation(); 

        SmartDashboard.putNumber("Robot x", currentTranslation.getX());
        SmartDashboard.putNumber("Roybot y", currentTranslation.getY());


        double minDistance = Double.MAX_VALUE;
        AprilTag closestTag = null;

        for (AprilTag tag : Constants.kVision.APRIL_TAG_FIELD_LAYOUT.getTags()) {
            Translation2d tagTranslation = tag.pose.toPose2d().getTranslation();
            double distanceToTag = currentTranslation.getDistance(tagTranslation);

            SmartDashboard.putNumber("Tag Distance" + tag.ID, distanceToTag);

            if (distanceToTag < minDistance) {
                minDistance = distanceToTag;
                closestTag = tag;
            }
        }

        return closestTag.pose.toPose2d(); 
    }

    /**
     * Returns a command that will drive the specified offset from the given
     * pose.
     *
     * @param poseSupplier A Supplier that returns the field relative pose to 
     *      drive to.
     * @param offset The offset of the tag in tag space (x+ away from
     *      tag, y+ left from tag). A null value will default to 1 meter in front of
     *      the target.
     */
    public Command moveToPose(Supplier<Pose2d> poseSupplier, Translation2d newOffset) {
        PIDController yaxisPid = new PIDController(
            kAutoAlign.Y_P, 
            kAutoAlign.Y_I, 
            kAutoAlign.Y_D
        );

        PIDController xaxisPid = new PIDController(
            kAutoAlign.X_P, 
            kAutoAlign.X_I, 
            kAutoAlign.X_D
        );

        PIDController thetaPid = new PIDController(
            kAutoAlign.THETA_P, 
            kAutoAlign.THETA_I, 
            kAutoAlign.THETA_D
        );

        thetaPid.enableContinuousInput(0, 2 * Math.PI);

        xaxisPid.setTolerance(kAutoAlign.X_TOLLERENCE);
        yaxisPid.setTolerance(kAutoAlign.Y_TOLLERENCE);
        thetaPid.setTolerance(kAutoAlign.THETA_TOLLERENCE);

        return runOnce(() -> {
            xaxisPid.calculate(swerveOdometry.getEstimatedPosition().getX());
            yaxisPid.calculate(swerveOdometry.getEstimatedPosition().getY());
            thetaPid.calculate(
                        swerveOdometry.getEstimatedPosition().getRotation().getRadians()
            );

            SmartDashboard.putNumber("In Auto Align", 1);
            Translation2d offset = newOffset;
            // Give offset a default value
            if (offset == null) {
                offset = kAutoAlign.DEFAULT_OFFSET;
            }

            // Get forward vector of pose and add it to offset
            Pose2d pose = poseSupplier.get();
            Rotation2d targetRot = pose.getRotation();
            offset = offset.rotateBy(targetRot);
            Translation2d targetTrans = pose.getTranslation();
            Translation2d offsetTarget = targetTrans.plus(offset);
            field.getObject("target").setPose(new Pose2d(offsetTarget, targetRot));

            // Set pid setpoints
            xaxisPid.setSetpoint(offsetTarget.getX());
            yaxisPid.setSetpoint(offsetTarget.getY());

            // Invert theta to ensure we're facing towards the target
            thetaPid.setSetpoint(targetRot.minus(kAutoAlign.DEFAULT_ROTATION).getRadians());
        }).andThen(run(
            () -> {
                SmartDashboard.putNumber("x tolerance", xaxisPid.getPositionError());
                SmartDashboard.putNumber("y tolerance", yaxisPid.getPositionError());
                SmartDashboard.putNumber("theta tolerance", thetaPid.getPositionError());

                drive(
                    new Translation2d(
                        xaxisPid.calculate(swerveOdometry.getEstimatedPosition().getX()),
                        yaxisPid.calculate(swerveOdometry.getEstimatedPosition().getY())
                    ),
                    thetaPid.calculate(
                        swerveOdometry.getEstimatedPosition().getRotation().getRadians()
                    ),
                    true, 
                    false
                );
            }
        )).until(
            () -> xaxisPid.atSetpoint() && yaxisPid.atSetpoint() && thetaPid.atSetpoint()
        ).andThen(
            () -> { 
                SmartDashboard.putNumber("In Auto Align", 0);

                xaxisPid.close(); 
                yaxisPid.close(); 
                thetaPid.close(); 
            }
        );
    }

    public void drive(Translation2d translation, 
        double rotation, boolean fieldRelative, boolean isOpenLoop
    ) {
        if (locationLock) {
            rotation = locationLockPID.calculate(gyro.getAngle().getDegrees());
        }
        driveWithLocationLock(translation, rotation, fieldRelative, isOpenLoop);
    }

    /**
     * Updates the swerve module values for the swerve.
     */
    private void driveWithLocationLock(Translation2d translation, 
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

    /**
     * Resets the odom.
     */
    public Command resetOdometryToBestAprilTag() {
        return runOnce(() -> {
            VisionMeasurement measurement = Vision.getVision().getBestMeasurement();
            SmartDashboard.putBoolean("Running", measurement == null);
            if (measurement != null) {
                this.resetOdometryAndGyro(measurement.robotPose);
            }
        });
    }

    public void resetOdometryAndGyro(Pose2d pose) {
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

        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle().getDegrees());

        for (SwerveModule m : swerveMods) {
            SmartDashboard.putNumber("Module Angle " + m.moduleNumber, m.getDriveSpeed());
        }   

        // Loop through all measurements and add it to pose estimator
        List<VisionMeasurement> measurements = Vision.getVision().getMeasurements();

        field.getObject("best")
            .setPoses(measurements.stream().map(
                (measurement) -> measurement.robotPose
            )
            .collect(Collectors.toList()));

        if (measurements != null) {
            for (VisionMeasurement measurement : measurements) {
                // Skip measurement if it's more than a meter away
                if (measurement.robotPose.getTranslation().getDistance(swerveOdometry.getEstimatedPosition().getTranslation()) > 1.0 && measurement.ambiguity > 0.1) {
                    continue;
                }
        
                swerveOdometry.addVisionMeasurement(
                    measurement.robotPose,
                    measurement.timestampSeconds,
                    kSwerve.VISION_STANDARD_DEVIATION
                );
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