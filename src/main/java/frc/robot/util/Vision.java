package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision {
    private static Vision instance;
    
    public final PhotonCamera camera;

    private final NetworkTableEntry rawBytesEntry;

    private double lastUpdateTimeMicro;

    private ArrayList<VisionMeasurement> measurements = new ArrayList<>();
    private PhotonTrackedTarget bestTarget;
    private VisionMeasurement bestMeasurement;

    private Vision() {
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

        if (Robot.isReal()) {
            camera = new PhotonCamera("limelight");
        } else {
            // This is required for PV simulation in localhost
            ntInstance.stopServer();
            ntInstance.setServer("localhost");
            ntInstance.startClient3("localhost");
            camera = new PhotonCamera(ntInstance, "Integrated_Webcam");
        }

        rawBytesEntry = ntInstance
            .getTable("photonvision")
            .getSubTable("limelight")
            .getEntry("rawBytes");
    }

    public static Vision getVision() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    public List<VisionMeasurement> getMeasurements() {
        update();
        return measurements;
    }

    public PhotonTrackedTarget getBestTarget() {
        update();
        return bestTarget;
    }

    public VisionMeasurement getBestMeasurement() {
        update();
        return bestMeasurement;
    }
  
    private void update() {
        // Return if no new data is ready
        if (lastUpdateTimeMicro == rawBytesEntry.getLastChange()) {
            return;
        } else {
            lastUpdateTimeMicro = rawBytesEntry.getLastChange();
        }
            PhotonPipelineResult res = camera.getLatestResult();
        if (!res.hasTargets()) {
            measurements.clear();
            bestTarget = null;
            bestMeasurement = null;
            return;
        }
        
        // Get best target and the pose from that target
        bestTarget = res.getBestTarget();
        Pose3d bestPose = getRobotPoseFromTarget(bestTarget);
        SmartDashboard.putBoolean("Best target", bestPose == null);

        if (bestPose == null) {
            // If no pose, then no measurement
            bestMeasurement = null;
        } else {
            // Else populate a measurement
            bestMeasurement = new VisionMeasurement(
                bestTarget,
                new Pose2d(
                    bestPose.getX(),
                    bestPose.getY(),
                    new Rotation2d(bestPose.getRotation().getZ())), 
                res.getTimestampSeconds(),
                bestTarget.getPoseAmbiguity()); 
        }

        measurements.clear();
        // Loop through all targets
        for (PhotonTrackedTarget target : res.targets) {
            // Skip anything with too high of an ambiguity
            if (target.getPoseAmbiguity() > 0.2) {
                continue;
            }

            // Get pose
            Pose3d estRobotPose = getRobotPoseFromTarget(target);
            if (estRobotPose == null) {
                continue;
            }

            // Add to measurement
            measurements.add(new VisionMeasurement(
                target,
                new Pose2d(
                    estRobotPose.getX(),
                    estRobotPose.getY(),
                    new Rotation2d(estRobotPose.getRotation().getZ())), 
                res.getTimestampSeconds(),
                target.getPoseAmbiguity())); 
        }
    }

    public static Pose3d getRobotPoseFromTarget(PhotonTrackedTarget target) {
        // Get transform that converts from camera pose to target pose
        Transform3d cameraToTarget = target.getBestCameraToTarget();

        // Get the april tag's pose on the field
        Optional<Pose3d> feducialPos = Constants.kVision.APRIL_TAG_FIELD_LAYOUT
            .getTagPose(target.getFiducialId());

        // If we don't have a pose for that apriltag id, then skip
        if (feducialPos.isEmpty()) {
            return null;
        }

        return PhotonUtils.estimateFieldToRobotAprilTag(
            cameraToTarget,
            feducialPos.get(), 
            Constants.kVision.CAMERA_TO_ROBOT_METERS_DEGREES);
    }
}
