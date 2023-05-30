package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Class to hold vision measurments.
 */
public class VisionMeasurement {
    public final PhotonTrackedTarget target;
    public final Pose2d robotPose;
    public final double timestampSeconds;
    public final double ambiguity;
   
    /**
     * Construct class.
     */
    public VisionMeasurement(
        PhotonTrackedTarget target, 
        Pose2d robotPose, 
        double timestampSeconds, 
        double ambiguity
    ) {
        this.target = target;
        this.robotPose = robotPose;
        this.timestampSeconds = timestampSeconds;
        this.ambiguity = ambiguity;
    }
}
