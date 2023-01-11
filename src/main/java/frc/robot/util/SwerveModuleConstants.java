package frc.robot.util;

/**
 * Wrapper class for swerve module constants.
 */
public class SwerveModuleConstants {
    public final int driveMotorId;
    public final int angleMotorId;
    public final int cancoderId;
    public final double angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     */
    public SwerveModuleConstants(
        int driveMotorId, int angleMotorId, int canCoderId, double angleOffset
    ) {
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.cancoderId = canCoderId;
        this.angleOffset = angleOffset;
    }
}
