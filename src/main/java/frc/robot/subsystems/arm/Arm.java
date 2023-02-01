package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Defines arm ABC for 2023 robots.
 */
public abstract class Arm extends SubsystemBase {
    /**
     * Gets through bore encoder position.
     *
     * @return position in degrees
     */
    public abstract double getAbsolutePosition();

    /**
     * Sets velocity of the of the arm motors.
     *
     * @param speed from -1.0 to 1.0
     */
    public abstract void runArm(double speed);

    /**
     * Stops the arm motors.
     */
    public abstract void stopArm();

    /**
     * Sets arm to a certain position.
     *
     * @param degrees target position of the arm
     */
    public abstract void setPosition(double degrees);

    /**
     * Sychronize motor encoder and through bore encoder readings, resets the arm to 0 degree.
     */
    public abstract void resetArm();
}
