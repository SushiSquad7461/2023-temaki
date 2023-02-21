package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kArm.ArmPos;

/**
 * Defines arm ABC for 2023 robots.
 */
public abstract class Arm extends SubsystemBase {
    /**
     * Gets through bore encoder position.
     *
     * @return position in degrees
     */
    // public abstract double getAbsolutePosition();

    /**
     * Sets arm to a certain position.
     *
     * @param degrees target position of the arm
     */
    public abstract void setPosition(double degrees);

    /**
     * Moves arm to a certain position.
     */
    public abstract Command moveArm(ArmPos angle);

    /**
     * Sychronize motor encoder and through bore encoder readings, resets the arm to 0 degree.
     */
    public abstract void resetArm();

}
