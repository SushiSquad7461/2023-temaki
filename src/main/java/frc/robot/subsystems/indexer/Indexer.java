package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Defines an interface for a 2023 indexer.
 */
public abstract class Indexer extends SubsystemBase {
    /**
     * Runs indexer in positive direction.
     */
    public abstract Command runIndexer();

    /**
     * Stops indexer.
     */
    public abstract Command stopIndexer();

    /**
     * Reverses indexer.
     */
    public abstract Command reverseIndexer();

    public abstract void registerMotors();
}
