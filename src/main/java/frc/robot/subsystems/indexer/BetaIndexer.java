package frc.robot.subsystems.indexer;

import SushiFrcLib.Motor.MotorHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kIndexer;
import frc.robot.Constants.kPorts;

/**
 * Implements a indexer for temaki.
 */
public class BetaIndexer extends Indexer {
    private final CANSparkMax indexerMotor;
    private final CANSparkMax coneRamp;

    private static BetaIndexer instance;

    /**
     * singleton get instance method.
     */
    public static BetaIndexer getInstance() {
        if (instance == null) {
            instance = new BetaIndexer();
        }
        return instance;
    }

    private BetaIndexer() {
        indexerMotor = MotorHelper.createSparkMax(kPorts.INDEXER_MOTOR, MotorType.kBrushless);
        coneRamp = MotorHelper.createSparkMax(kPorts.CONE_RAMP_MOTOR, MotorType.kBrushless);
    }

    /**
     * Runs indexer in positive direction.
     */
    @Override
    public Command runIndexer() {
        return runOnce(() -> {
            indexerMotor.set(kIndexer.INDEXER_SPEED);
            coneRamp.set(kIndexer.CONE_RAMP_SPEED);
        });
    }

    /**
     * Stops indexer.
     */
    @Override
    public Command stopIndexer() {
        return runOnce(() -> {
            indexerMotor.set(0);
            coneRamp.set(0);
        });
    }

    /**
     * Reverses indexer.
     */
    @Override
    public Command reverseIndexer() {
        return runOnce(() -> {
            indexerMotor.set(kIndexer.INDEXER_SPEED * -1);
            coneRamp.set(kIndexer.CONE_RAMP_SPEED * -1);
        });
    }
}
