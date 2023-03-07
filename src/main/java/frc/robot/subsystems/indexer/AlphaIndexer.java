package frc.robot.subsystems.indexer;

import SushiFrcLib.Motor.MotorHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kIndexer;
import frc.robot.Constants.kPorts;
import frc.robot.subsystems.util.MotorTest;
import frc.robot.subsystems.util.Neo;

/**
 * Controls indexer subsytem.
 */
public class AlphaIndexer extends Indexer {
    private final CANSparkMax indexerMotor;
    private MotorTest motorTest;

    private static AlphaIndexer instance;

    /**
     * singleton get instance method.
     */
    public static AlphaIndexer getInstance() {
        if (instance == null) {
            instance = new AlphaIndexer();
        }
        return instance;
    }

    private AlphaIndexer() {
        indexerMotor = MotorHelper.createSparkMax(kPorts.INDEXER_MOTOR, MotorType.kBrushless);
        Neo neoIndexer = new Neo(indexerMotor);
        motorTest = MotorTest.getInstance();
        motorTest.register(neoIndexer, null, "IndexerSubsystem", "indexer", neoIndexer.canID, neoIndexer.currentLimit, 0);
    }

    /**o
     * Runs indexer in positive direction.
     */
    @Override
    public Command runIndexer() {
        return runOnce(() -> {
            indexerMotor.set(kIndexer.INDEXER_SPEED);
        });
    }

    /**
     * Stops indexer.
     */
    @Override
    public Command stopIndexer() {
        return runOnce(() -> {
            indexerMotor.set(0);
        });
    }

    /**
     * Reverses indexer.
     */
    @Override
    public Command reverseIndexer() {
        return runOnce(() -> {
            indexerMotor.set(kIndexer.INDEXER_SPEED * -1);
        });
    }
}
