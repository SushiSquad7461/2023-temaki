package frc.robot.subsystems;

import SushiFrcLib.Motor.MotorHelper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIndexer;
import frc.robot.Constants.kPorts;

/**
 * Controls indexer subsytem.
 */
public class Indexer extends SubsystemBase {
    private final CANSparkMax indexerMotor;

    private static Indexer instance;

    /**
     * singleton get instance method.
     */
    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    private Indexer() {
        indexerMotor = MotorHelper.createSparkMax(kPorts.INDEXER_MOTOR, MotorType.kBrushless);
    }

    /**
     * Runs indexer in positive direction.
     */
    public Command runIndexer() {
        return runOnce(() -> {
            indexerMotor.set(kIndexer.SPEED);
        });
    }

    /**
     * Stops indexer.
     */
    public Command stopIndexer() {
        return runOnce(() -> {
            indexerMotor.set(0);
        });
    }

    /**
     * Reverses indexer.
     */
    public Command reverseIndexer() {
        return runOnce(() -> {
            indexerMotor.set(kIndexer.SPEED * -1);
        });
    }

    @Override
    public
    void periodic() {
        indexerMotor.set(kIndexer.SPEED);

    }
}
