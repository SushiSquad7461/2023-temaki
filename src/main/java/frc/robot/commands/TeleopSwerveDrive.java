package frc.robot.commands;

import SushiFrcLib.Math.Normalization;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

/**
 * Command that controls teleop swerve.
 */
public class TeleopSwerveDrive extends CommandBase {
    private final Swerve swerve;
    private final boolean fieldRelative;
    private final boolean openLoop;

    private final Supplier<Double> xaxisSupplier; 
    private final Supplier<Double> yaxisSupplier;
    private final Supplier<Double> rotSupplier;

    /**
     * Set swerve subsytem, controlers, axis's, and other swerve paramaters.
     */
    public TeleopSwerveDrive(Swerve swerve,
        Supplier<Double> xaxisSupplier, 
        Supplier<Double> yaxisSupplier, 
        Supplier<Double> rotSupplier,
        boolean fieldRelative,
        boolean openLoop
    ) {
        this.swerve = swerve;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        this.xaxisSupplier = xaxisSupplier;
        this.yaxisSupplier = yaxisSupplier;
        this.rotSupplier = rotSupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double forwardBack = yaxisSupplier.get();
        double leftRight = xaxisSupplier.get();
        double rot = rotSupplier.get();

        forwardBack = Normalization.cube(
            Math.abs(forwardBack) < Constants.STICK_DEADBAND ? 0 : forwardBack
        );

        leftRight = Normalization.cube(
            Math.abs(leftRight) < Constants.STICK_DEADBAND ? 0 : leftRight
        );

        Translation2d translation = new Translation2d(forwardBack, leftRight)
                .times(kSwerve.MAX_SPEED);

        rot = Normalization.cube(rot);
        rot *= kSwerve.MAX_ANGULAR_VELOCITY;

        swerve.drive(translation, rot, fieldRelative, openLoop);
    }
}
