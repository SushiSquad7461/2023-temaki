package frc.robot.commands;

import SushiFrcLib.Math.Normalization;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;

/**
 * Command that controls teleop swerve.
 */
public class TeleopSwerveDrive extends CommandBase {
    private final Swerve swerve;
    private final GenericHID controller;
    private final int translationAxis;
    private final int strafeAxis;
    private final int rotationsAxis;
    private final boolean fieldRelative;
    private final boolean openLoop;

    /**
     * Set swerve subsytem, controlers, axis's, and other swerve paramaters.
     */
    public TeleopSwerveDrive(Swerve swerve, GenericHID controller, 
        int translationAxis, int strafeAxis, int rotationsAxis, 
        boolean fieldRelative, boolean openLoop
    ) {
        this.swerve = swerve;
        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationsAxis = rotationsAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double forwardBack = controller.getRawAxis(translationAxis);
        double leftRight = controller.getRawAxis(strafeAxis);
        double rot = -controller.getRawAxis(rotationsAxis);

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
