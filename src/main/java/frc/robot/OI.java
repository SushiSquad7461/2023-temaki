package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kOI;

public class OI {
    private CommandXboxController driverController;

    private static OI instance;

    /**
     * Makes class a singelton.
     */
    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    private OI() {
        driverController = new CommandXboxController(kOI.DRIVE_PORT);
    }

    public double getDriveTrainRotation() {
        return driverController.getHID().getRawAxis(kOI.DRIVE_ROTATE) * -1;
    }

    public double getDriveTrainTranslationY() {
        return driverController.getHID().getRawAxis(kOI.DRIVE_TRANSLATION_Y) * -1;
    }

    public double getDriveTrainTranslationX() {
        return driverController.getHID().getRawAxis(kOI.DRIVE_TRANSLATION_X) * -1;
    }
}
