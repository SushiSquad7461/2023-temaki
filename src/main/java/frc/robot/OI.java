package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kOI;

/** 
 * Handels xbox controllers.
*/
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
        return getRawAxis(kOI.DRIVE_ROTATE);
    }

    public double getDriveTrainTranslationY() {
        return getRawAxis(kOI.DRIVE_TRANSLATION_Y);
    }

    public double getDriveTrainTranslationX() {
        return getRawAxis(kOI.DRIVE_TRANSLATION_X);
    }

    private double getRawAxis(int id) {
        return driverController.getHID().getRawAxis(id) * -1;
    }
}
