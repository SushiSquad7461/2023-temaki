package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kOI;

/** 
 * Handels xbox controllers.
*/
public class OI {
    private CommandXboxController driverController;
    private CommandXboxController operatorController;
    private double[] buttonValues;
    private double[] axisValues;
    private double[] povValues;
    private int povCount;
    private final int povAngleCount = 8;
    private Trigger trigger;

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
        operatorController = new CommandXboxController(kOI.OPERATOR_PORT);

        // get the count of axis, buttons, and povs on all controllers
        XboxController hid = driverController.getHID();
        buttonValues = new double[hid.getButtonCount()];
        axisValues = new double[hid.getAxisCount()];
        povCount = hid.getPOVCount();
        povValues = new double[povAngleCount];
        trigger = new Trigger();
    }

    public double getDriveTrainRotation() {
        return kOI.DRIVE_THETA_LIMITER.calculate(getRawAxis(kOI.DRIVE_ROTATE));
    }

    public double getDriveTrainTranslationY() {
        return kOI.DRIVE_Y_LIMITER.calculate(getRawAxis(kOI.DRIVE_TRANSLATION_Y));
    }

    public double getDriveTrainTranslationX() {
        return kOI.DRIVE_X_LIMITER.calculate(getRawAxis(kOI.DRIVE_TRANSLATION_X));
    }

    private double getRawAxis(int id) {
        return driverController.getHID().getRawAxis(id) * -1;
    }

    public CommandXboxController getDriverController() {
        return driverController;
    }

    public CommandXboxController getOperatorController() {
        return operatorController;
    }

    public Trigger logAllControllers(){
        logController("Driver", driverController);
        logController("Operator", operatorController);

        return trigger;
    }
    
    private void logController(String name, CommandXboxController ctrl){

        for (int i=0; i<buttonValues.length; i++){
            buttonValues[i] = ctrl.button(i).getAsBoolean() ? 1.0: 0.0;
        }
        SmartDashboard.putNumberArray(name+"ControllerButtonInputs",buttonValues);

        for (int i=0; i<axisValues.length; i++){
            axisValues[i] = ctrl.getRawAxis(i);
        }
        SmartDashboard.putNumberArray(name+"ControllerAxisInputs",axisValues);

        for (int i=0; i<povCount; i++){
            for (int j=0; j<axisValues.length; j++){
                axisValues[i] = ctrl.pov(j*45).getAsBoolean() ? 1.0 : 0.0;
            }
            SmartDashboard.putNumberArray(name+"ControllerPov"+i+"Inputs",axisValues);
        }
    }
}
