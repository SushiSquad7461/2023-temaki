package frc.robot.subsystems.manipulator;

/**
 * Controls the manipulator subsytem.
 */
public class AlphaManipulator extends Manipulator {
    private static AlphaManipulator instance;

    /**
     * Gets the current manipular instance. for singleton support.
     */
    public static AlphaManipulator getInstance() {
        if (instance == null) {
            instance = new AlphaManipulator();
        }
        return instance;
    }

    private AlphaManipulator() {
        super();
    }

}
