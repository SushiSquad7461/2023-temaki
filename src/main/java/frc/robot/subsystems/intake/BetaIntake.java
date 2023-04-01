package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kIntake;
import frc.robot.Constants.kPorts;

/**
 * Class that controls a cube intake.
 */
public class BetaIntake extends Intake {
    private CANSparkMax bottom;
    private CANSparkMax top;

    private static BetaIntake instance;

    /**
     * Gets instance for singleton.
     */
    public static BetaIntake getInstance() {
        if (instance == null) {
            instance = new BetaIntake();
        }
        return instance;
    }


    private BetaIntake() {
        super();
        bottom = new CANSparkMax(kPorts.INTAKE_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        bottom.setInverted(true);
        bottom.setSmartCurrentLimit(kIntake.CURRENT_LIMIT);
        bottom.burnFlash();

        top = new CANSparkMax(kPorts.INTAKE_TOP_MOTOR_ID, MotorType.kBrushless);
        top.setInverted(false);
        top.setSmartCurrentLimit(kIntake.CURRENT_LIMIT);
        top.burnFlash();
    }

    /**
     * Makes sure intake is extended and turns on motor.
     */
    public Command runIntake() {
        return runOnce(() -> {
            bottom.set(kIntake.MOTOR_SPEED);
            top.set(kIntake.MOTOR_SPEED/1.5); //half to maintain tangential velocity
        });
    }

    /**
     * Makes sure intake is retracted and turns of motor. 
     */
    public Command stopIntake() {
        return runOnce(() -> {
            bottom.set(0);
            top.set(0);
        });
    }

    /**
     * Reverses intake.
     */
    public Command reverseIntake() {
        return runOnce(() -> {
            bottom.set(-kIntake.MOTOR_SPEED);
            top.set(-kIntake.MOTOR_SPEED/1.5);
        });
    }
   
    /**
     * Intake cones using cone ramp.
     */
    public Command coneIntake() {
        return runOnce(() -> {
            bottom.set(-kIntake.MOTOR_SPEED);
            top.set(kIntake.MOTOR_SPEED/1.5);
        });
    }

    /**
     * Shoot out cubes.
     */
    public Command cubeShoot() {
        return runOnce(() -> {
            bottom.set(1.0);
            top.set(-0.6);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Top intake current", top.getOutputCurrent());
    }

}

