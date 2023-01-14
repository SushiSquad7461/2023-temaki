package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.kPorts;
import frc.robot.Constants.kSwerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.SmartDashboard.TunableNumber;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;

    private final TunableNumber mDriveP;
    private final TunableNumber mDriveD;
    private final TunableNumber mDriveF;
    private final TunableNumber mAngleP;
    private final TunableNumber mAngleD;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new WPI_CANCoder(moduleConstants.cancoderID, kPorts.CANIVORE_NAME);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = MotorHelper.createFalconMotor(moduleConstants.angleMotorID, kSwerve.ANGLE_CURRENT_LIMIT,
                kSwerve.ANGLE_INVERSION, kSwerve.ANGLE_NEUTRAL_MODE, kSwerve.ANGLE_P, kSwerve.ANGLE_I, kSwerve.ANGLE_D,
                kSwerve.ANGLE_F,
                kPorts.CANIVORE_NAME, SensorInitializationStrategy.BootToZero);
        resetToAbsolute();

        /* Drive Motor Config */
        driveMotor = MotorHelper.createFalconMotor(moduleConstants.driveMotorID, kSwerve.DRIVE_CURRENT_LIMIT,
                kSwerve.DRIVE_INVERSION, kSwerve.DRIVE_NEUTRAL_MODE, kSwerve.DRIVE_P, kSwerve.DRIVE_I, kSwerve.DRIVE_D,
                kSwerve.DRIVE_F,
                kPorts.CANIVORE_NAME, SensorInitializationStrategy.BootToZero, kSwerve.OPEN_LOOP_RAMP);
        driveMotor.setSelectedSensorPosition(0);

        lastAngle = getState().angle.getDegrees();

        mDriveP = new TunableNumber("Mod " + moduleNumber + " drive P", kSwerve.DRIVE_P, Constants.TUNING_MODE);
        mDriveD = new TunableNumber("Mod " + moduleNumber + " drive D", kSwerve.DRIVE_D, Constants.TUNING_MODE);
        mDriveF = new TunableNumber("Mod " + moduleNumber + " drive F", kSwerve.DRIVE_F, Constants.TUNING_MODE);

        mAngleP = new TunableNumber("Mod " + moduleNumber + " angle P", kSwerve.ANGLE_P, Constants.TUNING_MODE);
        mAngleD = new TunableNumber("Mod " + moduleNumber + " angle D", kSwerve.ANGLE_D, Constants.TUNING_MODE);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (mDriveP.hasChanged()) {
            driveMotor.config_kP(0, mDriveP.get());
        }
        if (mDriveD.hasChanged()) {
            driveMotor.config_kD(0, mDriveD.get());
        }
        if (mDriveF.hasChanged()) {
            driveMotor.config_kF(0, mDriveF.get());
        }

        if (mAngleP.hasChanged()) {
            angleMotor.config_kP(0, mAngleP.get());
        }

        if (mAngleD.hasChanged()) {
            angleMotor.config_kD(0, mAngleD.get());
        }

        desiredState = CTREModuleState.optimize(desiredState, getState().angle); // Custom optimize command, since
                                                                                 // default WPILib optimize assumes
                                                                                 // continuous controller which CTRE is
                                                                                 // not

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kSwerve.MAX_SPEED;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, kSwerve.WHEEL_CIRCUMFRANCE,
                    kSwerve.DRIVE_GEAR_RATIO);
            driveMotor.set(ControlMode.Velocity, velocity);

            SmartDashboard.putNumber("Mod " + moduleNumber + " wanted velocity ", velocity);
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kSwerve.MAX_SPEED * 0.01)) ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, kSwerve.ANGLE_GEAR_RATIO));

        SmartDashboard.putNumber("Mod " + moduleNumber + " wanted angle ", angle);

        lastAngle = angle;
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getAngle(), kSwerve.ANGLE_GEAR_RATIO);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        angleEncoder.configSensorDirection(kSwerve.CANCODER_INVERSION);
        angleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public double getAngle() {
        return -getCanCoder().getDegrees() + angleOffset;
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), kSwerve.WHEEL_CIRCUMFRANCE,
                kSwerve.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(
                Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), kSwerve.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double distance = Conversions.falconToM(driveMotor.getSelectedSensorPosition(), kSwerve.WHEEL_CIRCUMFRANCE,
        kSwerve.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(
            Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), kSwerve.ANGLE_GEAR_RATIO));
        return new SwerveModulePosition(distance, angle);
    }

    public double getDriveSpeed() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getAngleCurrentDraw() {
        return angleMotor.getSupplyCurrent();
    }

    public double getDriveCurrentDraw() {
        return driveMotor.getSupplyCurrent();
    }

    public boolean exceedsCurrentLimit() {
        return getAngleCurrentDraw() > kSwerve.ANGLE_CURRENT_LIMIT
                && getDriveCurrentDraw() > kSwerve.DRIVE_CURRENT_LIMIT;
    }
}