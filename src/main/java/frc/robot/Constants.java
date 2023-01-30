// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean TUNING_MODE = false;
    public static final double STICK_DEADBAND = 0.1;

    /**
     * Defines port values.
     */
    public static class kPorts {
        public static final String CANIVORE_NAME = "Sussy Squad";
        public static final int PIGEON_ID = 13;
        public static final int INDEXER_MOTOR = 21;
        public static final int INTAKE_MOTOR_ID = 20;
        public static final int PNEUMATIC_FORWARD_CHANNEL_LEFT = 3;
        public static final int PNEUMATIC_REVERSE_CHANNEL_LEFT = 0;

        public static final int PNEUMATIC_FORWARD_CHANNEL_RIGHT = 2;
        public static final int PNEUMATIC_REVERSE_CHANNEL_RIGHT = 1;  

        public static final int MANIPULATOR_MOTOR_ID = 24;

        public static final int LEFT_MOTOR_ID = 23;
        public static final int RIGHT_MOTOR_ID = 22;
        public static final int ENCODER_CHANNEL = 0;
    }

    /**
     * Constants for indexer.
     */
    public static class kIndexer {
        public static final double SPEED = 0.7;
    }

    /**
     * Constants for intake.
     */
    public static class kIntake {
        public static final double MOTOR_SPEED = 0.5;
    }
    
    /**
     * Constants for swerve drive.
     */
    public static final class kSwerve {
        public static final boolean OPEN_LOOP = false;
        public static final boolean FEILD_RELATIVE = false;

        public static final boolean GYRO_INVERSION = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.73);
        public static final double WHEEL_BASE = Units.inchesToMeters(21.73);
        public static final double WHEEL_DIAMATER = Units.inchesToMeters(4);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMATER * Math.PI;

        public static final double OPEN_LOOP_RAMP = 0.25;

        public static final double DRIVE_GEAR_RATIO = 6.75; // 6.86:1
        public static final double ANGLE_GEAR_RATIO = (150.0 / 7.0); // 12.8:1

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 20;
        public static final int DRIVE_CURRENT_LIMIT = 25;

        /* Angle Motor PID Values */
        public static final double ANGLE_P = 0.3;
        public static final double ANGLE_I = 0.0;
        public static final double ANGLE_D = 12.0;
        public static final double ANGLE_F = 0.0;

        /* Drive Motor PID Values */
        public static final double DRIVE_P = 0.009000;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_F = 0.046;

        /* Swerve Profiling Values */
        public static final double MAX_ACCELERATION = 3; // 2
        public static final double MAX_SPEED = 5; // 4.5 meters per second
        public static final double MAX_ANGULAR_VELOCITY = 20; // 11.5

        public static final PIDController X_CONTROLLER = new PIDController(2, 0, 0);
        public static final PIDController Y_CONTROLLER = new PIDController(2, 0, 0);
        public static final PIDController ANGLE_CONTROLLER = new PIDController(4, 0, 0);

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean DRIVE_INVERSION = false;
        public static final boolean ANGLE_INVERSION = true; // make false if we have a stroke

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERSION = true;

        /* Module Specific Constants */

        /** Front Left Module - Module 0. */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 3;
            public static final int CAN_CODER_ID = 2;
            public static final double ANGLE_OFFSET = 89.648438;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET
            );
        }

        /** Front Right Module - Module 1. */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 10;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CAN_CODER_ID = 11;
            public static final double ANGLE_OFFSET = 195.380859;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET
            );
        }

        /** Back Left Module - Module 2. */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 5;
            public static final double ANGLE_OFFSET = 67.675781;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET
            );
        }

        /** Back Right Module - Module 3. */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CAN_CODER_ID = 8;
            public static final double ANGLE_OFFSET = 69.785156;
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET
            );
        }

    }

    /**
     * Controller values.
     */
    public static final class kOI {
        public static final int DRIVE_TRANSLATION_Y = XboxController.Axis.kLeftY.value;
        public static final int DRIVE_TRANSLATION_X = XboxController.Axis.kLeftX.value;
        public static final int DRIVE_ROTATE = XboxController.Axis.kRightX.value;

        public static final int UPDATE_ENCODER = XboxController.Button.kY.value;

        public static final int DRIVE_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    /**
     * Arm values.
     */
    public static final class kArm {
        public static final double GEAR_RATIO = 72.00;

        public static final boolean LEFT_INVERSION = true;
        public static final boolean RIGHT_INVERSION = true;

        public static final int LEFT_CURRENT_LIMIT = 40;
        public static final int RIGHT_CURRENT_LIMIT = 40;

        public static final IdleMode LEFT_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode RIGHT_IDLE_MODE = IdleMode.kBrake;

        public static final double kP = 0.0150000; // 0.015
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
        public static final double kS = 0.32245;
        public static final double kG = 0.42; //0.42
        public static final double kV = 0.018286;
        public static final double kA = 0.0019367;

        public static final double ERROR = 5.0; // degrees
        public static final double MAX_POSITION = 110.00; // in degrees

        public static final double ENCODER_ANGLE_OFFSET = 233.6;
        public static final double FEEDFORWARD_ANGLE_OFFSET = 313 - 233.6;

        public enum ArmPos {
            LOWERED(0),
            CONE_PICKUP_ALLIGMENT(100),
            CONE_PICKUP_LOWERED(75),
            L2_SCORING(75),
            L3_SCORING(0);

            private double angle;

            private ArmPos(double angle) {
                this.angle = angle;
            }

            public double getAngle() {
                return angle;
            }
        }
    }

    /**
     * Class that defines constants for wait command timmings in between commands. 
     * Times are in seconds.
     */
    public static final class kCommandTimmings {
        public static final double PNEUMATIC_WAIT_TIME = 0.7;
    }

    /**
     * Constants for manipulator.
     */
    public static final class kManipulator {
        public static final double SPEED = 1.0;
    }
}
