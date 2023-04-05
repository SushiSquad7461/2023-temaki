// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import SushiFrcLib.Swerve.SwerveModuleConstants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import java.io.IOException;

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
    public static final double MIN_PRESSURE = 115;
    public static final double MAX_PRESSURE = 120;

    enum RobotNames {
        ALPHA,
        BETA
    }

    public static final RobotNames ROBOT_NAME = RobotNames.BETA;

    /**
     * Defines port values.
     */
    public static class kPorts {
        public static final String CANIVORE_NAME = "Sussy Squad";
        public static final String PIGEON_CANIVORE_NAME;

        static {
            switch (ROBOT_NAME) {
              case ALPHA:
                  PIGEON_CANIVORE_NAME = "";
                  break;
              default:
                  PIGEON_CANIVORE_NAME = "Sussy Squad";
                  break;
            }
        }

        public static final int PIGEON_ID = 13;
        public static final int INDEXER_MOTOR = 21;
        public static final int CONE_RAMP_MOTOR = 25;
        public static final int INTAKE_BOTTOM_MOTOR_ID = 20;
        public static final int INTAKE_TOP_MOTOR_ID = 26;

        // intake pneumatic
        public static final int PNEUMATIC_FORWARD_CHANNEL;
        public static final int PNEUMATIC_REVERSE_CHANNEL;

        static {
            switch (ROBOT_NAME) {
              case ALPHA:
                  PNEUMATIC_FORWARD_CHANNEL = 5;
                  PNEUMATIC_REVERSE_CHANNEL = 6;
                  break;
              default:
                  PNEUMATIC_FORWARD_CHANNEL = 0; //correct
                  PNEUMATIC_REVERSE_CHANNEL = 2; //correct
                  break;
            }
        }

        // arm pneumatic
        public static final int PNEUMATIC_FORWARD_CHANNEL_ARM = 1;
        public static final int PNEUMATIC_REVERSE_CHANNEL_ARM = 5;  

        public static final int PNEUMATIC_FORWARD_CHANNEL_ARM2 = 4;
        public static final int PNEUMATIC_REVERSE_CHANNEL_ARM2 = 6;

        public static final int MANIPULATOR_MOTOR_ID = 24;

        public static final int LEFT_MOTOR_ID = 23;
        public static final int RIGHT_MOTOR_ID = 22;
        public static final int ENCODER_CHANNEL = 0;

        public static final int MANIPULATOR_BEAM_BREAk = 1;
        public static final int ARM_LIMIT_SWITCH = 3;
    }

    /**
     * Constants for indexer.
     */
    public static class kIndexer {
        public static final double INDEXER_SPEED;
        public static final double CONE_RAMP_SPEED;

        static {
            switch (ROBOT_NAME) {
              case ALPHA:
                  INDEXER_SPEED = 0.7;
                  CONE_RAMP_SPEED = 0.5;
                  break;
              default:
                  INDEXER_SPEED = -0.7;
                  CONE_RAMP_SPEED = -1;
                  break;
            }
        }
    }

    /**
     * Constants for intake.
     */
    public static class kIntake {
        public static final double MOTOR_SPEED = 0.8;
        public static final double CUBE_SHOOT_TOP_SPEED = 0.8;
        public static final int CURRENT_LIMIT = 30;
    }

    public static class kAutoBalance {
        public static final double FLATNESS_THRESHOLD_DEGREES = 2;
        public static final double MAX_SPEED = Constants.kSwerve.MAX_SPEED * 0.0035;
    }
    
    /**
     * Constants for swerve drive.
     */
    
    public static final class kSwerve {
        public static final boolean OPEN_LOOP = false;
        public static final boolean FEILD_RELATIVE = false;

        public static final double SPEED_MULTIPLER = 1.0;

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
        public static final int DRIVE_CURRENT_LIMIT = 40;

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
        public static final double MAX_SPEED = 10; // 4.5 meters per second
        public static final double MAX_ACCELERATION = 4; // 2
        public static final double MAX_ANGULAR_VELOCITY = 10; // 11.5
        public static final double MAX_ANGULAR_ACCELERATION = 20; // 11.5

        public static final PIDConstants TRANSLATION_CONTROLLER = new PIDConstants(1, 0, 0);
        public static final PIDConstants ANGLE_CONTROLLER = new PIDConstants(2, 0, 0);

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean DRIVE_INVERSION = false;
        public static final boolean ANGLE_INVERSION = true; // make false if we have a stroke

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERSION = true;

        /** Pose estimation standard deviations. */
        public static final Matrix<N3, N1> STATE_STANDARD_DEVIATION = 
            VecBuilder.fill(4.2, 4.2, 0.1);
        public static final Matrix<N3, N1> VISION_STANDARD_DEVIATION = 
            VecBuilder.fill(30, 30, 70); // TODO : tune values properly

        /* Module Specific Constants */

        /** Front Left Module - Module 0. */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 3;
            public static final int CAN_CODER_ID = 2;
            public static final double ANGLE_OFFSET;

            static {
                switch (ROBOT_NAME) {
                  case ALPHA:
                      ANGLE_OFFSET = 89.648438;
                      break;
                  default:
                      ANGLE_OFFSET = 2.109375 + 180;
                      break;
                }
            }

            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET
            );
        }

        /** Front Right Module - Module 1. */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 10;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CAN_CODER_ID = 11;
            public static final double ANGLE_OFFSET;

            static {
                switch (ROBOT_NAME) {
                  case ALPHA:
                      ANGLE_OFFSET = 195.380859;
                      break;
                  default:
                      ANGLE_OFFSET = 229.130859 - 180;
                      break;
                }
            }

            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET
            );
        }

        /** Back Left Module - Module 2. */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 5;
            public static final double ANGLE_OFFSET;

            static {
                switch (ROBOT_NAME) {
                  case ALPHA:
                      ANGLE_OFFSET = 67.675781;
                      break;
                  default:
                      ANGLE_OFFSET = 16.083984 + 180;
                      break;
                }
            }
            
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET
            );
        }

        /** Back Right Module - Module 3. */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CAN_CODER_ID = 8;
            public static final double ANGLE_OFFSET;

            static {
                switch (ROBOT_NAME) {
                  case ALPHA:
                      ANGLE_OFFSET = 69.785156;
                      break;
                  default:
                      ANGLE_OFFSET = 303.574219 - 180;
                      break;
                }
            }

            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET
            );
        }
    }

    /** 
     * Auto align values.
     */
    public static final class kAutoAlign {
        public static final Rotation2d DEFAULT_ROTATION;
        
        static {
            switch (ROBOT_NAME) {
            case ALPHA:
                DEFAULT_ROTATION = Rotation2d.fromDegrees(180);
                break;
            default:
                DEFAULT_ROTATION = Rotation2d.fromDegrees(0); 
                break;
            }
        }

        /** PID tolerance. */
        public static final double X_TOLLERENCE = 0.01;
        public static final double Y_TOLLERENCE = 0.02;
        public static final double THETA_TOLLERENCE = 0.02;

        /* Pid values */ // TODO : tune properly
        public static final double X_P = 9.0;
        public static final double X_I = 0.0;
        public static final double X_D = 0.0;
        public static final double Y_P = 9.0;
        public static final double Y_I = 0.0;
        public static final double Y_D = 0.0;
                
        public static final double THETA_P = 9.0;
        public static final double THETA_I = 0.0;
        public static final double THETA_D = 0.0;

        public static final double ERROR = 0.1;
        /** Default offset value. */
        public static final Translation2d DEFAULT_OFFSET = new Translation2d(1.0, 0);
        public static final Pose2d[] SCORING_POSES = {
            new Pose2d(15.3, 0.6, Rotation2d.fromDegrees(180)),
            new Pose2d(15.3, 1.071626, Rotation2d.fromDegrees(180)),
            new Pose2d(15.3, 1.64, Rotation2d.fromDegrees(180)),
            new Pose2d(15.3, 2.1, Rotation2d.fromDegrees(180)),
            new Pose2d(15.3, 2.748026, Rotation2d.fromDegrees(180)),
            new Pose2d(15.3, 3.3, Rotation2d.fromDegrees(180)),
            new Pose2d(15.3, 3.857, Rotation2d.fromDegrees(180)),
            new Pose2d(15.3, 4.424426, Rotation2d.fromDegrees(180)),
            new Pose2d(15.3, 5.0, Rotation2d.fromDegrees(180)),
            new Pose2d(1.2, 0.6, Rotation2d.fromDegrees(0)),
            new Pose2d(1.2, 1.071626, Rotation2d.fromDegrees(0)),
            new Pose2d(1.2, 1.64, Rotation2d.fromDegrees(0)),
            new Pose2d(1.2, 2.1, Rotation2d.fromDegrees(0)),
            new Pose2d(1.2, 2.748026, Rotation2d.fromDegrees(0)),
            new Pose2d(1.2, 3.3, Rotation2d.fromDegrees(0)),
            new Pose2d(1.2, 3.857, Rotation2d.fromDegrees(0)),
            new Pose2d(1.2, 4.424426, Rotation2d.fromDegrees(0)),
            new Pose2d(1.2, 5.0, Rotation2d.fromDegrees(0))
        };
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

        /** 
         * Slew rate limiters for anti tip.
         * Limits the acceleration essentially.
         */
        public static final SlewRateLimiter DRIVE_X_LIMITER;
        public static final SlewRateLimiter DRIVE_Y_LIMITER;
        public static final SlewRateLimiter DRIVE_THETA_LIMITER;

        static {
            switch (ROBOT_NAME) {
              case ALPHA:
                  DRIVE_X_LIMITER = new SlewRateLimiter(3);
                  DRIVE_Y_LIMITER = new SlewRateLimiter(3);
                  DRIVE_THETA_LIMITER = new SlewRateLimiter(3);
                  break;
              default:
                  DRIVE_X_LIMITER = new SlewRateLimiter(3000);
                  DRIVE_Y_LIMITER = new SlewRateLimiter(3000);
                  DRIVE_THETA_LIMITER = new SlewRateLimiter(3000);
                  break;
            }
        }

    }

    /**
     * Arm values.
     */
    public static final class kArm {
        public static final double GEAR_RATIO;

        public static final boolean LEFT_INVERSION;
        public static final boolean RIGHT_INVERSION;

        static {
            switch (ROBOT_NAME) {
              case ALPHA:
                  LEFT_INVERSION = true;
                  RIGHT_INVERSION = true;
                  break;
              default:
                  LEFT_INVERSION = false;
                  RIGHT_INVERSION = false;
                  break;
            }
        }

        public static final int LEFT_CURRENT_LIMIT = 25;
        public static final int RIGHT_CURRENT_LIMIT = 25;

        public static final IdleMode LEFT_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode RIGHT_IDLE_MODE = IdleMode.kBrake;

        public static final double ERROR = 5.0; // degrees
        public static final double MAX_POSITION = 110.00; // in degrees

        public static final double ENCODER_ANGLE_OFFSET;

        static {
            switch (ROBOT_NAME) {
              case ALPHA:
                  ENCODER_ANGLE_OFFSET = 233.6;
                  break;
              default:
                  ENCODER_ANGLE_OFFSET = 166.789129;
                  break;
            }
        }

        public static final double PARLLELE_ZERO_ANGLED = 84.602423; // Angle of arm parllele to ground when zeroed at the bottom
        public static final double FEEDFORWARD_ANGLE_OFFSET;

        static {
            switch (ROBOT_NAME) {
              case ALPHA:
                FEEDFORWARD_ANGLE_OFFSET = 313 - ENCODER_ANGLE_OFFSET;
                break;
              default:
                FEEDFORWARD_ANGLE_OFFSET = PARLLELE_ZERO_ANGLED;
                break;
            }
        }

        public static final double KP;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
        public static final double KF = 0.0;
        public static final double KS;
        public static final double KG;
        public static final double KGR; // kG for beta retracted arm
        public static final double KGE; // kG for beta extended arm
        public static final double KV;
        public static final double KA;

        static {
            switch (ROBOT_NAME) {
              case ALPHA:
                  GEAR_RATIO = 72.0;

                  KP = 0.0100000;
                  KS = 0.32245;
                  KG = 0.42;
                  KGE = 0;
                  KGR = 0;
                  KV = 0.018286;
                  KA = 0.0019367;
                  break;
              default:
                  GEAR_RATIO = 96.67;

                  KP = 0.020000; //0.01
                  KS = 0.0;
                  KG = 0.0;
                  KGR = 0.58; //0.6, 2.5
                  KGE = 1.5;
                  KV = 0.0;
                  KA = 0.0;
                  break;
            }
        }

        /**
         * Enum for arm angles.
         */
        public enum ArmPos {
            L1_SCORING(47),
            LOWERED(0),
            CONE_PICKUP_ALLIGMENT(ROBOT_NAME == RobotNames.ALPHA ? 91.5 : 92.5),
            CONE_PICKUP_LOWERED(75),
            L2_SCORING(ROBOT_NAME == RobotNames.ALPHA ? 81 : 79),
            L3_SCORING(ROBOT_NAME == RobotNames.ALPHA ? 81 : 95);

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
        public static final double PNEUMATIC_WAIT_TIME = 0.5;
        public static final double MANIPULATOR_WAIT_TIME = 0.8;
    }

    /**
     * Constants for manipulator.
     */
    public static final class kManipulator {
        public static final double SPEED = 1.0;
        public static final double CUBE_REVERSE_SPEED = -0.4;
        public static final double CONE_REVERSE_SPEED = 0.5;
        public static final int CURRENT_LIMITING = 30;
        public static final int MAX_CURRENT = 30;
    }

    /** Vision constants. */
    public static class kVision {
        public static final double AMBIGUITY_THRESHOLD = 0.15;
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createFieldLayout();

        /**
         * Since the april tag field layout constructor throws something, we need
         * create a method to handle it.
         */ 
        private static AprilTagFieldLayout createFieldLayout() {
            try {
                return new AprilTagFieldLayout(Filesystem
                    .getDeployDirectory()
                    .toPath()
                    .resolve("april-tag-layout.json")
                );
            } catch (IOException e) {
                throw new Error(e);
            }
        }

        public static final Translation3d CAMERA_POS_METERS;
        public static final Rotation3d CAMERA_ANGLE_DEGREES;

        static {
            switch (ROBOT_NAME) {
              case ALPHA:
                  // X is forward, Y is left.
                  CAMERA_POS_METERS = new Translation3d(
                      Units.inchesToMeters(2.691), 
                      Units.inchesToMeters(6),
                      Units.inchesToMeters(27.75)
                  );
                  CAMERA_ANGLE_DEGREES = new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(12),
                      Units.degreesToRadians(0)
                  ).unaryMinus();
                  break;
              default:
                  // X is forward, Y is left.
                  CAMERA_POS_METERS = new Translation3d(
                      Units.inchesToMeters(2.092), 
                      Units.inchesToMeters(8.5),
                      Units.inchesToMeters(31.161)
                  );
                  CAMERA_ANGLE_DEGREES = new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(16.579),
                      Units.degreesToRadians(180)
                  ).unaryMinus();
            }
        }

        public static final Transform3d CAMERA_TO_ROBOT_METERS_DEGREES = new Transform3d(
            CAMERA_POS_METERS.unaryMinus(), 
            CAMERA_ANGLE_DEGREES
        ); 
    }
}
