package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.swerve.SwerveModuleConstants;

public final class Constants {
    public static final class GlobalConstants {
        public static final String CANIVORE_NAME = "CANivore";
        public static final int PCM_ID = 17;
        public static final Translation2d goalLocation = new Translation2d(8.23, 4.115);
    }

    public static final class SwerveConstants {
        // See https://github.com/Team364/BaseFalconSwerve for getting these values.

        public static final double trackWidth = 0.5969;
        public static final double wheelBase = 0.5969;
        public static final double wheelDiameter = 0.10033;
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.0; // 0.25
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS =
                (0.667 / 12); // divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.968230454756032; // meters per second
        public static final double maxAngularVelocity = 11.771048567785275;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 24;
            public static final double angleOffset = 60.996;
            public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canivoreName);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 26;
            public static final double angleOffset = 348.135;
            public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canivoreName);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 25;
            public static final double angleOffset = 339.434;
            public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canivoreName);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 27;
            public static final double angleOffset = 94.043;
            public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canivoreName);
        }
    }

    public static final class ControllerConstants {
        public static final int LEFT_DRIVE_CONTROLLER = 0;
        public static final int RIGHT_DRIVE_CONTROLLER = 1;
        public static final int OPERATOR_CONTROLLER = 2;
    }

    public static final class TimesliceConstants {
        public static final double ROBOT_PERIODIC_ALLOCATION = 0.002;
        public static final double CONTROLLER_PERIOD = 0.005;

        public static final double DRIVETRAIN_PERIOD = 0.0017;
        public static final double SHOOTER_PERIOD = 0.0003;
        public static final double BALLTRACK_PERIOD = 0.0005;
        public static final double LIMELIGHT_PERIOD = 0.0004;
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_SOLENOID_FORWARD_CHANNEL = 4;
        public static final int SHOOTER_SOLENOID_REVERSE_CHANNEL = 5;
        public static final int SHOOTER_REAR_MOTOR_PORT = 8;
        public static final int SHOOTER_FRONT_MOTOR_PORT = 9;
    }

    public static final class LightsConstants {
        public static final int LED_CONTROLLER_PWM_ID = 9;
    }

    public static final class BalltrackConstants {
        public static final int BALLTRACK_INTAKE_MOTOR_PORT = 10;
        public static final int BALLTRACK_CONVEYOR_MOTOR_PORT = 11;
        public static final int BALLTRACK_CHAMBER_MOTOR_PORT = 12;

        public static final int BALLTRACK_CONVEYOR_SENSOR_PORT = 0;
        public static final int BALLTRACK_CHAMBER_SENSOR_PORT = 1;

        public static final int BALLTRACK_INTAKE_SOLENOID_FORWARD_CHANNEL = 0;
        public static final int BALLTRACK_INTAKE_SOLENOID_REVERSE_CHANNEL = 1;
    }

    public static final class ClimberConstants {
        public static final int CLIMBER_MOTOR_PORT = 18;

        public static final int CLIMBER_SOLENOID_FORWARD_CHANNEL = 6;
        public static final int CLIMBER_SOLENOID_REVERSE_CHANNEL = 7;

        public static final int CLIMBER_SENSOR_RIGHT_PORT = 2;
        public static final int CLIMBER_SENSOR_LEFT_PORT = 3;
    }
}
