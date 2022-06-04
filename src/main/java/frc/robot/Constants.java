package frc.robot;

public class Constants {
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 0;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 2;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 3;

    public static final int DRIVETRAIN_FRONT_RIGHT_TURN_MOTOR = 6;
    public static final int DRIVETRAIN_FRONT_LEFT_TURN_MOTOR = 4;
    public static final int DRIVETRAIN_BACK_LEFT_TURN_MOTOR = 5;
    public static final int DRIVETRAIN_BACK_RIGHT_TURN_MOTOR = 7;

    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 24;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 26;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 25;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 27;

    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = Math.toRadians(299.434);
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = Math.toRadians(191.78);
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = Math.toRadians(20.033);
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = Math.toRadians(84.915);

    public static final int LEFT_DRIVE_CONTROLLER = 0;
    public static final int RIGHT_DRIVE_CONTROLLER = 1;
    public static final int OPERATOR_CONTROLLER = 2;

    public static final String CANIVORE_NAME = "CANivore";

    public static final double ROBOT_PERIODIC_ALLOCATION = 0.002;
    public static final double CONTROLLER_PERIOD = 0.005;

    public static final double DRIVETRAIN_PERIOD = 0.0015;
    public static final double SHOOTER_PERIOD = 0.0007;
    public static final double BALLTRACK_PERIOD = 0.0006;
    public static final double LIMELIGHT_PERIOD = 0.0002;

    public static final int PCM_ID = 17;

    public static final int SHOOTER_SOLENOID_FORWARD_CHANNEL = 4;
    public static final int SHOOTER_SOLENOID_REVERSE_CHANNEL = 5;
    public static final int SHOOTER_REAR_MOTOR_PORT = 8;
    public static final int SHOOTER_FRONT_MOTOR_PORT = 9;

    public static final int LED_CONTROLLER_PWM_ID = 9;

    public static final int BALLTRACK_INTAKE_MOTOR_PORT = 10;
    public static final int BALLTRACK_CONVEYOR_MOTOR_PORT = 11;
    public static final int BALLTRACK_CHAMBER_MOTOR_PORT = 12;

    public static final int BALLTRACK_CONVEYOR_SENSOR_PORT = 0;
    public static final int BALLTRACK_CHAMBER_SENSOR_PORT = 1;

    public static final int BALLTRACK_INTAKE_SOLENOID_FORWARD_CHANNEL = 0;
    public static final int BALLTRACK_INTAKE_SOLENOID_REVERSE_CHANNEL = 1;

    public static final int CLIMBER_MOTOR_PORT = 18;

    public static final int CLIMBER_SOLENOID_FORWARD_CHANNEL = 6;
    public static final int CLIMBER_SOLENOID_REVERSE_CHANNEL = 7;
}
