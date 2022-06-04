package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.common.control.SwerveDriveSignal;
import frc.robot.util.TrajectoryFollower;
import frc.robot.util.Updatable;
import java.util.Optional;

public class SwerveDriveSubsystem extends NetworkTablesSubsystem implements Updatable {
    // Measured in meters (ask CAD dept. for this information in new robots)
    public static final double TRACKWIDTH = 0.5969;
    public static final double WHEELBASE = 0.5969;

    public static final double MAX_VOLTAGE = 12.0;

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Front left
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // Front right
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Back left
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // Back right
            );

    public static final double MAX_VELOCITY = 6380.0
            / 60
            * SdsModuleConfigurations.MK4_L2.getDriveReduction()
            * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
            * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(TRACKWIDTH / 2.0, WHEELBASE / 2.0);

    private final TrajectoryFollower follower = new TrajectoryFollower(
            new PIDController(1, 0, 0, Constants.CONTROLLER_PERIOD),
            new PIDController(1, 0, 0, Constants.CONTROLLER_PERIOD),
            new ProfiledPIDController(
                    1,
                    0,
                    0,
                    new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY / 2),
                    Constants.CONTROLLER_PERIOD));

    private SwerveModule[] modules;

    private final AHRS gyroscope = new AHRS();

    private final SwerveDriveOdometry swerveOdometry =
            new SwerveDriveOdometry(swerveKinematics, new Rotation2d(), new Pose2d());

    private Pose2d pose = new Pose2d();
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private SwerveDriveSignal driveSignal = null;

    private NetworkTableEntry odometryXEntry;
    private NetworkTableEntry odometryYEntry;
    private NetworkTableEntry odometryAngleEntry;

    private NetworkTableEntry trajectoryXEntry;
    private NetworkTableEntry trajectoryYEntry;
    private NetworkTableEntry trajectoryAngleEntry;

    private NetworkTableEntry driveTemperaturesEntry;
    private NetworkTableEntry steerTemperaturesEntry;

    private NetworkTableEntry navXAngleEntry;

    public SwerveDriveSubsystem() {
        super("Swerve Drive");

        Mk4ModuleConfiguration invertedConfiguration = new Mk4ModuleConfiguration();

        invertedConfiguration.setDriveInverted(true);

        SwerveModule frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_TURN_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET);
        SwerveModule frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                invertedConfiguration,
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_TURN_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET);
        SwerveModule backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_TURN_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT,
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET);
        SwerveModule backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                invertedConfiguration,
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_TURN_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT,
                Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET);

        modules = new SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        odometryXEntry = getEntry("X");
        odometryYEntry = getEntry("Y");
        odometryAngleEntry = getEntry("Angle");

        trajectoryXEntry = getEntry("Trajectory X");
        trajectoryYEntry = getEntry("Trajectory Y");
        trajectoryAngleEntry = getEntry("Trajectory Angle");

        navXAngleEntry = getEntry("NavX Angle");

        driveTemperaturesEntry = getEntry("Drive Temperatures");
        steerTemperaturesEntry = getEntry("Steer Temperatures");
    }

    public Pose2d getPose() {
        return pose;
    }

    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    public Rotation2d getGyroRotation2d() {
        return gyroscope.getRotation2d();
    }

    public double getGyroAngle() {
        return gyroscope.getAngle() % 360;
    }

    public double getRawGyroAngle() {
        return gyroscope.getAngle();
    }

    public void drive(ChassisSpeeds velocity, boolean isFieldOriented) {
        driveSignal = new SwerveDriveSignal(velocity, isFieldOriented);
    }

    public void stop() {
        driveSignal = new SwerveDriveSignal();
    }

    public void resetPose(Pose2d pose) {
        this.pose = pose;
        swerveOdometry.resetPosition(pose, getGyroRotation2d());
    }

    public void resetGyroAngle(Rotation2d angle) {
        gyroscope.reset();
        gyroscope.setAngleAdjustment(angle.getDegrees());
    }

    public void resetGyroAngle() {
        resetGyroAngle(new Rotation2d());
    }

    private void updateOdometry() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];

            moduleStates[i] = new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
        }

        this.velocity = swerveKinematics.toChassisSpeeds(moduleStates);
        this.pose = swerveOdometry.updateWithTime(Timer.getFPGATimestamp(), getGyroRotation2d(), moduleStates);
    }

    private void updateModules(SwerveDriveSignal driveSignal) {
        ChassisSpeeds chassisVelocity;
        if (driveSignal == null) {
            chassisVelocity = new ChassisSpeeds();
        } else if (driveSignal.isFieldOriented()) {
            chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveSignal.vxMetersPerSecond,
                    driveSignal.vyMetersPerSecond,
                    driveSignal.omegaRadiansPerSecond,
                    getGyroRotation2d());
        } else {
            chassisVelocity = new ChassisSpeeds(
                    driveSignal.vxMetersPerSecond, driveSignal.vyMetersPerSecond, driveSignal.omegaRadiansPerSecond);
        }

        if (chassisVelocity.vxMetersPerSecond == 0
                && chassisVelocity.vyMetersPerSecond == 0
                && chassisVelocity.omegaRadiansPerSecond == 0) {
            stopModules();
            return;
        }

        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(chassisVelocity);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY);
        for (int i = 0; i < moduleStates.length; i++) {
            var module = modules[i];
            module.set(
                    moduleStates[i].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE,
                    moduleStates[i].angle.getRadians());
        }
    }

    public void stopModules() {
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.set(0, module.getSteerAngle());
        }
    }

    @Override
    public void update() {
        updateOdometry();

        SwerveDriveSignal driveSignal;

        Optional<SwerveDriveSignal> trajectorySignal = follower.update(getPose());

        if (trajectorySignal.isPresent()) {
            driveSignal = trajectorySignal.get();
            driveSignal = new SwerveDriveSignal(
                    driveSignal.vxMetersPerSecond,
                    driveSignal.vyMetersPerSecond,
                    driveSignal.omegaRadiansPerSecond,
                    false);
        } else {
            driveSignal = this.driveSignal;
        }

        updateModules(driveSignal);
    }

    @Override
    public void periodic() {
        // Pose2d pose = getPose();

        // odometryXEntry.setDouble(pose.getX());
        // odometryYEntry.setDouble(pose.getY());
        // odometryAngleEntry.setDouble(pose.getRotation().getDegrees());

        // if (follower.getLastState() == null) {
        //     trajectoryXEntry.setDouble(0);
        //     trajectoryYEntry.setDouble(0);
        //     trajectoryAngleEntry.setDouble(0);
        // } else {
        //     Pose2d trajectoryPose = getFollower().getLastState().poseMeters;

        //     trajectoryXEntry.setDouble(trajectoryPose.getX());
        //     trajectoryYEntry.setDouble(trajectoryPose.getY());
        //     trajectoryAngleEntry.setDouble(trajectoryPose.getRotation().getDegrees());
        // }

        // navXAngleEntry.setDouble(getGyroRotation2d().getDegrees());

        driveTemperaturesEntry.setDoubleArray(
            new double[]{
                modules[0].getDriveTemperature(),
                modules[1].getDriveTemperature(),
                modules[2].getDriveTemperature(),
                modules[3].getDriveTemperature()});

        steerTemperaturesEntry.setDoubleArray(
            new double[]{
                modules[0].getSteerTemperature(),
                modules[1].getSteerTemperature(),
                modules[2].getSteerTemperature(),
                modules[3].getSteerTemperature()});
    }

    public TrajectoryFollower getFollower() {
        return follower;
    }
}
