package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.team2539.cougarlib.control.MovingAverageVelocity;
import com.team2539.cougarlib.control.SwerveDriveSignal;
import com.team2539.cougarlib.util.Updatable;
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
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.TimesliceConstants;
import frc.robot.SwerveModule;
import frc.robot.util.LoggingManager;
import frc.robot.util.TrajectoryFollower;
import java.util.Optional;

/**
 * SwerveDriveSubsystem
 */
public class SwerveDriveSubsystem extends ShootingComponentSubsystem implements Updatable {
    public final PIDController autoXController = new PIDController(1, 0, 0, TimesliceConstants.CONTROLLER_PERIOD);
    public final PIDController autoYController = new PIDController(1, 0, 0, TimesliceConstants.CONTROLLER_PERIOD);
    public final ProfiledPIDController autoThetaController = new ProfiledPIDController(
            0.17,
            0,
            0.07,
            new TrapezoidProfile.Constraints(
                    Constants.SwerveConstants.maxSpeed, Constants.SwerveConstants.maxAngularVelocity),
            TimesliceConstants.CONTROLLER_PERIOD);

    private final TrajectoryFollower follower =
            new TrajectoryFollower(autoXController, autoYController, autoThetaController);

    private frc.robot.SwerveModule[] modules;

    private final AHRS gyro = new AHRS();

    private final SwerveDriveOdometry swerveOdometry =
            new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, new Rotation2d(), new Pose2d());

    private final MovingAverageVelocity velocityEstimator = new MovingAverageVelocity(50);

    private Pose2d pose = new Pose2d();
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private SwerveDriveSignal driveSignal = null;

    private final boolean LOG_TRAJECTORY_INFO = false;

    private NetworkTableEntry odometryXEntry;
    private NetworkTableEntry odometryYEntry;
    private NetworkTableEntry odometryAngleEntry;

    private NetworkTableEntry trajectoryXEntry;
    private NetworkTableEntry trajectoryYEntry;
    private NetworkTableEntry trajectoryAngleEntry;

    private NetworkTableEntry driveTemperaturesEntry;
    private NetworkTableEntry steerTemperaturesEntry;

    private NetworkTableEntry vx;
    private NetworkTableEntry vy;

    private DoubleArrayLogEntry driveTemperaturesLogEntry;
    private DoubleArrayLogEntry steerTemperaturesLogEntry;

    private Timer loggingTimer = new Timer();

    private static double TEMPERATURE_LOGGING_PERIOD = 5; // seconds
    private boolean TEMPERATURE_LOGGING_ENABLED = false;

    public SwerveDriveSubsystem() {
        super("Swerve");

        modules = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        odometryXEntry = getEntry("X");
        odometryYEntry = getEntry("Y");
        odometryAngleEntry = getEntry("Angle");

        trajectoryXEntry = getEntry("Trajectory X");
        trajectoryYEntry = getEntry("Trajectory Y");
        trajectoryAngleEntry = getEntry("Trajectory Angle");

        driveTemperaturesEntry = getEntry("Drive Temperatures");
        steerTemperaturesEntry = getEntry("Steer Temperatures");

        vx = getEntry("vx");
        vy = getEntry("vy");

        vx.setDouble(0);
        vy.setDouble(0);

        startLoggingTemperatures();

        // Flip the initial pose estimate to match the practice pose estimate to the post-auto pose estimate
        resetGyroAngle(new Rotation2d());
        resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)));
    }

    public void startLoggingTemperatures() {
        // Log motor temperatures only when not simulated
        if (TEMPERATURE_LOGGING_ENABLED && LoggingManager.getLog().isPresent()) {
            driveTemperaturesLogEntry =
                    new DoubleArrayLogEntry(LoggingManager.getLog().get(), "/temps/drive");
            steerTemperaturesLogEntry =
                    new DoubleArrayLogEntry(LoggingManager.getLog().get(), "/temps/steer");

            loggingTimer.start();
        }
    }

    public void enableLoggingTemperatures() {
        TEMPERATURE_LOGGING_ENABLED = true;

        startLoggingTemperatures();
    }

    public Pose2d getPose() {
        return pose;
    }

    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    public ChassisSpeeds getSmoothedVelocity() {
        return velocityEstimator.getAverage();
    }

    public Rotation2d getGyroRotation2d() {
        return gyro.getRotation2d();
    }

    public double getGyroAngle() {
        return gyro.getAngle() % 360;
    }

    public double getRawGyroAngle() {
        return gyro.getAngle();
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
        gyro.reset();
        gyro.setAngleAdjustment(angle.getDegrees());
    }

    public void resetGyroAngle() {
        resetGyroAngle(new Rotation2d());
    }

    public double[] getDriveTemperatures() {
        return new double[] {
            modules[0].getDriveTemperature(),
            modules[1].getDriveTemperature(),
            modules[2].getDriveTemperature(),
            modules[3].getDriveTemperature()
        };
    }

    public double[] getSteerTemperatures() {
        return new double[] {
            modules[0].getSteerTemperature(),
            modules[1].getSteerTemperature(),
            modules[2].getSteerTemperature(),
            modules[3].getSteerTemperature()
        };
    }

    private void updateOdometry() {
        SwerveModuleState[] moduleStates = getModuleStates();

        velocity = Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(moduleStates);

        velocityEstimator.add(velocity);

        pose = swerveOdometry.updateWithTime(Timer.getFPGATimestamp(), getGyroRotation2d(), moduleStates);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
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

        vx.setDouble(chassisVelocity.vxMetersPerSecond);
        vy.setDouble(chassisVelocity.vyMetersPerSecond);

        SwerveModuleState[] moduleStates =
                Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisVelocity);

        setModuleStates(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule module : modules) {
            module.setDesiredState(desiredStates[module.moduleNumber], true);
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
        Pose2d pose = getPose();

        odometryXEntry.setDouble(pose.getX());
        odometryYEntry.setDouble(pose.getY());
        odometryAngleEntry.setDouble(getGyroRotation2d().getDegrees());

        if (LOG_TRAJECTORY_INFO) {
            if (follower.getLastState() == null) {
                trajectoryXEntry.setDouble(0);
                trajectoryYEntry.setDouble(0);
                trajectoryAngleEntry.setDouble(0);
            } else {
                Pose2d trajectoryPose = getFollower().getLastState().poseMeters;

                trajectoryXEntry.setDouble(trajectoryPose.getX());
                trajectoryYEntry.setDouble(trajectoryPose.getY());
                trajectoryAngleEntry.setDouble(trajectoryPose.getRotation().getDegrees());
            }
        }

        // Log the motor temperatures periodically
        if (loggingTimer.advanceIfElapsed(TEMPERATURE_LOGGING_PERIOD)) {
            double[] driveTemperatures = getDriveTemperatures();
            double[] steerTemperatures = getSteerTemperatures();

            driveTemperaturesEntry.setDoubleArray(driveTemperatures);
            steerTemperaturesEntry.setDoubleArray(steerTemperatures);

            driveTemperaturesLogEntry.append(driveTemperatures);
            steerTemperaturesLogEntry.append(steerTemperatures);
        }
    }

    public TrajectoryFollower getFollower() {
        return follower;
    }
}
