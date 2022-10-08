package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.control.MovingAverageVelocity;
import frc.lib.control.SwerveDriveSignal;
import frc.lib.estimator.SwerveDrivePoseEstimator;
import frc.lib.loops.Updatable;
import frc.robot.Constants;
import frc.robot.Constants.GlobalConstants;
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

    private final SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
            new Rotation2d(),
            new Pose2d(),
            Constants.SwerveConstants.swerveKinematics,
            VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.01)),
            VecBuilder.fill(Units.degreesToRadians(0.01)),
            VecBuilder.fill(0.025, 0.025, Units.degreesToRadians(0.025)),
            TimesliceConstants.CONTROLLER_PERIOD);

    private final MovingAverageVelocity velocityEstimator = new MovingAverageVelocity(50);

    private Pose2d pose = new Pose2d();
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private SwerveDriveSignal driveSignal = null;

    private final boolean LOG_TRAJECTORY_INFO = false;

    private NetworkTableEntry stationaryEntry;

    private NetworkTableEntry robotPoseEntry;

    private NetworkTableEntry enableGhostPose;
    private NetworkTableEntry ghostPoseEntry;

    private NetworkTableEntry calculatedDistanceEntry;

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
    private boolean TEMPERATURE_LOGGING_ENABLED = true;

    public SwerveDriveSubsystem() {
        super("Swerve");

        modules = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        // Reset each module using its absolute encoder to avoid having modules fail to align
        for (SwerveModule module : modules) {
            module.resetToAbsolute();
        }

        stationaryEntry = getEntry("stationary");

        robotPoseEntry = getEntry("robotPose");

        enableGhostPose = getEntry("enableGhostPose");
        enableGhostPose.setBoolean(false);
        ghostPoseEntry = getEntry("ghostPose");

        trajectoryXEntry = getEntry("Trajectory X");
        trajectoryYEntry = getEntry("Trajectory Y");
        trajectoryAngleEntry = getEntry("Trajectory Angle");

        calculatedDistanceEntry = getEntry("Swerve Based Target Distance");

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

    public boolean isStationary() {
        return driveSignal != null
                && Math.abs(driveSignal.vxMetersPerSecond) < 0.15
                && Math.abs(driveSignal.vyMetersPerSecond) < 0.15
                && Math.abs(driveSignal.omegaRadiansPerSecond) < 0.15;
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return swervePoseEstimator;
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
        swervePoseEstimator.resetPosition(pose, getGyroRotation2d());
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

        pose = swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroRotation2d(), moduleStates);
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

    public void setGhostPosition(Pose2d ghostPosition) {
        setGhostPositionState(true);
        ghostPoseEntry.setDoubleArray(new double[] {
            ghostPosition.getX(),
            ghostPosition.getY(),
            ghostPosition.getRotation().getDegrees()
        });
    }

    public void setGhostPositionState(boolean ghostPositionState) {
        enableGhostPose.setBoolean(ghostPositionState);
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

        stationaryEntry.setBoolean(isStationary());

        calculatedDistanceEntry.setDouble(getPose().getTranslation().getDistance(GlobalConstants.goalLocation));

        robotPoseEntry.setDoubleArray(
                new double[] {pose.getX(), pose.getY(), getGyroRotation2d().getDegrees()});

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
