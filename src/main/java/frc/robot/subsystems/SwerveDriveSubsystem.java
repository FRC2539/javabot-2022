package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;

public class SwerveDriveSubsystem extends NetworkTablesSubsystem {
    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.427799754, 0.427799754), // Front left
            new Translation2d(0.427799754, -0.427799754), // Front right
            new Translation2d(-0.427799754, 0.427799754), // Back left
            new Translation2d(-0.427799754, -0.427799754) // Back right
    );


    private SwerveModule[] modules;

    private final AHRS gyroscope = new AHRS();

    private final SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(swerveKinematics, new Rotation2d(), new Pose2d());

    private Pose2d pose = new Pose2d();
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private ChassisSpeeds driveSignal = null;


    public SwerveDriveSubsystem() {
        super("Swerve Drive");

        SwerveModule frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_TURN_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET
        );
        SwerveModule frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_TURN_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET
        );
        SwerveModule backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_TURN_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT,
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET
        );
        SwerveModule backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_TURN_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT,
                Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET
        );

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backLeftModule, backRightModule};
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

    public void drive(ChassisSpeeds velocity) {
            driveSignal = velocity;
    }

    public void resetPose(Pose2d pose) {
            this.pose = pose;
            swerveOdometry.resetPosition(pose, getGyroRotation2d());
    }

    public void setGyroAngle(Rotation2d angle) {
        gyroscope.reset();
        gyroscope.setAngleAdjustment(angle.getDegrees());
    }

    public void resetGyroAngle() {
        setGyroAngle(new Rotation2d());
    }

    private void updateOdometry() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
    
            moduleStates[i] = new SwerveModuleState(module.getDriveVelocity() * 39.37008, new Rotation2d(module.getSteerAngle()));
        }

        this.velocity = swerveKinematics.toChassisSpeeds(moduleStates);
        this.pose = swerveOdometry.update(getGyroRotation2d(), moduleStates);
    }

    private void updateModules(ChassisSpeeds driveSignal) {
        ChassisSpeeds chassisVelocity;
        if (driveSignal == null) {
            chassisVelocity = new ChassisSpeeds();
        } else {
            chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(driveSignal.vxMetersPerSecond, driveSignal.vyMetersPerSecond, driveSignal.omegaRadiansPerSecond, getGyroRotation2d());
        }

        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(chassisVelocity);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);
        for (int i = 0; i < moduleStates.length; i++) {
            var module = modules[i];
            module.set(moduleStates[i].speedMetersPerSecond * 12.0, moduleStates[i].angle.getRadians());
        }
    }

    public void update() {
        updateOdometry();

        ChassisSpeeds driveSignal = this.driveSignal;

        updateModules(driveSignal);
    }

    @Override
    public void periodic() {
    }
}