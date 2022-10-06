package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.BalltrackSubsystem.BalltrackMode;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LimelightPipeline;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.ShootingComponent;
import java.util.Optional;
import java.util.OptionalDouble;

public class ShootingSuperstructure {
    private BalltrackSubsystem balltrackSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private SwerveDriveSubsystem swerveDriveSubsystem;

    public ShootingSuperstructure() {}

    public void registerComponent(ShootingComponent component) {
        component.registerSuperstructure(this);

        switch (component.getName()) {
            case "BallSystem":
                balltrackSubsystem = (BalltrackSubsystem) component;
                break;
            case "limelight":
                limelightSubsystem = (LimelightSubsystem) component;
                break;
            case "Shooter":
                shooterSubsystem = (ShooterSubsystem) component;
                break;
            case "Swerve":
                swerveDriveSubsystem = (SwerveDriveSubsystem) component;
                break;
            default:
                break;
        }
    }

    public BalltrackSubsystem getBalltrackSubsystem() {
        return balltrackSubsystem;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDriveSubsystem;
    }

    public LimelightSubsystem getLimelightSubsystem() {
        return limelightSubsystem;
    }

    public void activateShootingPipeline() {
        limelightSubsystem.setPipeline(LimelightPipeline.SHOOT);
    }

    public void spinupShooterWithLimelight() {
        activateShootingPipeline();

        shooterSubsystem.setFarShot(limelightSubsystem.getMeasuredDistanceSupplier());
    }

    public void prepareBallsToShoot() {
        balltrackSubsystem.setBalltrackMode(BalltrackMode.PREPARE);
    }

    public void spinupShooterWithCustomSpeeds() {
        shooterSubsystem.setCustomShot();
    }

    public void shootWithLimelight() {
        if (limelightSubsystem.isAimed() && shooterSubsystem.isShooterAtVelocity()) balltrackSubsystem.shootMode();
    }

    public void shootWithoutLimelight() {
        if (shooterSubsystem.isShooterAtVelocity()) balltrackSubsystem.shootMode();
    }

    public void stopShooting() {
        balltrackSubsystem.stopShootMode();
        shooterSubsystem.stopShooter();
    }

    public void stopShootingMaintainSpeed() {
        balltrackSubsystem.stopShootMode();
    }

    public void storeCurrentShotDistance() {
        limelightSubsystem.storeCurrentShotDistance();
    }

    public void storeCurrentShotDistance(double distanceToTarget) {
        limelightSubsystem.storeCurrentShotDistance(distanceToTarget);
    }

    public double getFieldBasedDistance() {
        return swerveDriveSubsystem.getPose().getTranslation().getDistance(GlobalConstants.goalLocation);
    }

    public OptionalDouble getStoredShotDistance() {
        return limelightSubsystem.getStoredShotDistance();
    }

    public ChassisSpeeds getSmoothedRobotVelocity() {
        return swerveDriveSubsystem.getSmoothedVelocity();
    }

    public static Pose2d getPoseEstimateFromLimelightValues(
            Rotation2d gyroAngle, double limelightAngle, double limelightDistance) {

        Rotation2d robotAngle = gyroAngle.plus(Rotation2d.fromDegrees(-limelightAngle));
        Translation2d originBasedLimelightTranslation = new Translation2d(limelightDistance, robotAngle);
        Translation2d originBasedRobotTranslation = originBasedLimelightTranslation.minus(new Translation2d(
                        LimelightSubsystem.LIMELIGHT_FORWARD_OFFSET, LimelightSubsystem.LIMELIGHT_SIDEWAYS_OFFSET)
                .rotateBy(gyroAngle));

        return new Pose2d(GlobalConstants.goalLocation.plus(originBasedRobotTranslation), gyroAngle);
    }

    public Optional<Pose2d> getLimelightPoseEstimate() {
        if (!limelightSubsystem.hasTarget()) return Optional.empty();
        else {
            return Optional.of(getPoseEstimateFromLimelightValues(
                    swerveDriveSubsystem.getGyroRotation2d(),
                    limelightSubsystem.getUncorrectedHorizontalAngle(),
                    limelightSubsystem.getDistanceToTarget().orElse(0)));
        }
    }

    public void updatePoseEstimateWithVision(Pose2d visionPose) {
        updatePoseEstimateWithVision(visionPose, Timer.getFPGATimestamp());
    }

    public void updatePoseEstimateWithVision(Pose2d visionPose, double timestamp) {
        swerveDriveSubsystem.getPoseEstimator().addVisionMeasurement(visionPose, timestamp);
    }

    public void setGhostPosition(Pose2d ghostPose) {
        swerveDriveSubsystem.setGhostPosition(ghostPose);
    }

    public void setGhostPositionState(boolean state) {
        swerveDriveSubsystem.setGhostPositionState(state);
    }

    public Translation2d getTranslationToTarget() {
        return limelightSubsystem
                .getRobotRelativePoseEstimate()
                .getTranslation()
                .unaryMinus();
    }

    public boolean hasTarget() {
        return limelightSubsystem.hasTarget();
    }
}
