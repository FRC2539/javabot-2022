package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
            case "Swerve Drive":
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

    public OptionalDouble getStoredShotDistance() {
        return limelightSubsystem.getStoredShotDistance();
    }

    public ChassisSpeeds getSmoothedRobotVelocity() {
        return swerveDriveSubsystem.getSmoothedVelocity();
    }

    public Optional<Translation2d> getLimelightPositionEstimate() {
        if(!limelightSubsystem.hasTarget()) return Optional.empty();
        else {
            Rotation2d robotAngle = swerveDriveSubsystem.getGyroRotation2d().plus(Rotation2d.fromDegrees(-limelightSubsystem.getHorizontalAngle()));
            Translation2d originBasedRobotTranslation = new Translation2d(limelightSubsystem.getDistanceToTarget().orElse(0), robotAngle);
            
            return Optional.of(GlobalConstants.goalLocation.plus(originBasedRobotTranslation));
        }
    }

    public void rotateAroundTarget() {
        swerveDriveSubsystem.setAxisOfRotation(Optional.of(() -> getTranslationToTarget()));
    }

    public Translation2d getTranslationToTarget() {
        return limelightSubsystem
                .getRobotRelativePoseEstimate()
                .getTranslation()
                .unaryMinus();
    }

    public void stopRotatingAroundTarget() {
        swerveDriveSubsystem.setAxisOfRotation(Optional.empty());
    }
}
