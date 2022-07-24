package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.BalltrackSubsystem.BalltrackMode;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LimelightPipeline;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.ShootingComponent;
import java.util.Optional;

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

    public void spinupShooterWithLimelightPrediction() {
        activateShootingPipeline();

        shooterSubsystem.setFarShot(limelightSubsystem.getPredictedDistanceSupplier());
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

    public ChassisSpeeds getSmoothedRobotVelocity() {
        return swerveDriveSubsystem.getSmoothedVelocity();
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
