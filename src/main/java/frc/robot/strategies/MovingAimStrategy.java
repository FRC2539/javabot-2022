package frc.robot.strategies;

import com.team2539.cougarlib.MathUtils;
import com.team2539.cougarlib.control.InterpolatingMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Regressions;
import frc.robot.ShootingSuperstructure;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.InterpolatableDouble;
import frc.robot.util.ObjectTracker;
import frc.robot.util.ShooterState;

import java.util.Optional;

public class MovingAimStrategy implements VariableDistanceStrategy {

    private static double LIMELIGHT_HORIZONTAL_ERROR = 0.1;

    private ShootingSuperstructure shootingSuperstructure;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private PIDController pidController;
    private Pose2d currentPosition;
    private ObjectTracker goalTracker = new ObjectTracker();
    private Rotation2d currentTargetRotation = new Rotation2d();

    private InterpolatingMap<InterpolatableDouble> timingMap = Regressions.getShootingTimeMap();

    public MovingAimStrategy(ShootingSuperstructure shootingSuperstructure, PIDController pidController) {
        this.shootingSuperstructure = shootingSuperstructure;
        this.pidController = pidController;
        swerveDriveSubsystem = shootingSuperstructure.getSwerveDriveSubsystem();
    }

    public void updatePositionLimelight() {
        Optional<Pose2d> limelightPos = shootingSuperstructure.getLimelightPoseEstimate();
        if (limelightPos.isPresent()) {
            currentPosition =
                    goalTracker.calculate(limelightPos.orElseThrow(), swerveDriveSubsystem.getGyroRotation2d());
        } else {
            currentPosition = goalTracker.calculate(swerveDriveSubsystem.getVelocity());
        }
    }

    private double predictShotTime() {
        Translation2d relativeGoalPosition = GlobalConstants.goalLocation.minus(currentPosition.getTranslation());
        double distance = relativeGoalPosition.getNorm();
        double timeForShot = timingMap.get(distance).value;
        // want to add ability to account for robot rotation distance maybe? probably not super neccesary.
        return timeForShot;
    }

    private Pose2d getRelativeFutureGoalPosition(double timeToFuture) {
        return goalTracker.predict(timeToFuture);
    }

    private Rotation2d getRobotGoalRotation2d() {
        double predictedShotTime = predictShotTime();
        Pose2d predictedFuturePose2d = getRelativeFutureGoalPosition(predictedShotTime);
        double angle = Math.atan2(predictedFuturePose2d.getX(), predictedFuturePose2d.getY());
        return new Rotation2d(angle);
    }

    public double calculateRotationalVelocity() {
        updatePositionLimelight();
        double robotRotation = swerveDriveSubsystem.getGyroRotation2d().getRadians();
        currentTargetRotation = getRobotGoalRotation2d();
        return pidController.calculate(robotRotation, currentTargetRotation.getRadians());
    }

    public boolean isAimed() {
        double robotRotation = swerveDriveSubsystem.getGyroRotation2d().getRadians();
        return MathUtils.equalsWithinError(
                0,
                MathUtil.angleModulus(currentTargetRotation.getRadians() - robotRotation),
                LIMELIGHT_HORIZONTAL_ERROR);
    }

    public ShooterState getShooterState() {
        double predictedShotTime = predictShotTime();
        Pose2d predictedFuturePose2d = getRelativeFutureGoalPosition(predictedShotTime);
        return shooterSubsystem.calculateShooterStateForDistance(predictedFuturePose2d.getTranslation().getNorm());
    }
}
