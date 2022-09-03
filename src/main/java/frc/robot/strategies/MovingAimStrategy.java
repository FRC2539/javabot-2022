package frc.robot.strategies;

import com.team2539.cougarlib.MathUtils;
import com.team2539.cougarlib.control.InterpolatingMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Regressions;
import frc.robot.ShootingSuperstructure;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.InterpolatableDouble;
import frc.robot.util.ShooterState;
import java.util.Optional;

public class MovingAimStrategy implements LimelightAimStrategy {

    private static double LIMELIGHT_HORIZONTAL_ERROR = 0.1;

    private ShootingSuperstructure shootingSuperstructure;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private PIDController pidController;
    private Pose2d robotFieldPosition;
    private Rotation2d currentTargetRotation = new Rotation2d();

    private InterpolatingMap<InterpolatableDouble> timingMap = Regressions.getShootingTimeMap();

    public MovingAimStrategy(ShootingSuperstructure shootingSuperstructure, PIDController pidController) {
        this.shootingSuperstructure = shootingSuperstructure;
        //PID controller should wrap, if it doesn't things will break
        this.pidController = pidController;
        swerveDriveSubsystem = shootingSuperstructure.getSwerveDriveSubsystem();
    }

    private void updatePositionLimelight() {
        Optional<Pose2d> robotLimelightRelativePose = shootingSuperstructure.getLimelightPoseEstimate();

        if (robotLimelightRelativePose.isPresent()) {
            robotFieldPosition = robotLimelightRelativePose.orElseThrow();
        } else {
            robotFieldPosition = estimateFutureRobotPosition(0.05);
        }
    }

    private double predictShotTime() {
        Translation2d relativeGoalPosition = GlobalConstants.goalLocation.minus(robotFieldPosition.getTranslation());
        double distance = relativeGoalPosition.getNorm();
        double timeForShot = timingMap.get(distance).value;
        // want to add ability to account for robot rotation distance maybe? probably not super neccesary.
        return timeForShot;
    }

    private Pose2d estimateRelativeFutureGoalPosition(double timeToFuture) {
        Pose2d futureRobotPosition = estimateFutureRobotPosition(timeToFuture);
        return new Pose2d(
                GlobalConstants.goalLocation.minus(futureRobotPosition.getTranslation()),
                futureRobotPosition.getRotation().times(-1));
    }

    private Pose2d estimateFutureRobotPosition(double timeToFuture) {
        ChassisSpeeds velocityChassisSpeeds = swerveDriveSubsystem.getVelocity();
        Transform2d velocityTransform2d = new Transform2d(
                new Translation2d(velocityChassisSpeeds.vxMetersPerSecond, velocityChassisSpeeds.vyMetersPerSecond),
                new Rotation2d(velocityChassisSpeeds.omegaRadiansPerSecond));
        return robotFieldPosition.plus(velocityTransform2d.times(timeToFuture));
    }

    private Rotation2d getRobotGoalRotation2d() {
        double predictedShotTime = predictShotTime();
        Pose2d predictedFuturePose2d = estimateRelativeFutureGoalPosition(predictedShotTime);
        double angle = Math.atan2(predictedFuturePose2d.getX(), predictedFuturePose2d.getY());
        return new Rotation2d(angle);
    }

    public double calculateRotationalVelocity() {
        double robotRotation = swerveDriveSubsystem.getGyroRotation2d().getRadians();
        currentTargetRotation = getRobotGoalRotation2d();
        double rotationOutput = pidController.calculate(robotRotation, currentTargetRotation.getRadians());
        return -1 * rotationOutput * SwerveDriveSubsystem.MAX_ANGULAR_VELOCITY;
    }

    public boolean isAimed() {
        double robotRotation = swerveDriveSubsystem.getGyroRotation2d().getRadians();
        return MathUtils.equalsWithinError(
                0,
                MathUtil.angleModulus(currentTargetRotation.getRadians() - robotRotation),
                LIMELIGHT_HORIZONTAL_ERROR);
    }

    public double calculateShotDistance() {
        double predictedShotTime = predictShotTime();
        Pose2d predictedFutureGoalPose2d = estimateRelativeFutureGoalPosition(predictedShotTime);
        return predictedFutureGoalPose2d.getTranslation().getNorm();
    }

    public void update() {
        updatePositionLimelight();
    }
}
