package frc.robot.strategies;

import com.team2539.cougarlib.MathUtils;
import com.team2539.cougarlib.control.InterpolatingMap;
import com.team2539.cougarlib.util.Updatable;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Regressions;
import frc.robot.ShootingSuperstructure;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.InterpolatableDouble;

public class MovingAimStrategy implements Updatable {
    // for using, call update before using other functions

    private static double LIMELIGHT_HORIZONTAL_ERROR = 0.1;

    private ShootingSuperstructure shootingSuperstructure;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private PIDController pidController;
    private Translation2d relativeVirtualGoalPosition;
    private Rotation2d currentTargetRotation = new Rotation2d();

    private InterpolatingMap<InterpolatableDouble> timingMap = Regressions.getShootingTimeMap();

    public MovingAimStrategy(ShootingSuperstructure shootingSuperstructure, PIDController pidController) {
        this.shootingSuperstructure = shootingSuperstructure;
        // PID controller should wrap, if it doesn't things will break
        this.pidController = pidController;
        swerveDriveSubsystem = shootingSuperstructure.getSwerveDriveSubsystem();
    }

    private Translation2d estimateRelativeFutureGoalTranslation(double timeToFuture, Pose2d currentRobotPose) {
        Pose2d futureRobotPosition = estimateFutureRobotPosition(timeToFuture, currentRobotPose);
        return GlobalConstants.goalLocation.minus(futureRobotPosition.getTranslation());
    }

    private Pose2d estimateFutureRobotPosition(double timeToFuture, Pose2d robotFieldPosition) {
        ChassisSpeeds velocityChassisSpeeds = swerveDriveSubsystem.getSmoothedVelocity();
        Transform2d velocityTransform2d = new Transform2d(
                new Translation2d(velocityChassisSpeeds.vxMetersPerSecond, velocityChassisSpeeds.vyMetersPerSecond),
                new Rotation2d(velocityChassisSpeeds.omegaRadiansPerSecond));
        return robotFieldPosition.plus(velocityTransform2d.times(timeToFuture));
    }

    public double calculateRotationalVelocity() {
        double robotRotation = swerveDriveSubsystem.getGyroRotation2d().getRadians();
        double rotationOutput = pidController.calculate(robotRotation, currentTargetRotation.getRadians());
        return -rotationOutput * SwerveConstants.maxAngularVelocity * 0.5;
    }

    public boolean isAimed() {
        double robotRotation = swerveDriveSubsystem.getGyroRotation2d().getRadians();
        return MathUtils.equalsWithinError(
                0,
                MathUtil.angleModulus(currentTargetRotation.getRadians() - robotRotation),
                LIMELIGHT_HORIZONTAL_ERROR);
    }

    public double calculateShotDistance() {
        return relativeVirtualGoalPosition.getNorm();
    }

    public void update() {
        // updates the field position for 0.02 seconds in the future
        Pose2d robotFieldPosition = swerveDriveSubsystem.getPose();

        // claculates the relative virtual goal position
        Translation2d relativeGoalPosition = GlobalConstants.goalLocation.minus(robotFieldPosition.getTranslation());
        double distance = relativeGoalPosition.getNorm();
        double predictedShotTime = timingMap.getInterpolated(distance).orElse(new InterpolatableDouble(0.84)).value;
        relativeVirtualGoalPosition = estimateRelativeFutureGoalTranslation(predictedShotTime, robotFieldPosition);

        // this has - to accound for the fact that 180 is shooter
        double robotGoalAngle = Math.atan2(-relativeVirtualGoalPosition.getY(), -relativeVirtualGoalPosition.getX());
        currentTargetRotation = new Rotation2d(robotGoalAngle);

        swerveDriveSubsystem.setGhostPosition(new Pose2d(robotFieldPosition.getTranslation(), currentTargetRotation));
    }

    public void resetPIDController() {
        pidController.reset();
    }
}
