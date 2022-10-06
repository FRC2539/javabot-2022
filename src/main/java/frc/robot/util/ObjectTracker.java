package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ObjectTracker {

    private double period;

    private ChassisSpeeds lastKnownVelocity;

    private Pose2d objectPosition;

    private Rotation2d robotRotation;

    public ObjectTracker() {
        this(0.02);
    }

    public ObjectTracker(double period) {
        this.period = period;
    }

    public Pose2d calculate(Pose2d objectPositionRelative, Rotation2d robotAbsoluteRotation) {
        objectPosition = objectPositionRelative.transformBy(
                new Transform2d(new Translation2d(), robotAbsoluteRotation).times(-1));
        robotRotation = robotAbsoluteRotation;
        return objectPosition;
    }

    public Pose2d calculate(ChassisSpeeds robotVelocityRelative) {
        lastKnownVelocity = robotVelocityRelative;
        return calculate();
    }

    public Pose2d calculate() {
        objectPosition = getFutureObjectTranslation(lastKnownVelocity, period);
        robotRotation = getFutureObjectRotation(lastKnownVelocity, period);
        return rotatePoseByRotation(objectPosition, robotRotation);
    }

    public Pose2d predict(double seconds) {
        Pose2d tempObjectPosition = getFutureObjectTranslation(lastKnownVelocity, seconds);
        Rotation2d tempRobotRotation = getFutureObjectRotation(lastKnownVelocity, seconds);
        return rotatePoseByRotation(tempObjectPosition, tempRobotRotation);
    }

    private Pose2d rotatePoseByRotation(Pose2d pose, Rotation2d rotation) {
        return new Pose2d(
                pose.getTranslation().rotateBy(rotation.times(-1)),
                pose.getRotation().minus(rotation));
    }

    private Pose2d getFutureObjectTranslation(ChassisSpeeds velocity, double seconds) {
        Translation2d changeInObjectPosition =
                new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond).times(seconds * -1);
        Pose2d tempObjectPosition =
                objectPosition.transformBy(new Transform2d(changeInObjectPosition, new Rotation2d()));
        return tempObjectPosition;
    }

    private Rotation2d getFutureObjectRotation(ChassisSpeeds velocity, double seconds) {
        Rotation2d tempRobotRotation =
                robotRotation.rotateBy(new Rotation2d(lastKnownVelocity.omegaRadiansPerSecond * seconds));
        return tempRobotRotation;
    }
}
