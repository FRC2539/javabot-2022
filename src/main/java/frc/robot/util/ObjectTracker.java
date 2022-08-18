package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**THIS CLASS REQUIRES TESTING!!!!*/
public class ObjectTracker {

    private double period;

    private ChassisSpeeds lastKnownVelocity;

    private Pose2d objectPosition;

    private Rotation2d robotRotation;

    public ObjectTracker() {
        this(0.005);
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
        Translation2d changeInPosition = new Translation2d(
                        lastKnownVelocity.vxMetersPerSecond, lastKnownVelocity.vyMetersPerSecond)
                .times(period);
        objectPosition = objectPosition.transformBy(new Transform2d(changeInPosition, new Rotation2d()));
        robotRotation = robotRotation.rotateBy(new Rotation2d(lastKnownVelocity.omegaRadiansPerSecond * period));
        return objectPosition.transformBy(new Transform2d(new Translation2d(), robotRotation));
    }

    public Pose2d calculate() {
        Translation2d changeInPosition = new Translation2d(
                        lastKnownVelocity.vxMetersPerSecond, lastKnownVelocity.vyMetersPerSecond)
                .times(period);
        objectPosition = objectPosition.transformBy(new Transform2d(changeInPosition, new Rotation2d()));
        robotRotation = robotRotation.rotateBy(new Rotation2d(lastKnownVelocity.omegaRadiansPerSecond * period));
        return objectPosition.transformBy(new Transform2d(new Translation2d(), robotRotation));
    }

    public Pose2d predict(double seconds) {
        Translation2d changeInPosition = new Translation2d(
                        lastKnownVelocity.vxMetersPerSecond, lastKnownVelocity.vyMetersPerSecond)
                .times(seconds);
        objectPosition = objectPosition.transformBy(new Transform2d(changeInPosition, new Rotation2d()));
        robotRotation = robotRotation.rotateBy(new Rotation2d(lastKnownVelocity.omegaRadiansPerSecond * seconds));
        return objectPosition.transformBy(new Transform2d(new Translation2d(), robotRotation));
    }
}
