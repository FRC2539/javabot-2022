import static org.junit.Assert.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.LimelightSubsystem;
import org.junit.*;

public class LimelightPoseEstimateTest {
    Translation2d limelightOffset = new Translation2d(
            LimelightSubsystem.LIMELIGHT_FORWARD_OFFSET, LimelightSubsystem.LIMELIGHT_SIDEWAYS_OFFSET);

    public Translation2d getLimelightPositionEstimate(
            Rotation2d gyroAngle, double limelightAngle, double limelightDistance) {
        Rotation2d robotAngle = gyroAngle.plus(Rotation2d.fromDegrees(-limelightAngle));
        Translation2d originBasedLimelightTranslation = new Translation2d(limelightDistance, robotAngle);
        Translation2d originBasedRobotTranslation =
                originBasedLimelightTranslation.minus(limelightOffset.rotateBy(gyroAngle));

        return GlobalConstants.goalLocation.plus(originBasedRobotTranslation);
    }

    @Test
    public void limelightEstimatesCorrectCardinalPose() {
        Translation2d zeroPose = getLimelightPositionEstimate(new Rotation2d(), 0, 3);
        Translation2d leftPose = getLimelightPositionEstimate(new Rotation2d(Math.PI / 2), 0, 3);
        Translation2d rightPose = getLimelightPositionEstimate(new Rotation2d(-Math.PI / 2), 0, 3);
        Translation2d bottomPose = getLimelightPositionEstimate(new Rotation2d(Math.PI), 0, 3);

        assertEquals(zeroPose, new Translation2d(11.23, 4.115).minus(limelightOffset.rotateBy(new Rotation2d())));
        assertEquals(
                leftPose, new Translation2d(8.23, 7.115).minus(limelightOffset.rotateBy(new Rotation2d(Math.PI / 2))));
        assertEquals(
                rightPose,
                new Translation2d(8.23, 1.115).minus(limelightOffset.rotateBy(new Rotation2d(-Math.PI / 2))));
        assertEquals(
                bottomPose, new Translation2d(5.23, 4.115).minus(limelightOffset.rotateBy(new Rotation2d(Math.PI))));
    }

    @Test
    public void limelightEstimatesCorrectOffsetPose() {
        Translation2d testPose = getLimelightPositionEstimate(Rotation2d.fromDegrees(10), 10, 3);

        assertEquals(
                testPose, new Translation2d(11.23, 4.115).minus(limelightOffset.rotateBy(Rotation2d.fromDegrees(10))));

        Translation2d beyondRangePose = getLimelightPositionEstimate(Rotation2d.fromDegrees(720), 0, 3);

        assertEquals(
                beyondRangePose,
                new Translation2d(11.23, 4.115).minus(limelightOffset.rotateBy(Rotation2d.fromDegrees(720))));
    }
}
