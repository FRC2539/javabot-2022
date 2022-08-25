import static org.junit.Assert.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.GlobalConstants;
import org.junit.*;

public class LimelightPoseEstimateTest {
    public Translation2d getLimelightPositionEstimate(
            Rotation2d gyroAngle, double limelightAngle, double limelightDistance) {
        Rotation2d robotAngle = gyroAngle.plus(Rotation2d.fromDegrees(-limelightAngle));
        Translation2d originBasedRobotTranslation = new Translation2d(limelightDistance, robotAngle);

        return GlobalConstants.goalLocation.plus(originBasedRobotTranslation);
    }

    @Test
    public void limelightEstimatesCorrectCardinalPose() {
        Translation2d zeroPose = getLimelightPositionEstimate(new Rotation2d(), 0, 3);
        Translation2d leftPose = getLimelightPositionEstimate(new Rotation2d(Math.PI / 2), 0, 3);
        Translation2d rightPose = getLimelightPositionEstimate(new Rotation2d(-Math.PI / 2), 0, 3);
        Translation2d bottomPose = getLimelightPositionEstimate(new Rotation2d(Math.PI), 0, 3);

        assertEquals(zeroPose, new Translation2d(11.23, 4.115));
        assertEquals(leftPose, new Translation2d(8.23, 7.115));
        assertEquals(rightPose, new Translation2d(8.23, 1.115));
        assertEquals(bottomPose, new Translation2d(5.23, 4.115));
    }

    @Test
    public void limelightEstimatesCorrectOffsetPose() {
        Translation2d testPose = getLimelightPositionEstimate(Rotation2d.fromDegrees(10), 10, 3);

        assertEquals(testPose, new Translation2d(11.23, 4.115));

        Translation2d beyondRangePose = getLimelightPositionEstimate(Rotation2d.fromDegrees(720), 0, 3);

        assertEquals(beyondRangePose, new Translation2d(11.23, 4.115));
    }
}
