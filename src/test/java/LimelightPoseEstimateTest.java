import static org.junit.Assert.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.ShootingSuperstructure;
import frc.robot.subsystems.LimelightSubsystem;
import org.junit.*;

public class LimelightPoseEstimateTest {
    Translation2d limelightOffset = new Translation2d(
            LimelightSubsystem.LIMELIGHT_FORWARD_OFFSET, LimelightSubsystem.LIMELIGHT_SIDEWAYS_OFFSET);

    @Test
    public void limelightEstimatesCorrectCardinalPose() {
        Translation2d zeroPose = ShootingSuperstructure.getPoseEstimateFromLimelightValues(new Rotation2d(), 0, 3)
                .getTranslation();
        Translation2d leftPose = ShootingSuperstructure.getPoseEstimateFromLimelightValues(
                        new Rotation2d(Math.PI / 2), 0, 3)
                .getTranslation();
        Translation2d rightPose = ShootingSuperstructure.getPoseEstimateFromLimelightValues(
                        new Rotation2d(-Math.PI / 2), 0, 3)
                .getTranslation();
        Translation2d bottomPose = ShootingSuperstructure.getPoseEstimateFromLimelightValues(
                        new Rotation2d(Math.PI), 0, 3)
                .getTranslation();

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
        Translation2d testPose = ShootingSuperstructure.getPoseEstimateFromLimelightValues(
                        Rotation2d.fromDegrees(10), 10, 3)
                .getTranslation();

        assertEquals(
                testPose, new Translation2d(11.23, 4.115).minus(limelightOffset.rotateBy(Rotation2d.fromDegrees(10))));

        Translation2d beyondRangePose = ShootingSuperstructure.getPoseEstimateFromLimelightValues(
                        Rotation2d.fromDegrees(720), 0, 3)
                .getTranslation();

        assertEquals(
                beyondRangePose,
                new Translation2d(11.23, 4.115).minus(limelightOffset.rotateBy(Rotation2d.fromDegrees(720))));

        Translation2d unrealisticPose = ShootingSuperstructure.getPoseEstimateFromLimelightValues(
                        Rotation2d.fromDegrees(90), 90, 5)
                .getTranslation();

        assertEquals(
                unrealisticPose,
                new Translation2d(13.23, 4.115).minus(limelightOffset.rotateBy(Rotation2d.fromDegrees(90))));
    }
}
