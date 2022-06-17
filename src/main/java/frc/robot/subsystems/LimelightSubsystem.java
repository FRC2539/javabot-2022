package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.util.Updatable;
import java.util.OptionalDouble;

public class LimelightSubsystem extends NetworkTablesSubsystem implements Updatable {
    private static double TARGET_HEIGHT = 2.62;
    private static double LIMELIGHT_HEIGHT = 0.75819;
    private static double DELTA_HEIGHT = TARGET_HEIGHT - LIMELIGHT_HEIGHT;
    public static double TARGET_RADIUS = 0.678;

    private static double LIMELIGHT_MOUNTING_ANGLE = 60;

    private static double LIMELIGHT_ANGLE = Math.toRadians(90 - LIMELIGHT_MOUNTING_ANGLE);

    private static double LIMELIGHT_HORIZONTAL_ERROR = 2;

    private static double X_OFFSET_STEP = 0.2;
    private static double Y_OFFSET_STEP = 0.2;

    private double xOffset = 0.2;
    private double yOffset = 0;

    private static double lookaheadTime = 0.1; // tune this number to improve shooting with moving

    private OptionalDouble predictedDistanceToTarget = OptionalDouble.empty();
    private OptionalDouble predictedHorizontalAngle = OptionalDouble.empty();

    private OptionalDouble distanceToTarget = OptionalDouble.empty();

    private boolean isAimed = false;

    private double horizontalAngle = Double.NaN;
    private double verticalAngle = Double.NaN;

    private NetworkTableEntry pipelineEntry;
    private NetworkTableEntry hasTargetEntry;
    private NetworkTableEntry snapshotEntry;
    private NetworkTableEntry xEntry;
    private NetworkTableEntry yEntry;
    private NetworkTableEntry xOffsetEntry;
    private NetworkTableEntry yOffsetEntry;
    private NetworkTableEntry distanceEntry;
    private NetworkTableEntry predictedDistanceEntry;
    private NetworkTableEntry predictedAngleEntry;

    private final SwerveDriveSubsystem swerveDriveSubsystem;

    public LimelightSubsystem(SwerveDriveSubsystem swerveDriveSubsystem) {
        super("limelight");

        this.swerveDriveSubsystem = swerveDriveSubsystem;

        pipelineEntry = getEntry("pipeline");
        hasTargetEntry = getEntry("tv");
        snapshotEntry = getEntry("snapshot");
        xEntry = getEntry("tx");
        yEntry = getEntry("ty");
        xOffsetEntry = getEntry("xOffset");
        yOffsetEntry = getEntry("yOffset");
        distanceEntry = getEntry("distance");
        predictedDistanceEntry = getEntry("predictedDistance");
        predictedAngleEntry = getEntry("predictedAngle");

        xOffsetEntry.setDouble(xOffset);
        yOffsetEntry.setDouble(yOffset);

        setPipeline(LimelightPipeline.SHOOT);
    }

    public double getHorizontalAngle() {
        return horizontalAngle;
    }

    public double getVerticalAngle() {
        return verticalAngle;
    }

    public boolean isAimed() {
        return this.isAimed;
    }

    public OptionalDouble calculateDistanceToTarget() {
        if (hasTarget())
            return OptionalDouble.of(
                    DELTA_HEIGHT / Math.tan(LIMELIGHT_ANGLE + Math.toRadians(getVerticalAngle())) + TARGET_RADIUS);
        else return OptionalDouble.empty();
    }

    public OptionalDouble getDistanceToTarget() {
        return distanceToTarget;
    }

    public OptionalDouble getPredictedDistanceToTarget() {
        return predictedDistanceToTarget;
    }

    public OptionalDouble getPredictedHorizontalAngle() {
        return predictedHorizontalAngle;
    }

    public void updatePredictedLimelightMeasurements() {
        if (hasTarget() && getDistanceToTarget().isPresent()) {
            Pose2d robotRelativePoseEstimate = new Pose2d(
                    new Translation2d(getDistanceToTarget().getAsDouble(), new Rotation2d(getHorizontalAngle())),
                    new Rotation2d());

            ChassisSpeeds robotRelativeVelocity = swerveDriveSubsystem.getSmoothedVelocity();

            Transform2d estimatedTransform = new Transform2d(
                            new Translation2d(
                                    robotRelativeVelocity.vxMetersPerSecond, robotRelativeVelocity.vyMetersPerSecond),
                            new Rotation2d(robotRelativeVelocity.omegaRadiansPerSecond))
                    .times(lookaheadTime);

            Pose2d predictedPoseEstimate = robotRelativePoseEstimate.transformBy(estimatedTransform);

            predictedDistanceToTarget =
                    OptionalDouble.of(predictedPoseEstimate.getTranslation().getNorm());

            double predictedChangeInRobotAngle =
                    predictedPoseEstimate.getRotation().getRadians();
            double predictedChangeInAngleToTarget =
                    new Rotation2d(predictedPoseEstimate.getX(), predictedPoseEstimate.getY()).getRadians();

            predictedHorizontalAngle = OptionalDouble.of(predictedChangeInAngleToTarget - predictedChangeInRobotAngle);
        }
    }

    public void setPipeline(LimelightPipeline pipeline) {
        switch (pipeline) {
            case DRIVE:
                pipelineEntry.setNumber(0);
                break;
            case SHOOT:
                pipelineEntry.setNumber(1);
                break;
        }
    }

    public boolean hasTarget() {
        return hasTargetEntry.getNumber(0).doubleValue() == 1;
    }

    public void takeSnapshots() {
        snapshotEntry.setNumber(1);
    }

    public void stopTakingSnapshots() {
        snapshotEntry.setNumber(0);
    }

    @Override
    public void update() {
        horizontalAngle = xEntry.getDouble(0) + xOffset;
        verticalAngle = yEntry.getDouble(0) + yOffset;

        distanceToTarget = calculateDistanceToTarget();
        isAimed = hasTarget() && Math.abs(getHorizontalAngle()) < LIMELIGHT_HORIZONTAL_ERROR;

        updatePredictedLimelightMeasurements();
    }

    @Override
    public void periodic() {
        distanceEntry.setDouble(distanceToTarget.orElse(0));

        predictedDistanceEntry.setDouble(predictedDistanceToTarget.orElse(0));
        predictedAngleEntry.setDouble(predictedHorizontalAngle.orElse(0));
    }

    public void incrementXOffset() {
        xOffset += X_OFFSET_STEP;
        xOffsetEntry.setDouble(xOffset);
    }

    public void decrementXOffset() {
        xOffset -= X_OFFSET_STEP;
        xOffsetEntry.setDouble(xOffset);
    }

    public void incrementYOffset() {
        yOffset += Y_OFFSET_STEP;
        yOffsetEntry.setDouble(yOffset);
    }

    public void decrementYOffset() {
        yOffset -= Y_OFFSET_STEP;
        yOffsetEntry.setDouble(yOffset);
    }

    public enum LimelightPipeline {
        DRIVE,
        SHOOT
    }
}
