package frc.robot.subsystems;

import com.team2539.cougarlib.MathUtils;
import com.team2539.cougarlib.util.Updatable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class LimelightSubsystem extends ShootingComponentSubsystem implements Updatable {
    private static double TARGET_HEIGHT = 2.62;
    private static double LIMELIGHT_HEIGHT = 0.75819;
    private static double DELTA_HEIGHT = TARGET_HEIGHT - LIMELIGHT_HEIGHT;
    public static double TARGET_RADIUS = 0.678;

    private static double LIMELIGHT_MOUNTING_ANGLE = 60;

    private static double LIMELIGHT_ANGLE = Math.toRadians(90 - LIMELIGHT_MOUNTING_ANGLE);

    private static double LIMELIGHT_HORIZONTAL_ERROR = 2;

    private static double X_OFFSET_STEP = 0.2;
    private static double Y_OFFSET_STEP = 0.2;

    private OptionalDouble storedLimelightDistance = OptionalDouble.empty();

    private double xOffset = 0.2;
    private double yOffset = 0;

    private OptionalDouble distanceToTarget = OptionalDouble.empty();
    private Pose2d robotRelativePoseEstimate = null;

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

    public LimelightSubsystem() {
        super("limelight");

        pipelineEntry = getEntry("pipeline");
        hasTargetEntry = getEntry("tv");
        snapshotEntry = getEntry("snapshot");
        xEntry = getEntry("tx");
        yEntry = getEntry("ty");
        xOffsetEntry = getEntry("xOffset");
        yOffsetEntry = getEntry("yOffset");
        distanceEntry = getEntry("distance");

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

    public Pose2d getRobotRelativePoseEstimate() {
        return robotRelativePoseEstimate;
    }

    public DoubleSupplier getMeasuredDistanceSupplier() {
        return () -> getDistanceToTarget().orElse(0);
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

    public void storeCurrentShotDistance() {
        storedLimelightDistance = getDistanceToTarget();
    }

    public OptionalDouble getStoredShotDistance() {
        return storedLimelightDistance;
    }

    @Override
    public void update() {
        horizontalAngle = xEntry.getDouble(0) + xOffset;
        verticalAngle = yEntry.getDouble(0) + yOffset;

        distanceToTarget = calculateDistanceToTarget();
        isAimed = hasTarget() && MathUtils.equalsWithinError(0, getHorizontalAngle(), LIMELIGHT_HORIZONTAL_ERROR);
    }

    @Override
    public void periodic() {
        distanceEntry.setDouble(distanceToTarget.orElse(0));
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
