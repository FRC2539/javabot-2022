package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.util.Updatable;

public class LimelightSubsystem extends NetworkTablesSubsystem implements Updatable {
    private final double TARGET_HEIGHT = 2.62;
    private final double LIMELIGHT_HEIGHT = 0.737;
    private final double DELTA_HEIGHT = TARGET_HEIGHT - LIMELIGHT_HEIGHT;

    private final double LIMELIGHT_ANGLE = Math.toRadians(60);

    private final double LIMELIGHT_HORIZONTAL_ERROR = 0.5;

    private final double X_OFFSET_STEP = 0.2;
    private final double Y_OFFSET_STEP = 0.2;

    private double xOffset = 0.2;
    private double yOffset = 0;

    private OptionalDouble distanceToTarget = OptionalDouble.empty();

    private NetworkTableEntry pipelineEntry;
    private NetworkTableEntry hasTargetEntry;
    private NetworkTableEntry snapshotEntry;
    private NetworkTableEntry xEntry;
    private NetworkTableEntry yEntry;
    private NetworkTableEntry xOffsetEntry;
    private NetworkTableEntry yOffsetEntry;

    public LimelightSubsystem() {
        super("limelight");

        pipelineEntry = getEntry("pipeline");
        hasTargetEntry = getEntry("tv");
        snapshotEntry = getEntry("snapshot");
        xEntry = getEntry("tx");
        yEntry = getEntry("ty");
        xOffsetEntry = getEntry("xOffset");
        yOffsetEntry = getEntry("xOffset");

        setPipeline(LimelightPipeline.DRIVE);
    }

    public double getHorizontalAngle() {
        return xEntry.getDouble(0) + xOffset;
    }

    public double getVerticalAngle() {
        return yEntry.getDouble(0) + yOffset;
    }

    public boolean isAimed() {
        return Math.abs(getHorizontalAngle()) < LIMELIGHT_HORIZONTAL_ERROR;
    }

    public OptionalDouble calculateDistanceToTarget() {
        if (hasTarget())
            return OptionalDouble.of(DELTA_HEIGHT / Math.tan(LIMELIGHT_ANGLE + Math.toRadians(getVerticalAngle())));
        else
            return OptionalDouble.empty();
    }

    public OptionalDouble getDistanceToTarget() {
        return distanceToTarget;
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
        distanceToTarget = calculateDistanceToTarget();
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
