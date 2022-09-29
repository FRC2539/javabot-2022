package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.lib.MathUtils;
import frc.lib.loops.Updatable;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class LimelightSubsystem extends ShootingComponentSubsystem implements Updatable {
    private static double TARGET_HEIGHT = 2.62;
    public static double LIMELIGHT_HEIGHT = 0.75819;
    public static double LIMELIGHT_FORWARD_OFFSET = -0.10426;
    public static double LIMELIGHT_SIDEWAYS_OFFSET = 0.18015;
    private static double DELTA_HEIGHT = TARGET_HEIGHT - LIMELIGHT_HEIGHT;
    public static double TARGET_RADIUS = 0.678;

    private static double LIMELIGHT_MOUNTING_ANGLE = 60;

    private static double LIMELIGHT_ANGLE = Math.toRadians(90 - LIMELIGHT_MOUNTING_ANGLE);

    private static double LIMELIGHT_HORIZONTAL_ERROR = 2;
    private static double LIMELIGHT_HORIZONTAL_ERROR_TIGHTER = 0.5;

    private static double LIMELIGHT_FRAME_LATENCY = 0.011;

    private static double X_OFFSET_STEP = 0.2;
    private static double Y_OFFSET_STEP = 0.2;

    private OptionalDouble storedLimelightDistance = OptionalDouble.empty();

    private double xOffset = 0.2;
    private double yOffset = 0;

    private OptionalDouble distanceToTarget = OptionalDouble.empty();
    private Pose2d robotRelativePoseEstimate = null;

    private boolean isAimed = false;
    private boolean isAimedTigher = false;

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
    private NetworkTableEntry latencyEntry;

    private Optional<Updatable> updatableAimStrategy = Optional.empty();
    private int updatableAimStrategySemaphore = 0;

    private int ticksAimed;
    private int ticksAimedTighter;

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
        latencyEntry = getEntry("tl");

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

    public boolean isAimedTighter() {
        return this.isAimedTigher;
    }

    public int getTicksAimedTighter() {
        return ticksAimedTighter;
    }

    public int getTicksAimed() {
        return ticksAimed;
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
        return pipelineEntry.getNumber(0).doubleValue() == 1
                && hasTargetEntry.getNumber(0).doubleValue() == 1;
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

    public void storeCurrentShotDistance(double distanceToTarget) {
        storedLimelightDistance = OptionalDouble.of(distanceToTarget);
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
        isAimedTigher =
                hasTarget() && MathUtils.equalsWithinError(0, getHorizontalAngle(), LIMELIGHT_HORIZONTAL_ERROR_TIGHTER);

        if (updatableAimStrategy.isPresent()) {
            updatableAimStrategy.orElseThrow().update();
        }

        if (isAimed()) {
            ticksAimed++;
        } else {
            ticksAimed = 0;
        }

        if (isAimedTighter()) {
            ticksAimedTighter++;
        } else {
            ticksAimedTighter = 0;
        }

        shootingSuperstructure.getLimelightPoseEstimate().ifPresent((Pose2d e) -> {
            if (isAimedTighter()) {
                shootingSuperstructure.updatePoseEstimateWithVision(e);
            }
            shootingSuperstructure.setGhostPosition(e);
            shootingSuperstructure.setGhostPositionState(true);
        });
    }

    // do not try to bind multiple things or you will get errors
    public void bindUpdatable(Updatable updatable) {
        if (updatableAimStrategySemaphore <= 0) {
            updatableAimStrategy = Optional.of(updatable);
            updatableAimStrategySemaphore = 1;
        } else {
            updatableAimStrategySemaphore++;
        }
    }

    public void freeUpdatable() {
        updatableAimStrategySemaphore--;
        if (updatableAimStrategySemaphore <= 0) {
            updatableAimStrategySemaphore = 0;
            updatableAimStrategy = Optional.empty();
        }
    }

    public boolean isUpdatableFree() {
        return updatableAimStrategySemaphore <= 0;
    }

    public double getLatency() {
        return LIMELIGHT_FRAME_LATENCY + latencyEntry.getDouble(0);
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
