package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.OptionalDouble;

public class MachineLearningSubsystem extends NetworkTablesSubsystem {
    public static double STOPPING_Y = 178;//209;
    public static double STOPPING_TOLERANCE = 0.2;

    // Measured at 1 meter
    private final double HORIZONTAL_FIELD_OF_VIEW = 1.137;
    private final double VERTICAL_FIELD_OF_VIEW = 0.846;
    private final double SQRT_BALL_SIZE = 76;

    private final boolean FILTER_BALLS = false;

    private NetworkTableEntry targetAcquiredEntry;
    private NetworkTableEntry filterColorEntry;
    private NetworkTableEntry targetColorEntry;
    private NetworkTableEntry targetXEntry;
    private NetworkTableEntry targetYEntry;
    private NetworkTableEntry targetAreaEntry;
    private NetworkTableEntry resolutionXEntry;
    private NetworkTableEntry resolutionYEntry;

    public MachineLearningSubsystem() {
        super("ML");

        targetAcquiredEntry = getEntry("targetAcquired");
        filterColorEntry = getEntry("filterColor");
        targetColorEntry = getEntry("targetColor");
        targetXEntry = getEntry("targetX");
        targetYEntry = getEntry("targetY");
        targetAreaEntry = getEntry("targetArea");
        resolutionXEntry = getEntry("resolutionX");
        resolutionYEntry = getEntry("resolutionY");

        enableFiltering();
    }

    @Override
    public void periodic() {
        getEntry("Ball distance").setDouble(getBallDistance().orElse(0));
    }

    public boolean hasTarget() {
        return targetAcquiredEntry.getBoolean(false);
    }

    public void enableFiltering() {
        if (FILTER_BALLS) filterColorEntry.setString(DriverStation.getAlliance() == Alliance.Blue ? "blue" : "red");
        else filterColorEntry.setString("init_ML");
    }

    public Alliance getTargetColor() {
        Alliance color = Alliance.Invalid;

        switch (targetColorEntry.getString("")) {
            case "red":
                color = Alliance.Red;
                break;
            case "blue":
                color = Alliance.Blue;
                break;
        }

        return color;
    }

    public double getTargetX() {
        return targetXEntry.getDouble(0);
    }

    public double getTargetY() {
        return targetYEntry.getDouble(0);
    }

    public double getTargetArea() {
        return targetAreaEntry.getDouble(0);
    }

    public double getResolutionX() {
        return resolutionXEntry.getDouble(0);
    }

    public double getResolutionY() {
        return resolutionYEntry.getDouble(0);
    }

    public double getXNormalized() {
        return getTargetX() / getResolutionX() * 2 - 1;
    }

    public double getYNormalized() {
        return getTargetY() / getResolutionY() * 2 - 1;
    }

    public double getHorizontalAngle() {
        return -getXNormalized() / HORIZONTAL_FIELD_OF_VIEW;
    }

    public double getVerticalAngle() {
        return -getYNormalized() / VERTICAL_FIELD_OF_VIEW;
    }

    public OptionalDouble getBallDistance() {
        double targetArea = getTargetArea();

        if (targetArea == 0) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(SQRT_BALL_SIZE / Math.sqrt(getTargetArea()));
    }
}
