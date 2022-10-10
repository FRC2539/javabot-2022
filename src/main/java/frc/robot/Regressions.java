package frc.robot;

import frc.lib.control.InterpolatingMap;
import frc.lib.interpolation.InterpolatableDouble;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.util.ShooterState;
import java.util.stream.Collectors;

public class Regressions {
    // {Distance, Rear RPM, Front RPM, (0 = close angle, 1 = far angle)}
    public static double[][] practiceRegression = {
        {1.2, 1900, 1550, 0},
        {2, 2300, 1550, 0},
        {2.5, 2400, 1550, 0},
        {2.99, 2700, 1550, 0},
        {3, 2300, 1550, 1},
        {3.6, 2600, 1550, 1},
        {4.3, 3150, 1550, 1},
        {5.57, 3700, 1800, 1},
        {6.03, 4200, 2000, 1},
    };

    public static double[][] competitionRegression = {
        {3, 2300, 1550, 1},
        {3.6, 2600, 1550, 1},
        {4.3, 3150, 1550, 1},
        {5.57, 3700, 1800, 1},
        {6.03, 4200, 2000, 1},
    };

    // this is test data, we have not gotten actual data yet
    public static double[][] shootingTimeRegression = {
        {3.00, 0.70},
        {3.50, 1.02},
        {4.00, 1.17},
        {5.00, 1.37},
        {5.60, 1.4},
        {6.03, 1.45}
    };

    public static InterpolatingMap<ShooterState> loadShootingMap(double[][] regression) {
        InterpolatingMap<ShooterState> shootingMap = new InterpolatingMap<ShooterState>();

        for (double[] state : regression) {
            shootingMap.put(
                    state[0],
                    new ShooterState(
                            state[1], state[2], state[3] == 1 ? ShooterAngle.FAR_SHOT : ShooterAngle.CLOSE_SHOT));
        }

        return shootingMap;
    }

    public static InterpolatingMap<InterpolatableDouble> loadTimingMap(double[][] regression) {
        InterpolatingMap<InterpolatableDouble> timingMap = new InterpolatingMap<InterpolatableDouble>();

        for (double[] state : regression) {
            timingMap.put(state[0], new InterpolatableDouble(state[1]));
        }

        return timingMap;
    }

    /**
     * @param shootingMap
     * @return Map as a JSON String
     */
    public static String stringifyMap(InterpolatingMap<ShooterState> shootingMap) {
        String mapAsString = shootingMap.keySet().stream()
                .map(key -> stringifyShooterMapEntry(key, shootingMap.get(key)))
                .collect(Collectors.joining(", ", "{", "}"));

        return mapAsString;
    }

    private static String stringifyShooterMapEntry(Double key, ShooterState shooterState) {
        return "\"" + key + "\": " + "[" + shooterState.rearShooterRPM + ", " + shooterState.frontShooterRPM + "]";
    }

    public static InterpolatingMap<InterpolatableDouble> getShootingTimeMap() {
        return loadTimingMap(shootingTimeRegression);
    }

    public static InterpolatingMap<ShooterState> getPracticeShootingMap() {
        return loadShootingMap(practiceRegression);
    }

    public static InterpolatingMap<ShooterState> getCompetitionShootingMap() {
        return loadShootingMap(competitionRegression);
    }
}
