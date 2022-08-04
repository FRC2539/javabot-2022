package frc.robot;

import com.team2539.cougarlib.control.InterpolatingMap;
import frc.robot.util.ShooterState;
import java.util.stream.Collectors;

public class Regressions {
    public static double[][] practiceRegression = {
        {3, 2300, 1550},
        {3.6, 2600, 1550},
        {4.3, 3150, 1550},
        {5.57, 3700, 1800},
        {6.03, 4200, 2000},
    };

    public static double[][] competitionRegression = {
        {3, 2300, 1550},
        {3.6, 2550, 1550},
        {4.3, 3150, 1550},
        {5.57, 3700, 1800},
        {6.03, 4200, 2000},
    };

    public static InterpolatingMap<ShooterState> loadShootingMap(double[][] regression) {
        InterpolatingMap<ShooterState> shootingMap = new InterpolatingMap<ShooterState>();

        for (double[] state : regression) {
            shootingMap.put(state[0], new ShooterState(state[1], state[2]));
        }

        return shootingMap;
    }

    public static String stringifyMap(InterpolatingMap<ShooterState> shootingMap) {
        String mapAsString = shootingMap.keySet().stream()
                .map(key -> stringifyShooterMapEntry(key, shootingMap.get(key)))
                .collect(Collectors.joining(", ", "{", "}"));

        return mapAsString;
    }

    private static String stringifyShooterMapEntry(Double key, ShooterState shooterState) {
        return key + ": " + "{" + shooterState.rearShooterRPM + ", " + shooterState.frontShooterRPM + "}";
    }

    public static InterpolatingMap<ShooterState> getPracticeShootingMap() {
        return loadShootingMap(practiceRegression);
    }

    public static InterpolatingMap<ShooterState> getCompetitionShootingMap() {
        return loadShootingMap(competitionRegression);
    }
}
