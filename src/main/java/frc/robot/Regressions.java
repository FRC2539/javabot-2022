package frc.robot;

import frc.robot.common.control.InterpolatingMap;
import frc.robot.common.control.ShooterState;

public class Regressions {
    public static double[][] practiceRegression = {
        {3, 2300, 1550},
        {3.6, 2600, 1550},
        {4.3, 3150, 1550},
        {4.57, 3700, 1800},
        {6.03, 4200, 2000},
    };

    public static InterpolatingMap<ShooterState> loadShootingMap(double[][] regression) {
        InterpolatingMap<ShooterState> shootingMap = new InterpolatingMap<ShooterState>();

        for (double[] state : regression) {
            shootingMap.put(state[0], new ShooterState(state[1], state[2]));
        }

        return shootingMap;
    }

    public static InterpolatingMap<ShooterState> getPracticeShootingMap() {
        return loadShootingMap(practiceRegression);
    }
}
