import static org.junit.Assert.*;

import frc.robot.common.control.ShooterState;
import frc.robot.common.control.ShooterStateMap;

import org.junit.*;

public class ShooterStateMapTest {
    ShooterStateMap shooterStateMap;

    @Before 
    public void setup() {
        shooterStateMap = new ShooterStateMap();

        shooterStateMap.put(new ShooterState(2400, 1800, 0.8));
        shooterStateMap.put(new ShooterState(4160, 3120, 3));
    }

    public ShooterState calculateShooterStateForDistance(double distance) {
        return shooterStateMap.getInterpolatedShooterState(distance).orElse(new ShooterState());
    }

    @Test
    public void calculatesCorrectShooterStates() {
        ShooterState firstState = calculateShooterStateForDistance(0.8);
        ShooterState middleState = calculateShooterStateForDistance(1);
        ShooterState lastState = calculateShooterStateForDistance(3);

        assertEquals(firstState, new ShooterState(2400, 1800, 0.8));
        assertEquals(middleState, new ShooterState(2560, 1920, 1));
        assertEquals(lastState, new ShooterState(4160, 3120, 3));
    }
}