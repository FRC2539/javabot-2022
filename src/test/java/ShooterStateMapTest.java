import static org.junit.Assert.*;

import org.frc2539.cougarlib.control.InterpolatingMap;
import frc.robot.util.ShooterState;

import org.junit.*;

public class ShooterStateMapTest {
    InterpolatingMap<ShooterState> shooterStateMap;

    @Before
    public void setup() {
        shooterStateMap = new InterpolatingMap<ShooterState>();

        shooterStateMap.put(0.8, new ShooterState(2400, 1800));
        shooterStateMap.put(3, new ShooterState(4160, 3120));
    }

    public ShooterState calculateShooterStateForDistance(double distance) {
        return shooterStateMap.getInterpolated(distance).orElse(new ShooterState());
    }

    @Test
    public void calculatesCorrectShooterStates() {
        ShooterState firstState = calculateShooterStateForDistance(0.8);
        ShooterState middleState = calculateShooterStateForDistance(1);
        ShooterState lastState = calculateShooterStateForDistance(3);

        assertEquals(firstState, new ShooterState(2400, 1800));
        assertEquals(middleState, new ShooterState(2560, 1920));
        assertEquals(lastState, new ShooterState(4160, 3120));
    }
}
