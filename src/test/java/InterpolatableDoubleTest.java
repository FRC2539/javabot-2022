import static org.junit.Assert.*;

import frc.lib.control.InterpolatingMap;
import frc.lib.interpolation.InterpolatableDouble;
import org.junit.*;

public class InterpolatableDoubleTest {
    InterpolatingMap<InterpolatableDouble> shotTimeMap;

    @Before
    public void setup() {
        shotTimeMap = new InterpolatingMap<InterpolatableDouble>();

        shotTimeMap.put(1, new InterpolatableDouble(5));
        shotTimeMap.put(3, new InterpolatableDouble(14));
    }

    public InterpolatableDouble calculateShotTimeForDistance(double distance) {
        return shotTimeMap.getInterpolated(distance).orElse(new InterpolatableDouble(0));
    }

    @Test
    public void calculatesCorrectShotTimes() {
        InterpolatableDouble firstState = calculateShotTimeForDistance(1);
        InterpolatableDouble middleState = calculateShotTimeForDistance(2);
        InterpolatableDouble lastState = calculateShotTimeForDistance(2.5);

        assertEquals(firstState, new InterpolatableDouble(5));
        assertEquals(middleState, new InterpolatableDouble(9.5));
        assertEquals(lastState, new InterpolatableDouble(11.75));
    }
}
