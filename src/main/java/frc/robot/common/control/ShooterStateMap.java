package frc.robot.common.control;

import java.util.Optional;
import java.util.TreeMap;

public class ShooterStateMap extends TreeMap<Double, ShooterState> {
    public ShooterStateMap() {}

    public ShooterState put(ShooterState shooterState) {
        super.put(shooterState.distance, shooterState);

        return shooterState;
    }

    public Optional<ShooterState> getInterpolatedShooterState(double distance) {
        ShooterState shooterState = get(distance);

        if (shooterState != null)
            return Optional.of(shooterState);

        Double floor = floorKey(distance);
        Double ceiling = ceilingKey(distance);

        // Account for cases where the distance is outside the given range
        if (floor == null && ceiling == null) {
            return Optional.empty();
        } else if (floor == null) {
            return Optional.of(get(ceiling));
        } else if (ceiling == null) {
            return Optional.of(get(floor));
        }

        ShooterState floorState = get(floor);
        ShooterState ceilingState = get(ceiling);

        return Optional.of(floorState.interpolateWithDistance(ceilingState, distance));
    }
}
