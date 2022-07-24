package frc.robot.util;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class LazyPathPlannerTrajectory {
    private final String trajectoryPath;
    private final double maxVelocity;
    private final double maxAcceleration;

    // private Optional<PathPlannerTrajectory> trajectory = Optional.empty();

    private PathPlannerTrajectory trajectory;

    public LazyPathPlannerTrajectory(String trajectoryPath, double maxVelocity, double maxAcceleration) {
        this.trajectoryPath = trajectoryPath;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;

        this.trajectory = loadTrajectory(trajectoryPath, maxVelocity, maxAcceleration);
    }

    public PathPlannerTrajectory getTrajectory() {
        // Loads the trajectory only at the first request
        // if (trajectory.isEmpty())
        //     trajectory = Optional.of(loadTrajectory(trajectoryPath, maxVelocity, maxAcceleration));

        // return trajectory.orElse(null);

        return trajectory;
    }

    private PathPlannerTrajectory loadTrajectory(String path, double maxVel, double maxAccel) {
        return PathPlanner.loadPath(path, maxVel, maxAccel);
    }
}
