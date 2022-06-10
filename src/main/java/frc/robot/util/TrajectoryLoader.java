package frc.robot.util;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;

public class TrajectoryLoader {
    private static final String TWO_BALL_PATH = "twoball";
    private static final String THREE_BALL_PATH = "threeball";
    private static final String FIVE_BALL_PATH = "fiveball";

    private PathPlannerTrajectory threeBall;
    private PathPlannerTrajectory twoBall;
    private PathPlannerTrajectory fiveBall;

    public TrajectoryLoader() throws IOException {
        threeBall = loadTrajectory(THREE_BALL_PATH, 5, 2.5);
        twoBall = loadTrajectory(TWO_BALL_PATH, 5, 2.5);
        fiveBall = loadTrajectory(FIVE_BALL_PATH, 5, 2.5);

        System.out.println("\nAll trajectories loaded successfully.\n");
    }

    private PathPlannerTrajectory loadTrajectory(String path, double maxVel, double maxAccel) throws IOException {
        return PathPlanner.loadPath(path, maxVel, maxAccel);
    }

    private Trajectory loadTrajectory(String path) throws IOException {
        return TrajectoryUtil.fromPathweaverJson(getPath(path));
    }

    private Path getPath(String path) {
        return Filesystem.getDeployDirectory().toPath().resolve(path);
    }

    public PathPlannerTrajectory getThreeBall() {
        return threeBall;
    }

    public PathPlannerTrajectory getTwoBall() {
        return twoBall;
    }

    public PathPlannerTrajectory getFiveBall() {
        return fiveBall;
    }
}
