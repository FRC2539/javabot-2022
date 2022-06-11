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
    private static final String FIVE_BALL_1_PATH = "fiveball 1";
    private static final String FIVE_BALL_2_PATH = "fiveball 2";
    private static final String FOUR_BALL_PATH = "fourball 1";

    private PathPlannerTrajectory threeBall;
    private PathPlannerTrajectory twoBall;
    private PathPlannerTrajectory fiveBall;
    private PathPlannerTrajectory fourBall;
    private PathPlannerTrajectory fiveBall1;
    private PathPlannerTrajectory fiveBall2;

    public TrajectoryLoader() throws IOException {
        threeBall = loadTrajectory(THREE_BALL_PATH, 5, 2.5);
        twoBall = loadTrajectory(TWO_BALL_PATH, 5, 2.5);
        fiveBall = loadTrajectory(FIVE_BALL_PATH, 5, 2.5);
        fourBall = loadTrajectory(FOUR_BALL_PATH, 5, 2.5);
        fiveBall1 = loadTrajectory(FIVE_BALL_1_PATH, 5, 2.5);
        fiveBall2 = loadTrajectory(FIVE_BALL_2_PATH, 5, 2.5);

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

    public PathPlannerTrajectory getFiveBall1() {
        return fiveBall1;
    }

    public PathPlannerTrajectory getFiveBall2() {
        return fiveBall2;
    }

    public PathPlannerTrajectory getFourBall() {
        return fourBall;
    }
}
