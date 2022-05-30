package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryLoader {
    private static final String THREE_BALL_ONE_PATH = "output/threeball1.wpilib.json";
    private static final String THREE_BALL_TWO_PATH = "output/threeball2.wpilib.json";
    private static final String TWO_BALL_MAIN_PATH = "output/twoballmain.wpilib.json";
    private static final String TWO_BALL_SIDE_PATH = "output/twoballside.wpilib.json";

    private Trajectory threeBallOne;
    private Trajectory threeBallTwo;
    private Trajectory twoBallMain;
    private Trajectory twoBallSide;

    public TrajectoryLoader() throws IOException {
        threeBallOne = TrajectoryUtil.fromPathweaverJson(getPath(THREE_BALL_ONE_PATH));
        threeBallTwo = TrajectoryUtil.fromPathweaverJson(getPath(THREE_BALL_TWO_PATH));
        twoBallMain = TrajectoryUtil.fromPathweaverJson(getPath(TWO_BALL_MAIN_PATH));
        twoBallSide = TrajectoryUtil.fromPathweaverJson(getPath(TWO_BALL_SIDE_PATH));
    }

    private Path getPath(String path) {
        return Filesystem.getDeployDirectory().toPath().resolve(path);
    }

    public Trajectory getThreeBallOne() {
        return threeBallOne;
    }

    public Trajectory getThreeBallTwo() {
        return threeBallTwo;
    }

    public Trajectory getTwoBallMain() {
        return twoBallMain;
    }

    public Trajectory getTwoBallSide() {
        return twoBallSide;
    }
}
