package frc.robot.util;

import com.pathplanner.lib.PathPlannerTrajectory;

public class TrajectoryLoader {
    private static final String TWO_BALL_PATH = "twoball";
    private static final String TWO_BALL_FAR_PATH = "twoball far new";
    private static final String TWO_BALL_FAR_MANUAL_GYRO_PATH = "threeball";
    private static final String THREE_BALL_PATH = "threeball";
    private static final String THREE_BALL_2_PATH = "threeball 2";
    private static final String FIVE_BALL_PATH = "fiveball alt";
    private static final String FIVE_BALL_1_PATH = "fiveball 1";
    private static final String FIVE_BALL_SWEEP_PATH = "fiveball sweep";
    private static final String FIVE_BALL_2_PATH = "fiveball 2";
    private static final String FOUR_BALL_PATH = "fourball";
    private static final String FOUR_BALL_2_PATH = "fourball 2";
    private static final String FOUR_BALL_3_PATH = "fourball 3";
    private static final String TWO_BALL_STEAL_PATH = "twoball far steal";
    private static final String ONE_BALL_STEAL_PATH = "oneball far steal";

    private LazyPathPlannerTrajectory threeBall;
    private LazyPathPlannerTrajectory threeBall2;
    private LazyPathPlannerTrajectory twoBall;
    private LazyPathPlannerTrajectory twoBallFar;
    private LazyPathPlannerTrajectory twoBallFarManualGyro;
    private LazyPathPlannerTrajectory fourBall;
    private LazyPathPlannerTrajectory fourBall2;
    private LazyPathPlannerTrajectory fourBall3;
    private LazyPathPlannerTrajectory fiveBall;
    private LazyPathPlannerTrajectory fiveBall1;
    private LazyPathPlannerTrajectory fiveBallSweep;
    private LazyPathPlannerTrajectory fiveBall2;
    private LazyPathPlannerTrajectory twoBallSteal;
    private LazyPathPlannerTrajectory oneBallSteal;

    public TrajectoryLoader() {
        threeBall = new LazyPathPlannerTrajectory(THREE_BALL_PATH, 5, 6.0);
        threeBall2 = new LazyPathPlannerTrajectory(THREE_BALL_2_PATH, 5, 2.5);
        // twoBall = new LazyPathPlannerTrajectory(TWO_BALL_PATH, 5, 2.0);
        twoBallFar = new LazyPathPlannerTrajectory(THREE_BALL_PATH, 5, 4.0);
        // twoBallFarManualGyro = new LazyPathPlannerTrajectory(TWO_BALL_FAR_MANUAL_GYRO_PATH, 5, 2.5);
        // fiveBall = new LazyPathPlannerTrajectory(FIVE_BALL_PATH, 5, 2.5);
        fourBall = new LazyPathPlannerTrajectory(FOUR_BALL_PATH, 5, 2.5);
        fourBall2 = new LazyPathPlannerTrajectory(FOUR_BALL_2_PATH, 5, 6.0);
        fourBall3 = new LazyPathPlannerTrajectory(FOUR_BALL_3_PATH, 5, 6.0);
        fiveBall1 = new LazyPathPlannerTrajectory(FIVE_BALL_1_PATH, 5, 5.5);
        fiveBallSweep = new LazyPathPlannerTrajectory(FIVE_BALL_SWEEP_PATH, 2.5, 3.3);
        fiveBall2 = new LazyPathPlannerTrajectory(FIVE_BALL_2_PATH, 5, 6.0);
        // twoBallSteal = new LazyPathPlannerTrajectory(TWO_BALL_STEAL_PATH, 5, 2.5);
        // oneBallSteal = new LazyPathPlannerTrajectory(ONE_BALL_STEAL_PATH, 5, 2.5);
    }

    public PathPlannerTrajectory getThreeBall() {
        return threeBall.getTrajectory();
    }

    public PathPlannerTrajectory getThreeBall2() {
        return threeBall2.getTrajectory();
    }

    public PathPlannerTrajectory getTwoBall() {
        return twoBall.getTrajectory();
    }

    public PathPlannerTrajectory getTwoBallFar() {
        return twoBallFar.getTrajectory();
    }

    public PathPlannerTrajectory getTwoBallFarManualGyro() {
        return twoBallFarManualGyro.getTrajectory();
    }

    public PathPlannerTrajectory getFiveBall() {
        return fiveBall.getTrajectory();
    }

    public PathPlannerTrajectory getFiveBall1() {
        return fiveBall1.getTrajectory();
    }

    public PathPlannerTrajectory getFiveBallSweep() {
        return fiveBallSweep.getTrajectory();
    }

    public PathPlannerTrajectory getFiveBall2() {
        return fiveBall2.getTrajectory();
    }

    public PathPlannerTrajectory getFourBall() {
        return fourBall.getTrajectory();
    }

    public PathPlannerTrajectory getFourBall2() {
        return fourBall2.getTrajectory();
    }

    public PathPlannerTrajectory getFourBall3() {
        return fourBall3.getTrajectory();
    }

    public PathPlannerTrajectory getTwoBallSteal() {
        return twoBallSteal.getTrajectory();
    }

    public PathPlannerTrajectory getOneBallSteal() {
        return oneBallSteal.getTrajectory();
    }
}
