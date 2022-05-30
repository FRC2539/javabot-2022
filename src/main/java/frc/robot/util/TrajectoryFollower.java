package frc.robot.util;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.common.control.SwerveDriveSignal;

public class TrajectoryFollower {
    private HolonomicDriveController driveController;

    private Trajectory currentTrajectory = null;

    private Supplier<Rotation2d> desiredRotation = null;

    private Trajectory.State lastState = null;

    private double startTime = Double.NaN;

    private boolean finished = false;

    public TrajectoryFollower(PIDController forwardController, PIDController strafeController, ProfiledPIDController rotationController) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        driveController = new HolonomicDriveController(forwardController, strafeController, rotationController);
    }

    public Optional<SwerveDriveSignal> update(Pose2d currentPose) {
        Trajectory trajectory;
        Supplier<Rotation2d> desiredRotation;
        double timeSinceStart;

        if (currentTrajectory == null || this.desiredRotation == null) {
            return Optional.empty();
        }

        // If the trajectory has not been started, update the start time and reset the follower state
        if (Double.isNaN(startTime)) {
            startTime = Timer.getFPGATimestamp();
            reset();
        } else if (isFinished()) {
            currentTrajectory = null;
            this.desiredRotation = null;
            return Optional.empty();
        }

        trajectory = currentTrajectory;
        timeSinceStart = Timer.getFPGATimestamp() - startTime;
        desiredRotation = this.desiredRotation;

        SwerveDriveSignal signal = calculateDriveSignal(currentPose, trajectory, desiredRotation, timeSinceStart);

        return Optional.of(signal);
    }

    private SwerveDriveSignal calculateDriveSignal(Pose2d currentPose, Trajectory trajectory, Supplier<Rotation2d> desiredRotation, double timeSinceStart) {
        if (timeSinceStart > trajectory.getTotalTimeSeconds()) {
            finished = true;
            return new SwerveDriveSignal();
        }

        lastState = trajectory.sample(timeSinceStart);

        return new SwerveDriveSignal(driveController.calculate(currentPose, lastState, desiredRotation.get()), false);
    }

    public Trajectory.State getLastState() {
        return lastState;
    }

    public boolean isFinished() {
        return finished;
    }

    public void reset() {
        finished = false;
    }

    public void cancel() {
        currentTrajectory = null;
    }

    public void follow(Trajectory trajectory, Supplier<Rotation2d> desiredRotation) {
        currentTrajectory = trajectory;
        this.desiredRotation = desiredRotation;
        startTime = Double.NaN;
    }

    public Optional<Trajectory> getCurrentTrajectory() {
        return Optional.ofNullable(currentTrajectory);
    }
}
