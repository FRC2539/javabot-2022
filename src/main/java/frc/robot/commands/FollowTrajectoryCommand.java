package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FollowTrajectoryCommand extends CommandBase {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private Trajectory trajectory;
    private Supplier<Rotation2d> desiredRotation;

    public FollowTrajectoryCommand(SwerveDriveSubsystem swerveDriveSubsystem, Trajectory trajectory, Supplier<Rotation2d> desiredRotation) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.trajectory = trajectory;
        this.desiredRotation = desiredRotation;

        addRequirements(swerveDriveSubsystem);
    }

    public FollowTrajectoryCommand(SwerveDriveSubsystem swerveDriveSubsystem, Trajectory trajectory) {
        this(swerveDriveSubsystem, trajectory, () -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation());
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.getFollower().follow(trajectory, desiredRotation);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.getFollower().cancel();
    }

    @Override
    public boolean isFinished() {
        return swerveDriveSubsystem.getFollower().getCurrentTrajectory().isEmpty();
    }
}
