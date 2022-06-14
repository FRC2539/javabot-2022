package frc.robot.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LimelightDriveCommand;
import frc.robot.commands.LimelightShootCommand;
import frc.robot.commands.PrepareToShootCommand;

public class AutonomousManager {
    private TrajectoryLoader trajectoryLoader;

    private NetworkTable autonomousTable;

    private NetworkTableEntry selectedAuto;

    private final String[] autoStrings = {"twoball", "threeball", "fiveball", "fourball"};

    public AutonomousManager(TrajectoryLoader trajectoryLoader) {
        this.trajectoryLoader = trajectoryLoader;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        autonomousTable = inst.getTable("Autonomous");

        autonomousTable.getEntry("autos").setStringArray(autoStrings);

        selectedAuto = autonomousTable.getEntry("selectedAuto");

        // Choose the first auto as the default
        selectedAuto.setString(autoStrings[0]);
    }

    public Command getTwoBallCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectoryLoader.getTwoBall());

        followAndIntake(command, container, trajectoryLoader.getTwoBall());
        shootBallsAndAim(command, container, 2, true);

        return command;
    }

    public Command getThreeBallCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectoryLoader.getThreeBall());

        followAndIntakeWithTimeout(command, container, trajectoryLoader.getThreeBall(), 2.5);
        shootBallsAndAim(command, container, 0.5, false);
        shootAndIntake(command, container, 2.7);

        return command;
    }

    public Command getFiveBallCommand(RobotContainer container) {
        SequentialCommandGroup command = (SequentialCommandGroup) getThreeBallCommand(container);

        followAndIntake(command, container, trajectoryLoader.getFiveBall());

        shootBallsAndAim(command, container, 2.5, true);

        return command;
    }

    public Command getFourBallCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectoryLoader.getFourBall());

        followAndIntake(command, container, trajectoryLoader.getFourBall());

        shootBallsAndAim(command, container, 0.3, false);
        shootAndIntake(command, container, 2);

        followAndIntake(command, container, trajectoryLoader.getFiveBall1());

        intakeInPlace(command, container, 1.5);

        followAndIntake(command, container, trajectoryLoader.getFiveBall2());

        shootBallsAndAim(command, container, 2.5, true);

        return command;
    }

    private void shootBalls(SequentialCommandGroup command, RobotContainer container, double timeout) {
        command.addCommands(new LimelightShootCommand(
                        container.getShooterSubsystem(),
                        container.getBalltrackSubsystem(),
                        container.getLimelightSubsystem(),
                        true)
                .withTimeout(timeout));
    }

    private void shootBallsAndAim(
            SequentialCommandGroup command, RobotContainer container, double timeout, boolean stopShooting) {
        command.addCommands(new LimelightShootCommand(
                        container.getShooterSubsystem(),
                        container.getBalltrackSubsystem(),
                        container.getLimelightSubsystem(),
                        stopShooting)
                .alongWith(new LimelightDriveCommand(
                        container.getSwerveDriveSubsystem(),
                        () -> 0.0,
                        () -> 0.0,
                        container.getLimelightSubsystem(),
                        container.getLightsSubsystem()))
                .withTimeout(timeout));
    }

    private void shootAndIntake(SequentialCommandGroup command, RobotContainer container, double timeout) {
        command.addCommands(new LimelightShootCommand(
                        container.getShooterSubsystem(),
                        container.getBalltrackSubsystem(),
                        container.getLimelightSubsystem(),
                        true)
                .alongWith(new IntakeCommand(container.getBalltrackSubsystem()))
                .withTimeout(timeout));
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, PathPlannerTrajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getSwerveDriveSubsystem(), trajectory));
    }

    private void followAndPrepare(
            SequentialCommandGroup command, RobotContainer container, PathPlannerTrajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getSwerveDriveSubsystem(), trajectory)
                .deadlineWith(new PrepareToShootCommand(
                        container.getBalltrackSubsystem(),
                        container.getShooterSubsystem(),
                        container.getLimelightSubsystem())));
    }

    private void followAndIntake(
            SequentialCommandGroup command, RobotContainer container, PathPlannerTrajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getSwerveDriveSubsystem(), trajectory)
                .deadlineWith(new IntakeCommand(container.getBalltrackSubsystem())));
    }

    private void intakeInPlace(SequentialCommandGroup command, RobotContainer container, double timeout) {
        command.addCommands(new IntakeCommand(container.getBalltrackSubsystem()).withTimeout(timeout));
    }

    private void followAndIntakeWithTimeout(
            SequentialCommandGroup command,
            RobotContainer container,
            PathPlannerTrajectory trajectory,
            double intakeTimeout) {
        command.addCommands(new FollowTrajectoryCommand(container.getSwerveDriveSubsystem(), trajectory)
                .deadlineWith(new IntakeCommand(container.getBalltrackSubsystem()).withTimeout(intakeTimeout)));
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(
                new InstantCommand(() -> container.getSwerveDriveSubsystem().resetGyroAngle()));
        command.addCommands(
                new InstantCommand(() -> container.getSwerveDriveSubsystem().resetPose(trajectory.getInitialPose())));
    }

    public Command getAutonomousCommand(RobotContainer container) {
        switch (selectedAuto.getString(autoStrings[0])) {
            case "twoball":
                return getTwoBallCommand(container);
            case "threeball":
                return getThreeBallCommand(container);
            case "fiveball":
                return getFiveBallCommand(container);
            case "fourball":
                return getFourBallCommand(container);
        }

        // Return an empty command group if no auto is specified
        return new SequentialCommandGroup();
    }
}
