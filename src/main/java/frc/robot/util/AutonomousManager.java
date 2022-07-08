package frc.robot.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.EntryListenerFlags;
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
    private RobotContainer container;

    private NetworkTable autonomousTable;

    private NetworkTableEntry selectedAuto;

    private final String[] autoStrings = {"twoball", "threeball", "fiveball", "fourball"};

    private Command selectedAutonomousCommand;

    public AutonomousManager(TrajectoryLoader trajectoryLoader, RobotContainer container) {
        this.trajectoryLoader = trajectoryLoader;
        this.container = container;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        autonomousTable = inst.getTable("Autonomous");

        // Insert all of the auto options into the network tables
        autonomousTable.getEntry("autos").setStringArray(autoStrings);

        selectedAuto = autonomousTable.getEntry("selectedAuto");

        // Automatically load the selected auto and print a success message
        selectedAuto.addListener(
                event -> {
                    selectedAutonomousCommand = loadAutonomousCommand();

                    System.out.println("\nAuto loaded: " + event.getEntry().getString("null") + "\n");
                },
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        // Choose the first auto as the default
        selectedAuto.setString(autoStrings[0]);
    }

    public Command getTwoBallCommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectoryLoader.getTwoBall());

        followAndIntake(command, trajectoryLoader.getTwoBall());
        shootBallsAndAim(command, 2, true);

        return command;
    }

    public Command getThreeBallCommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectoryLoader.getThreeBall());

        followAndIntakeWithTimeout(command, trajectoryLoader.getThreeBall(), 2.5);
        shootBallsAndAim(command, 0.5, false);
        shootAndIntake(command, 2.7);

        return command;
    }

    public Command getFiveBallCommand() {
        SequentialCommandGroup command = (SequentialCommandGroup) getThreeBallCommand();

        followAndIntake(command, trajectoryLoader.getFiveBall());

        shootBallsAndAim(command, 2.5, true);

        return command;
    }

    public Command getFourBallCommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectoryLoader.getFourBall());

        followAndIntake(command, trajectoryLoader.getFourBall());

        shootBallsAndAim(command, 0.7, false);
        shootAndIntake(command, 1.6);

        followAndIntake(command, trajectoryLoader.getFiveBall1());

        intakeInPlace(command, 2.5);

        followAndIntake(command, trajectoryLoader.getFiveBall2());

        shootBallsAndAim(command, 2.5, true);

        return command;
    }

    private void shootBalls(SequentialCommandGroup command, double timeout) {
        command.addCommands(
                new LimelightShootCommand(container.getShootingSuperstructure(), true).withTimeout(timeout));
    }

    private void shootBallsAndAim(SequentialCommandGroup command, double timeout, boolean stopShooting) {
        command.addCommands(new LimelightShootCommand(container.getShootingSuperstructure(), stopShooting)
                .alongWith(new LimelightDriveCommand(
                        () -> 0.0, () -> 0.0, container.getShootingSuperstructure(), container.getLightsSubsystem()))
                .withTimeout(timeout));
    }

    private void shootAndIntake(SequentialCommandGroup command, double timeout) {
        command.addCommands(new LimelightShootCommand(container.getShootingSuperstructure(), true)
                .alongWith(new IntakeCommand(container.getBalltrackSubsystem(), container.getLightsSubsystem()))
                .withTimeout(timeout));
    }

    private void follow(SequentialCommandGroup command, PathPlannerTrajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getSwerveDriveSubsystem(), trajectory));
    }

    private void followAndPrepare(SequentialCommandGroup command, PathPlannerTrajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getSwerveDriveSubsystem(), trajectory)
                .deadlineWith(new PrepareToShootCommand(container.getShootingSuperstructure())));
    }

    private void followAndIntake(SequentialCommandGroup command, PathPlannerTrajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getSwerveDriveSubsystem(), trajectory)
                .deadlineWith(new IntakeCommand(container.getBalltrackSubsystem(), container.getLightsSubsystem())));
    }

    private void intakeInPlace(SequentialCommandGroup command, double timeout) {
        command.addCommands(new IntakeCommand(container.getBalltrackSubsystem(), container.getLightsSubsystem())
                .withTimeout(timeout));
    }

    private void followAndIntakeWithTimeout(
            SequentialCommandGroup command, PathPlannerTrajectory trajectory, double intakeTimeout) {
        command.addCommands(new FollowTrajectoryCommand(container.getSwerveDriveSubsystem(), trajectory)
                .deadlineWith(new IntakeCommand(container.getBalltrackSubsystem(), container.getLightsSubsystem())
                        .withTimeout(intakeTimeout)));
    }

    private void resetRobotPose(SequentialCommandGroup command, Trajectory trajectory) {
        command.addCommands(
                new InstantCommand(() -> container.getSwerveDriveSubsystem().resetGyroAngle()));
        command.addCommands(
                new InstantCommand(() -> container.getSwerveDriveSubsystem().resetPose(trajectory.getInitialPose())));
    }

    public Command loadAutonomousCommand() {
        switch (selectedAuto.getString(autoStrings[0])) {
            case "twoball":
                return getTwoBallCommand();
            case "threeball":
                return getThreeBallCommand();
            case "fiveball":
                return getFiveBallCommand();
            case "fourball":
                return getFourBallCommand();
        }

        // Return an empty command group if no auto is specified
        return new SequentialCommandGroup();
    }

    public Command getAutonomousCommand() {
        return selectedAutonomousCommand;
    }
}
