package frc.robot.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

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
import frc.robot.commands.ReverseBalltrackCommand;
import frc.robot.commands.SimpleShootCommand;

public class AutonomousManager {
    private TrajectoryLoader trajectoryLoader;
    private RobotContainer container;

    private NetworkTable autonomousTable;

    private NetworkTableEntry selectedAuto;

    private final String[] autoStrings = {
        "twoballfar", "twoball","threeball", "fiveball", "fourball", "twoballsteal", "oneballsteal"
    };

    public AutonomousManager(TrajectoryLoader trajectoryLoader, RobotContainer container) {
        this.trajectoryLoader = trajectoryLoader;
        this.container = container;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        autonomousTable = inst.getTable("Autonomous");

        // Insert all of the auto options into the network tables
        autonomousTable.getEntry("autos").setStringArray(autoStrings);

        selectedAuto = autonomousTable.getEntry("selectedAuto");

        // Choose the first auto as the default
        selectedAuto.setString(autoStrings[0]);
    }

    public Command getTwoBallCommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectoryLoader.getThreeBall());

        followAndIntake(command, trajectoryLoader.getThreeBall());
        shootBallsAndAim(command, 3, true);

        return command;
        // SequentialCommandGroup command = new SequentialCommandGroup();

        // resetRobotPose(command, trajectoryLoader.getTwoBall());

        // followAndIntake(command, trajectoryLoader.getTwoBall());
        // shootBallsAndAim(command, 2, true);

        // return command;
    }

    public Command getTwoBallFarCommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectoryLoader.getTwoBallFar());
        

        followAndIntake(command, trajectoryLoader.getTwoBallFar());
        shootBallsAndAim(command, 3, true);

        return command;
    }

    // public Command getTwoBallFarManualGyroCommand() {
    //     SequentialCommandGroup command = new SequentialCommandGroup();
    //     PathPlannerState initialState = trajectoryLoader.getTwoBallFar().getInitialState();

    //     resetRobotPose(command, trajectoryLoader.getTwoBallFarManualGyro());

    //     container.getSwerveDriveSubsystem().resetGyroAngle(initialState.holonomicRotation.times(-1));

    //     followAndIntake(command, trajectoryLoader.getTwoBallFar());
    //     shootBallsAndAim(command, 3, true);

    //     return command;
    // }

    public Command getThreeBallCommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectoryLoader.getThreeBall());

        followAndIntake(command, trajectoryLoader.getThreeBall());
        shootBallsAndAim(command, 2.0, true);

        followAndIntake(command, trajectoryLoader.getThreeBall2());
        shootBallsAndAim(command, 2.0, true);

        return command;
    }

    public Command getFiveBallCommand() {
        SequentialCommandGroup command = (SequentialCommandGroup) getThreeBallCommand();

        followAndIntake(command, trajectoryLoader.getFiveBall1());
        intakeInPlace(command, 2);
        followAndIntake(command, trajectoryLoader.getFiveBall2());
        shootBallsAndAim(command, 3.0, true);

        return command;
    }

    public Command getFourBallCommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectoryLoader.getFourBall());

        followAndIntake(command, trajectoryLoader.getFourBall());

        shootBallsAndAim(command, 1.7, true);

        followAndIntake(command, trajectoryLoader.getFourBall2());

        intakeInPlace(command, 2.5);

        followAndIntake(command, trajectoryLoader.getFourBall3());

        shootBallsAndAim(command, 2.5, true);

        return command;
    }

    private Command getTwoBallStealCommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectoryLoader.getTwoBallFar());

        followAndIntake(command, trajectoryLoader.getTwoBallFar());
        shootBallsAndAim(command, 2, true);

        followAndIntake(command, trajectoryLoader.getTwoBallSteal());
        shootWithoutLimelight(command, 2);

        return command;
    }

    private Command getOneBallStealCommand() {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, trajectoryLoader.getTwoBallFar());

        followAndIntake(command, trajectoryLoader.getTwoBallFar());
        shootBallsAndAim(command, 2, true);

        followAndIntakeWithTimeout(command, trajectoryLoader.getOneBallSteal(), 2);
        reverseIntake(command, 2);

        return command;
    }

    private void shootWithoutLimelight(SequentialCommandGroup command, double timeout) {
        command.addCommands(new SimpleShootCommand(
                        container.getShootingSuperstructure(),
                        () -> container.getShooterSubsystem().setFenderLowGoalShot())
                .withTimeout(timeout));
    }

    private void reverseIntake(SequentialCommandGroup command, double timeout) {
        command.addCommands(
                new ReverseBalltrackCommand(container.getBalltrackSubsystem(), container.getShooterSubsystem())
                        .withTimeout(timeout));
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

    private void resetRobotPose(SequentialCommandGroup command, PathPlannerTrajectory trajectory) {
        PathPlannerState initialState = trajectory.getInitialState();

        command.addCommands(new InstantCommand(
                () -> container.getSwerveDriveSubsystem().resetGyroAngle(initialState.holonomicRotation.times(-1)))); // might need to reverse this angle
        command.addCommands(
                new InstantCommand(() -> container.getSwerveDriveSubsystem().resetPose(initialState.poseMeters)));
    }

    public Command loadAutonomousCommand() {
        switch (selectedAuto.getString(autoStrings[0])) {
            case "twoball":
                return getTwoBallCommand();
            case "twoballfar":
                return getTwoBallFarCommand();
            case "threeball":
                return getThreeBallCommand();
            case "fiveball":
                return getFiveBallCommand();
            case "fourball":
                return getFourBallCommand();
            case "twoballsteal":
                return getTwoBallStealCommand();
            case "oneballsteal":
                return getOneBallStealCommand();
            // case "twoballfarmanual":
            //     return getTwoBallFarManualGyroCommand();
        }

        // Return an empty command group if no auto is specified
        return new SequentialCommandGroup();
    }

    public Command getAutonomousCommand() {
        return loadAutonomousCommand();
    }
}
