package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.MachineLearningSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.LoggingCommand;

public class BallCollectCommand extends LoggingCommand {
    private MachineLearningSubsystem machineLearningSubsystem;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private BalltrackSubsystem balltrackSubsystem;

    private ProfiledPIDController forwardController = new ProfiledPIDController(
            4,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    SwerveDriveSubsystem.MAX_VELOCITY / 2, SwerveDriveSubsystem.MAX_VELOCITY / 6));

    private ProfiledPIDController strafeController = new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    SwerveDriveSubsystem.MAX_VELOCITY / 2, SwerveDriveSubsystem.MAX_VELOCITY / 4));

    private TrapezoidProfile.State forwardGoal = new TrapezoidProfile.State(0, 0);

    private double strafeGoal = 0;

    private boolean collectionComplete = false;

    private final boolean shouldCollectTwo;
    private boolean collectTwo;

    private NetworkTableEntry forwardOffsetEntry = getEntry("forwardOffset");

    public BallCollectCommand(
            MachineLearningSubsystem machineLearningSubsystem,
            SwerveDriveSubsystem swerveDriveSubsystem,
            BalltrackSubsystem balltrackSubsystem) {
        this(machineLearningSubsystem, swerveDriveSubsystem, balltrackSubsystem, false);
    }

    public BallCollectCommand(
            MachineLearningSubsystem machineLearningSubsystem,
            SwerveDriveSubsystem swerveDriveSubsystem,
            BalltrackSubsystem balltrackSubsystem,
            boolean shouldCollectTwo) {
        this.machineLearningSubsystem = machineLearningSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.balltrackSubsystem = balltrackSubsystem;

        addRequirements(swerveDriveSubsystem, machineLearningSubsystem);

        forwardController.setTolerance(MachineLearningSubsystem.STOPPING_TOLERANCE);

        this.shouldCollectTwo = shouldCollectTwo;

        // Shuffleboard.getTab("Testing")
        //         .add("Forward", forwardController)
        //         .withPosition(2, 0);
        // Shuffleboard.getTab("Testing")
        //         .add("Strafe", strafeController)
        //         .withWidget(BuiltInWidgets.kPIDController)
        //         .withPosition(2, 1);
    }

    @Override
    public void initialize() {
        balltrackSubsystem.intakeMode();

        resetControllers();

        collectionComplete = false;

        collectTwo = shouldCollectTwo;

        // Make sure that we only collect one ball if we already have one
        if (balltrackSubsystem.hasOneBall()) collectTwo = false;
    }

    @Override
    public void execute() {
        // System.out.println(collectionComplete + " : " + getForwardOffset());

        forwardOffsetEntry.setDouble(getForwardOffset());

        if (collectionComplete || balltrackSubsystem.isBalltrackFull()) {
            swerveDriveSubsystem.stop();

            return;
        }

        double forwardVelocity = forwardController.calculate(getForwardOffset(), forwardGoal);
        double strafeVelocity = strafeController.calculate(machineLearningSubsystem.getHorizontalAngle(), strafeGoal);

        updateCollectionComplete();

        ChassisSpeeds velocity = new ChassisSpeeds(forwardVelocity, strafeVelocity, 0);

        swerveDriveSubsystem.drive(velocity, false);
    }

    private double getForwardOffset() {
        // Scale the y value of the target from 0 to 1, where 0 is at the target.
        return (MachineLearningSubsystem.STOPPING_Y - machineLearningSubsystem.getTargetY())
                / MachineLearningSubsystem.STOPPING_Y;
    }

    private void resetControllers() {
        forwardController.reset(getForwardOffset());
        strafeController.reset(machineLearningSubsystem.getHorizontalAngle());
    }

    private void updateCollectionComplete() {
        // No need to run the rest of the method if we are done collecting balls
        if (collectionComplete) return;

        // Evaluate if we are done collecting balls
        if (collectTwo && forwardController.atGoal()) {
            swerveDriveSubsystem.stop();

            resetControllers();

            collectTwo = false;
        } else if (!collectTwo && forwardController.atGoal()) {
            collectionComplete = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();

        balltrackSubsystem.stopIntakeMode();
    }
}
