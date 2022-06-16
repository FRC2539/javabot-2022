package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.MachineLearningSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BallCollectCommand extends CommandBase {
    private MachineLearningSubsystem machineLearningSubsystem;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private BalltrackSubsystem balltrackSubsystem;

    private ProfiledPIDController forwardController = new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    SwerveDriveSubsystem.MAX_VELOCITY / 2, SwerveDriveSubsystem.MAX_VELOCITY / 4));

    private ProfiledPIDController strafeController = new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    SwerveDriveSubsystem.MAX_VELOCITY / 2, SwerveDriveSubsystem.MAX_VELOCITY / 4));

    private TrapezoidProfile.State forwardGoal =
            new TrapezoidProfile.State(MachineLearningSubsystem.STOPPING_DISTANCE, 0);

    private double strafeGoal = 0;

    private boolean collectionComplete = false;

    // private boolean hasInitialBall = false;

    public BallCollectCommand(
            MachineLearningSubsystem machineLearningSubsystem,
            SwerveDriveSubsystem swerveDriveSubsystem,
            BalltrackSubsystem balltrackSubsystem) {
        this.machineLearningSubsystem = machineLearningSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.balltrackSubsystem = balltrackSubsystem;

        addRequirements(swerveDriveSubsystem, machineLearningSubsystem);

        forwardController.setTolerance(MachineLearningSubsystem.FORWARD_TOLERANCE);
    }

    @Override
    public void initialize() {
        balltrackSubsystem.intakeMode();

        forwardController.reset(
                machineLearningSubsystem.getBallDistance().orElse(0),
                swerveDriveSubsystem.getVelocity().vxMetersPerSecond);
        strafeController.reset(
                machineLearningSubsystem.getHorizontalAngle(), swerveDriveSubsystem.getVelocity().vyMetersPerSecond);

        // hasInitialBall = balltrackSubsystem.hasOneBall();

        collectionComplete = false;
    }

    @Override
    public void execute() {
        if (collectionComplete || balltrackSubsystem.isBalltrackFull()) {
            swerveDriveSubsystem.stop();
            return;
        }

        double forwardVelocity = forwardController.calculate(
                machineLearningSubsystem.getBallDistance().orElse(0), forwardGoal);
        double strafeVelocity = strafeController.calculate(machineLearningSubsystem.getHorizontalAngle(), strafeGoal);

        updateCollectionComplete();

        ChassisSpeeds velocity = new ChassisSpeeds(forwardVelocity, strafeVelocity, 0);

        swerveDriveSubsystem.drive(velocity, false);
    }

    private void updateCollectionComplete() {
        // if (collectionComplete
        //         || machineLearningSubsystem.isAtBall()
        //         || (!hasInitialBall && balltrackSubsystem.hasOneBall())) {
        //     collectionComplete = true;
        // }

        System.out.println(forwardController.atGoal() + " " + forwardController.atSetpoint());

        if (collectionComplete || forwardController.atGoal()) {
            collectionComplete = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        swerveDriveSubsystem.stop();

        balltrackSubsystem.stopIntakeMode();
    }
}
