package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.MachineLearningSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BallCollectCommand extends CommandBase {
    private MachineLearningSubsystem machineLearningSubsystem;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private BalltrackSubsystem balltrackSubsystem;

    private final double MAX_OUTPUT = 0.5;
    private final double PICKUP_OUTPUT = 0.2;

    private final double AVERAGE_FRAME_TIME = 0.08;

    private final double STRAFE_ANGLE_THRESHOLD = 0.3;

    private final Timer ballLostTimer = new Timer();
    private boolean ballLost = false;

    public BallCollectCommand(MachineLearningSubsystem machineLearningSubsystem, SwerveDriveSubsystem swerveDriveSubsystem, BalltrackSubsystem balltrackSubsystem) {
        this.machineLearningSubsystem = machineLearningSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.balltrackSubsystem = balltrackSubsystem;

        addRequirements(swerveDriveSubsystem, machineLearningSubsystem);
    }

    @Override
    public void initialize() {
        balltrackSubsystem.intakeMode();
    }

    @Override
    public void execute() {
        updateBallLost();

        if (ballLost || balltrackSubsystem.isBalltrackFull()) {
            swerveDriveSubsystem.stop();
            return;
        }

        double horizontalAngle = machineLearningSubsystem.getHorizontalAngle();

        double forwardVelocity = machineLearningSubsystem.isAtBall() ? PICKUP_OUTPUT : MAX_OUTPUT * (1 - Math.abs(horizontalAngle));
        double strafeVelocity = Math.abs(horizontalAngle) < STRAFE_ANGLE_THRESHOLD ? 0 : (MAX_OUTPUT * 0.55) * horizontalAngle;

        ChassisSpeeds velocity = new ChassisSpeeds(
            -forwardVelocity,
            -strafeVelocity,
            0 
        );

        swerveDriveSubsystem.drive(velocity, false);
    }

    private void updateBallLost() {
        boolean hasTarget = machineLearningSubsystem.hasTarget();

        if(!hasTarget && !ballLost) {
            ballLostTimer.reset();
            ballLostTimer.start();
        } else if (!hasTarget && ballLostTimer.hasElapsed(AVERAGE_FRAME_TIME)) {
            ballLost = true;
        } else {
            ballLost = false;
            ballLostTimer.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        swerveDriveSubsystem.stop();

        balltrackSubsystem.stopIntakeMode();
    }
}
