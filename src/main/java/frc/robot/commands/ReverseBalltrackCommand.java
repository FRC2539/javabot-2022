package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.BalltrackSubsystem.BalltrackMode;

public class ReverseBalltrackCommand extends CommandBase {
    private BalltrackSubsystem balltrackSubsystem;

    private BalltrackMode previousBalltrackMode;

    public ReverseBalltrackCommand(BalltrackSubsystem balltrackSubsystem) {
        this.balltrackSubsystem = balltrackSubsystem;
    }

    @Override
    public void initialize() {
        previousBalltrackMode = balltrackSubsystem.getBalltrackMode();

        balltrackSubsystem.setBalltrackMode(BalltrackMode.REVERSE);
    }

    @Override
    public void end(boolean interrupted) {
        if (balltrackSubsystem.getBalltrackMode() != BalltrackMode.REVERSE){
            return;
        } else {
            balltrackSubsystem.setBalltrackMode(previousBalltrackMode);
        }
    }
}
