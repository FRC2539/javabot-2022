package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.BalltrackSubsystem.BalltrackMode;

public class PrepareBallsCommand extends CommandBase {
    private BalltrackSubsystem balltrackSubsystem;
    
    public PrepareBallsCommand(BalltrackSubsystem balltrackSubsystem) {
        this.balltrackSubsystem = balltrackSubsystem;

        addRequirements(balltrackSubsystem);
    } 

    @Override
    public void initialize() {
        balltrackSubsystem.setBalltrackMode(BalltrackMode.PREPARE);
    }

    @Override
    public void end(boolean interrupted) {
        balltrackSubsystem.setBalltrackMode(BalltrackMode.DISABLED);
    }
}
