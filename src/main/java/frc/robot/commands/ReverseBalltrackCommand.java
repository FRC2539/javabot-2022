package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.BalltrackSubsystem.BalltrackMode;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseBalltrackCommand extends CommandBase {
    private BalltrackSubsystem balltrackSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public ReverseBalltrackCommand(BalltrackSubsystem balltrackSubsystem, ShooterSubsystem shooterSubsystem) {
        this.balltrackSubsystem = balltrackSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        balltrackSubsystem.setBalltrackMode(BalltrackMode.REVERSE);
        shooterSubsystem.setShooterPercents(-0.3, -0.3);
    }

    @Override
    public void end(boolean interrupted) {
        balltrackSubsystem.setBalltrackMode(BalltrackMode.DISABLED);
        shooterSubsystem.stopShooter();
    }
}
