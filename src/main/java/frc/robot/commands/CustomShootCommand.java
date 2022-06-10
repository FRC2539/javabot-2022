package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CustomShootCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private BalltrackSubsystem balltrackSubsystem;

    public CustomShootCommand(ShooterSubsystem shooterSubsystem, BalltrackSubsystem balltrackSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.balltrackSubsystem = balltrackSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.setCustomShot();

        balltrackSubsystem.shootMode();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();

        balltrackSubsystem.stopShootMode();
    }
}
