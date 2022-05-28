package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SimpleShootCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private BalltrackSubsystem balltrackSubsystem;
    private Runnable setShooterStateRunnable;
    
    public SimpleShootCommand(ShooterSubsystem shooterSubsystem, BalltrackSubsystem balltrackSubsystem, Runnable setShooterStateRunnable) {
        this.shooterSubsystem = shooterSubsystem;
        this.setShooterStateRunnable = setShooterStateRunnable;
        this.balltrackSubsystem = balltrackSubsystem;

        addRequirements(shooterSubsystem);
    } 

    @Override
    public void initialize() {
        setShooterStateRunnable.run();

        balltrackSubsystem.shootMode();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();

        balltrackSubsystem.stopShootMode();
    }
}
