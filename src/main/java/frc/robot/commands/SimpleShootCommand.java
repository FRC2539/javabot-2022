package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SimpleShootCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private Runnable setShooterStateRunnable;
    
    public SimpleShootCommand(ShooterSubsystem shooterSubsystem, Runnable setShooterStateRunnable) {
        this.shooterSubsystem = shooterSubsystem;
        this.setShooterStateRunnable = setShooterStateRunnable;
    } 

    @Override
    public void initialize() {
        setShooterStateRunnable.run();
    }

    @Override
    public void execute() {
        // Move balls through the robot
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
    }
}
