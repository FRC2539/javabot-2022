package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShootingSuperstructure;
import frc.robot.strategies.MovingAimStrategy;
import frc.robot.subsystems.ShooterSubsystem;

public class MovingSpinUpCommand extends CommandBase {
    private ShootingSuperstructure shootingSuperstructure;
    private ShooterSubsystem shooterSubsystem;
    private MovingAimStrategy aimStrategy;

    public MovingSpinUpCommand(ShootingSuperstructure shootingSuperstructure, MovingAimStrategy movingAimStrategy) {
        this.shootingSuperstructure = shootingSuperstructure;
        shooterSubsystem = shootingSuperstructure.getShooterSubsystem();
        aimStrategy = movingAimStrategy;
    }

    @Override
    public void initialize() {
        shootingSuperstructure.getLimelightSubsystem().bindUpdatable(aimStrategy);
        shootingSuperstructure.activateShootingPipeline();
        shooterSubsystem.setFarShot(() -> aimStrategy.calculateShotDistance());
    }

    @Override
    public void end(boolean interrupted) {
        shootingSuperstructure.getLimelightSubsystem().freeUpdatable();
    }
}
