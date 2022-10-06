package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShootingSuperstructure;
import frc.robot.strategies.MovingAimStrategy;
import frc.robot.subsystems.ShooterSubsystem;

public class MovingShootCommand extends CommandBase {
    private ShootingSuperstructure shootingSuperstructure;
    private ShooterSubsystem shooterSubsystem;

    // private PIDController pidController = new PIDController(1, 0, 0.04, 0.02);

    private MovingAimStrategy aimStrategy;

    private boolean stopShooting;

    public MovingShootCommand(
            ShootingSuperstructure shootingSuperstructure, MovingAimStrategy movingAimStrategy, boolean stopShooting) {
        this.stopShooting = stopShooting;

        this.shootingSuperstructure = shootingSuperstructure;
        this.shooterSubsystem = shootingSuperstructure.getShooterSubsystem();

        addRequirements(shooterSubsystem);

        aimStrategy = movingAimStrategy;
    }

    public MovingShootCommand(ShootingSuperstructure shootingSuperstructure, MovingAimStrategy movingAimStrategy) {
        this(shootingSuperstructure, movingAimStrategy, true);
    }

    @Override
    public void initialize() {
        shootingSuperstructure.getLimelightSubsystem().bindUpdatable(aimStrategy);

        shootingSuperstructure.activateShootingPipeline();

        shooterSubsystem.setFarShot(() -> aimStrategy.calculateShotDistance());
    }

    @Override
    public void execute() {
        if (aimStrategy.isAimed() && shooterSubsystem.isShooterAtVelocity()) {
            shootingSuperstructure.getBalltrackSubsystem().shootMode();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (stopShooting) shootingSuperstructure.stopShooting();
        else shootingSuperstructure.stopShootingMaintainSpeed();

        shootingSuperstructure.storeCurrentShotDistance();

        shootingSuperstructure.getLimelightSubsystem().freeUpdatable();
    }
}
