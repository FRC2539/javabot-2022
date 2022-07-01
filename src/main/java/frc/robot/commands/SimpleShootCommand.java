package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShootingSuperstructure;

public class SimpleShootCommand extends CommandBase {
    private ShootingSuperstructure shootingSuperstructure;
    private Runnable setShooterStateRunnable;

    public SimpleShootCommand(ShootingSuperstructure shootingSuperstructure, Runnable setShooterStateRunnable) {
        this.shootingSuperstructure = shootingSuperstructure;
        this.setShooterStateRunnable = setShooterStateRunnable;

        addRequirements(shootingSuperstructure.getShooterSubsystem());
    }

    @Override
    public void initialize() {
        setShooterStateRunnable.run();
    }

    @Override
    public void execute() {
        shootingSuperstructure.shootWithoutLimelight();
    }

    @Override
    public void end(boolean interrupted) {
        shootingSuperstructure.stopShooting();
    }
}
