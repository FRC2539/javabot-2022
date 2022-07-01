package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShootingSuperstructure;

public class LimelightShootCommand extends CommandBase {
    private ShootingSuperstructure shootingSuperstructure;

    private boolean stopShooting;

    public LimelightShootCommand(
            ShootingSuperstructure shootingSuperstructure,
            boolean stopShooting) {
        this.shootingSuperstructure = shootingSuperstructure;
        this.stopShooting = stopShooting;

        addRequirements(shootingSuperstructure.getShooterSubsystem());
    }

    public LimelightShootCommand(ShootingSuperstructure shootingSuperstructure) {
        this(shootingSuperstructure, true);
    }

    @Override
    public void initialize() {
        shootingSuperstructure.spinupShooterWithLimelight();
    }

    @Override
    public void execute() {
        shootingSuperstructure.shootWithLimelight();
    }

    @Override
    public void end(boolean interrupted) {
        if (stopShooting) shootingSuperstructure.stopShooting();
        else shootingSuperstructure.stopShootingMaintainSpeed();
    }
}
