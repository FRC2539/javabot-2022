package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShootingSuperstructure;

public class PrepareToShootCommand extends CommandBase {
    private ShootingSuperstructure shootingSuperstructure;

    public PrepareToShootCommand(ShootingSuperstructure shootingSuperstructure) {
        this.shootingSuperstructure = shootingSuperstructure;

        addRequirements(shootingSuperstructure.getShooterSubsystem(), shootingSuperstructure.getBalltrackSubsystem());
    }

    @Override
    public void initialize() {
        shootingSuperstructure.spinupShooterWithLimelight();
        shootingSuperstructure.prepareBallsToShoot();
    }

    @Override
    public void end(boolean interrupted) {
        shootingSuperstructure.stopShooting();
    }
}
