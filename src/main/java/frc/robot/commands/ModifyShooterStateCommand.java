package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShootingSuperstructure;
import frc.robot.subsystems.ShooterSubsystem;

public class ModifyShooterStateCommand extends InstantCommand {
    private ShooterSubsystem shooterSubsystem;
    private ShootingSuperstructure shootingSuperstructure;
    private double modificationFactor;

    public ModifyShooterStateCommand(
            ShooterSubsystem shooterSubsystem,
            ShootingSuperstructure shootingSuperstructure,
            double modificationFactor) {
        this.shooterSubsystem = shooterSubsystem;
        this.shootingSuperstructure = shootingSuperstructure;
        this.modificationFactor = modificationFactor;
    }

    @Override
    public void initialize() {
        if (shootingSuperstructure.getStoredShotDistance().isPresent())
            shooterSubsystem.modifyShooterStateForDistance(
                    shootingSuperstructure.getStoredShotDistance().getAsDouble(), modificationFactor);
    }
}
