package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShootingSuperstructure;

public class CustomShootCommand extends CommandBase {
    private ShootingSuperstructure shootingSuperstructure;

    public CustomShootCommand(ShootingSuperstructure shootingSuperstructure) {
        this.shootingSuperstructure = shootingSuperstructure;
    }

    @Override
    public void initialize() {
        shootingSuperstructure.spinupShooterWithCustomSpeeds();
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
