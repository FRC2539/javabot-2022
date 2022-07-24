package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShootingSuperstructure;

public class LimelightSpinUpCommand extends CommandBase {
    private ShootingSuperstructure shootingSuperstructure;

    public LimelightSpinUpCommand(ShootingSuperstructure shootingSuperstructure) {
        this.shootingSuperstructure = shootingSuperstructure;
    }

    @Override
    public void initialize() {
        shootingSuperstructure.spinupShooterWithLimelight();
    }
}
