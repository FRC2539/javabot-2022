package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Regressions;
import frc.robot.ShootingSuperstructure;
import frc.robot.subsystems.ShooterSubsystem;

public class ModifyShooterStateCommand extends InstantCommand {
    private ShooterSubsystem shooterSubsystem;
    private ShootingSuperstructure shootingSuperstructure;
    private double rearModificationFactor;
    private double frontModificationFactor;

    private NetworkTableEntry shooterMapEntry;

    public ModifyShooterStateCommand(
            ShooterSubsystem shooterSubsystem,
            ShootingSuperstructure shootingSuperstructure,
            double rearModificationFactor,
            double frontModificationFactor) {
        this.shooterSubsystem = shooterSubsystem;
        this.shootingSuperstructure = shootingSuperstructure;
        this.rearModificationFactor = rearModificationFactor;
        this.frontModificationFactor = frontModificationFactor;

        shooterMapEntry = NetworkTableInstance.getDefault().getTable("Commands").getEntry("shooterMap");
    }

    @Override
    public void initialize() {
        if (shootingSuperstructure.getStoredShotDistance().isPresent())
            shooterSubsystem.modifyShooterStateForDistance(
                    shootingSuperstructure.getStoredShotDistance().getAsDouble(), rearModificationFactor, frontModificationFactor);

        shooterMapEntry.setString(Regressions.stringifyMap(shooterSubsystem.getShooterMap()));
    }
}
