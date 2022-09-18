package frc.robot.commands;

import com.team2539.cougarlib.control.InterpolatingMap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Regressions;
import frc.robot.ShootingSuperstructure;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShooterState;

public class ModifyShooterStateCommand extends InstantCommand {
    private ShooterSubsystem shooterSubsystem;
    private ShootingSuperstructure shootingSuperstructure;
    private double rearModificationFactor;
    private double frontModificationFactor;

    private static NetworkTableEntry shooterMapEntry =
            NetworkTableInstance.getDefault().getTable("Commands").getEntry("shooterMap");

    public ModifyShooterStateCommand(
            ShooterSubsystem shooterSubsystem,
            ShootingSuperstructure shootingSuperstructure,
            double rearModificationFactor,
            double frontModificationFactor) {
        this.shooterSubsystem = shooterSubsystem;
        this.shootingSuperstructure = shootingSuperstructure;
        this.rearModificationFactor = rearModificationFactor;
        this.frontModificationFactor = frontModificationFactor;
    }

    @Override
    public void initialize() {
        shooterSubsystem.modifyShooterStateForDistance(
                shootingSuperstructure.getStoredShotDistance().orElse(0),
                rearModificationFactor,
                frontModificationFactor);

        sendShooterMapToNetworkTables(shooterSubsystem.getShooterMap());
    }

    public static void sendShooterMapToNetworkTables(InterpolatingMap<ShooterState> shooterMap) {
        shooterMapEntry.setString(Regressions.stringifyMap(shooterMap));
    }
}
