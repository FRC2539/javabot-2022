package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Regressions;
import frc.robot.subsystems.ShooterSubsystem;

public class SendShooterMapCommand extends InstantCommand {
    private ShooterSubsystem shooterSubsystem;

    private NetworkTableEntry shooterMapEntry;

    public SendShooterMapCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        shooterMapEntry = NetworkTableInstance.getDefault().getTable("Commands").getEntry("shooterMap");
    }

    @Override
    public void initialize() {
        shooterMapEntry.setString(Regressions.stringifyMap(shooterSubsystem.getShooterMap()));
    }
}
