package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class EnableTemperatureLogging extends InstantCommand {
    private SwerveDriveSubsystem swerveDriveSubsystem;

    public EnableTemperatureLogging(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.enableLoggingTemperatures();
    }
}
