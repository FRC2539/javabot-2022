package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightsSubsystem;

public class DefaultLightsCommand extends CommandBase {
    private LightsSubsystem lightsSubsystem;

    public DefaultLightsCommand(LightsSubsystem lightsSubsystem) {
        this.lightsSubsystem = lightsSubsystem;

        addRequirements(lightsSubsystem);
    }

    @Override
    public void initialize() {
        lightsSubsystem.showTeamColor();
    }
}
