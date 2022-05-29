package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightsSubsystem;

public class SeizureModeCommand extends CommandBase {
    private LightsSubsystem lightsSubsystem;
    
    private int currentColor;

    public SeizureModeCommand(LightsSubsystem lightsSubsystem) {
        this.lightsSubsystem = lightsSubsystem;

        addRequirements(lightsSubsystem);
    }

    @Override
    public void initialize() {
        currentColor = 0;
    }

    @Override
    public void execute() {
        switch (currentColor) {
            case 0:
                lightsSubsystem.solidRed();
                break;
            case 1:
                lightsSubsystem.solidGreen();
                break;
            case 2:
                lightsSubsystem.solidBlue();
                break;
        }

        currentColor = (currentColor + 1) % 3;
    }

    @Override
    public void end(boolean interrupted) {
        lightsSubsystem.off();
    }
}
