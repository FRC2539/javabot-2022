package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class LowerClimberCommand extends CommandBase {
    private ClimberSubsystem climberSubsystem;

    public LowerClimberCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        climberSubsystem.lowerClimber();
    }

    @Override
    public void execute() {
        // Because the command is on a while held, finishing it forcefully would not stop
        // the climber.
        if (climberSubsystem.isClimberOnBar()) {
            climberSubsystem.stopClimber();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stopClimber();
    }
}
