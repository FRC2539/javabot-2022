package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltrackSubsystem;

public class IntakeCommand extends CommandBase {
    private BalltrackSubsystem balltrackSubsystem;
    
    public IntakeCommand(BalltrackSubsystem balltrackSubsystem) {
        this.balltrackSubsystem = balltrackSubsystem;

        addRequirements(balltrackSubsystem);
    } 

    @Override
    public void initialize() {
        balltrackSubsystem.intakeMode();
    }

    @Override
    public void end(boolean interrupted) {
        balltrackSubsystem.stopIntakeMode();
    }
}
