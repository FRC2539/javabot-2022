package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.LightsSubsystem;

public class IntakeCommand extends CommandBase {
    private BalltrackSubsystem balltrackSubsystem;
    private LightsSubsystem lightsSubsystem;

    public IntakeCommand(BalltrackSubsystem balltrackSubsystem, LightsSubsystem lightsSubsystem) {
        this.balltrackSubsystem = balltrackSubsystem;
        this.lightsSubsystem = lightsSubsystem;

        // We don't require the lights subsystem to avoid interrupting any aiming that
        // is occurring simulatneously
        addRequirements(balltrackSubsystem);
    }

    @Override
    public void initialize() {
        balltrackSubsystem.intakeMode();

        lightsSubsystem.setBalltrackMode(() -> balltrackSubsystem.isBalltrackFull());
    }

    @Override
    public void end(boolean interrupted) {
        balltrackSubsystem.stopIntakeMode();

        lightsSubsystem.setDefaultMode();
    }
}
