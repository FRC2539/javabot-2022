package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.BalltrackSubsystem.BalltrackMode;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PrepareToShootCommand extends CommandBase {
    private BalltrackSubsystem balltrackSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private LimelightSubsystem limelightSubsystem;

    public PrepareToShootCommand(
            BalltrackSubsystem balltrackSubsystem,
            ShooterSubsystem shooterSubsystem,
            LimelightSubsystem limelightSubsystem) {
        this.balltrackSubsystem = balltrackSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.limelightSubsystem = limelightSubsystem;

        addRequirements(balltrackSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setFarShot(limelightSubsystem.getDistanceToTarget().orElse(0));

        balltrackSubsystem.setBalltrackMode(BalltrackMode.PREPARE);
    }

    @Override
    public void end(boolean interrupted) {
        balltrackSubsystem.setBalltrackMode(BalltrackMode.DISABLED);

        shooterSubsystem.stopShooter();
    }
}
