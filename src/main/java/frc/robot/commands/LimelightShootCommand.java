package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LimelightPipeline;
import frc.robot.subsystems.ShooterSubsystem;

public class LimelightShootCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private BalltrackSubsystem balltrackSubsystem;
    private LimelightSubsystem limelightSubsystem;

    private boolean stopShooting;

    public LimelightShootCommand(
            ShooterSubsystem shooterSubsystem,
            BalltrackSubsystem balltrackSubsystem,
            LimelightSubsystem limelightSubsystem,
            boolean stopShooting) {
        this.shooterSubsystem = shooterSubsystem;
        this.balltrackSubsystem = balltrackSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.stopShooting = stopShooting;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        limelightSubsystem.setPipeline(LimelightPipeline.SHOOT);
    }

    @Override
    public void execute() {
        shooterSubsystem.setFarShot(limelightSubsystem.getDistanceToTarget().orElse(0));

        if (limelightSubsystem.isAimed() && shooterSubsystem.isShooterAtVelocity()) balltrackSubsystem.shootMode();
    }

    @Override
    public void end(boolean interrupted) {
        if (stopShooting) shooterSubsystem.stopShooter();
        balltrackSubsystem.stopShootMode();
    }
}
