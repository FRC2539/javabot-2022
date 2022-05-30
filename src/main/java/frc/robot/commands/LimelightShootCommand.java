package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LimelightPipeline;

public class LimelightShootCommand extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private BalltrackSubsystem balltrackSubsystem;
    private LimelightSubsystem limelightSubsystem;

    public LimelightShootCommand(ShooterSubsystem shooterSubsystem, BalltrackSubsystem balltrackSubsystem, LimelightSubsystem limelightSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.balltrackSubsystem = balltrackSubsystem;
        this.limelightSubsystem = limelightSubsystem;

        addRequirements(shooterSubsystem);
    } 

    @Override
    public void initialize() {
        limelightSubsystem.setPipeline(LimelightPipeline.SHOOT);
    }

    @Override
    public void execute() {
        shooterSubsystem.setFarShot(limelightSubsystem.getDistanceToTarget().orElse(1));

        if (limelightSubsystem.isAimed() && shooterSubsystem.isShooterAtVelocity())
            balltrackSubsystem.shootMode();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        balltrackSubsystem.stopShootMode();
    }
}