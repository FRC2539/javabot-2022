package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LimelightPipeline;
import frc.robot.subsystems.ShooterSubsystem;

public class LimelightSpinUpCommand extends CommandBase {
    private LimelightSubsystem limelightSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public LimelightSpinUpCommand(LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        limelightSubsystem.setPipeline(LimelightPipeline.SHOOT);

        shooterSubsystem.setFarShot(limelightSubsystem.getMeasuredDistanceSupplier());
    }
}
