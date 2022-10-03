package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShootingSuperstructure;
import frc.robot.strategies.MovingAimStrategy;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.function.DoubleSupplier;

public class MovingShootDriveCommand extends CommandBase {
    private ShootingSuperstructure shootingSuperstructure;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private LightsSubsystem lightsSubsystem;

    private DoubleSupplier forward;
    private DoubleSupplier strafe;

    private MovingAimStrategy aimStrategy;

    private final double SPEED_CAP = 1.5;

    public MovingShootDriveCommand(
            DoubleSupplier forward,
            DoubleSupplier strafe,
            ShootingSuperstructure shootingSuperstructure,
            LightsSubsystem lightsSubsystem,
            MovingAimStrategy movingAimStrategy) {
        this.forward = forward;
        this.strafe = strafe;

        this.shootingSuperstructure = shootingSuperstructure;
        this.lightsSubsystem = lightsSubsystem;
        this.swerveDriveSubsystem = shootingSuperstructure.getSwerveDriveSubsystem();

        addRequirements(swerveDriveSubsystem, lightsSubsystem);

        aimStrategy = movingAimStrategy;
    }

    @Override
    public void initialize() {
        shootingSuperstructure.getLimelightSubsystem().bindUpdatable(aimStrategy);

        aimStrategy.resetPIDController();

        shootingSuperstructure.activateShootingPipeline();

        lightsSubsystem.setAimingMode(() -> aimStrategy.isAimed());
    }

    @Override
    public void execute() {
        // of all the moving shoot stuff, this should be the only one to call the calculateRotationalVelocity method, so
        // it isnt overclocked
        swerveDriveSubsystem.drive(
                new ChassisSpeeds(
                        Math.min(SPEED_CAP, Math.max(forward.getAsDouble(), -SPEED_CAP)),
                        Math.min(SPEED_CAP, Math.max(strafe.getAsDouble(), -SPEED_CAP)),
                        aimStrategy.calculateRotationalVelocity()),
                true);
    }

    @Override
    public void end(boolean interrupted) {
        shootingSuperstructure.getLimelightSubsystem().freeUpdatable();
    }
}
