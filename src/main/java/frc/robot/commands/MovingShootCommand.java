package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShootingSuperstructure;
import frc.robot.strategies.MovingAimStrategy;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.function.DoubleSupplier;

public class MovingShootCommand extends CommandBase {
    private ShootingSuperstructure shootingSuperstructure;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private LightsSubsystem lightsSubsystem;
    private ShooterSubsystem shooterSubsystem;

    private DoubleSupplier forward;
    private DoubleSupplier strafe;

    private PIDController pidController = new PIDController(1, 0, 0.04, 0.02);

    private MovingAimStrategy aimStrategy;

    private boolean stopShooting;

    public MovingShootCommand(
            DoubleSupplier forward,
            DoubleSupplier strafe,
            ShootingSuperstructure shootingSuperstructure,
            LightsSubsystem lightsSubsystem,
            boolean stopShooting) {
        this.forward = forward;
        this.strafe = strafe;

        this.stopShooting = stopShooting;

        this.shootingSuperstructure = shootingSuperstructure;
        this.lightsSubsystem = lightsSubsystem;
        this.swerveDriveSubsystem = shootingSuperstructure.getSwerveDriveSubsystem();
        this.shooterSubsystem = shootingSuperstructure.getShooterSubsystem();

        addRequirements(swerveDriveSubsystem, lightsSubsystem, shooterSubsystem);

        pidController.enableContinuousInput(-Math.PI, Math.PI);

        aimStrategy = new MovingAimStrategy(shootingSuperstructure, pidController);
    }

    @Override
    public void initialize() {
        pidController.reset();

        shootingSuperstructure.activateShootingPipeline();

        aimStrategy.update();

        lightsSubsystem.setAimingMode(() -> aimStrategy.isAimed());

        shooterSubsystem.setFarShot(() -> aimStrategy.calculateShotDistance());
    }

    @Override
    public void execute() {
        aimStrategy.update();
        swerveDriveSubsystem.drive(
                new ChassisSpeeds(
                        forward.getAsDouble(), strafe.getAsDouble(), aimStrategy.calculateRotationalVelocity()),
                true);
    }

    @Override
    public void end(boolean interrupted) {
        if (stopShooting) shootingSuperstructure.stopShooting();
        else shootingSuperstructure.stopShootingMaintainSpeed();

        shootingSuperstructure.storeCurrentShotDistance();
    }
}
