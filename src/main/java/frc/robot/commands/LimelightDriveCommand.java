package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ShootingSuperstructure;
import frc.robot.strategies.LimelightAimStrategy;
import frc.robot.strategies.StaticAimStrategy;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.function.DoubleSupplier;

public class LimelightDriveCommand extends CommandBase {
    private ShootingSuperstructure shootingSuperstructure;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private LightsSubsystem lightsSubsystem;

    private DoubleSupplier forward;
    private DoubleSupplier strafe;

    private static boolean ROTATE_AROUND_TARGET = false;

    private PIDController pidController = new PIDController(1, 0, 0.04, 0.02);

    private LimelightAimStrategy aimStrategy;

    public LimelightDriveCommand(
            DoubleSupplier forward,
            DoubleSupplier strafe,
            ShootingSuperstructure shootingSuperstructure,
            LightsSubsystem lightsSubsystem) {
        this.forward = forward;
        this.strafe = strafe;

        this.shootingSuperstructure = shootingSuperstructure;
        this.lightsSubsystem = lightsSubsystem;
        this.swerveDriveSubsystem = shootingSuperstructure.getSwerveDriveSubsystem();

        addRequirements(swerveDriveSubsystem, lightsSubsystem);

        pidController.enableContinuousInput(-Math.PI, Math.PI);

        aimStrategy = new StaticAimStrategy(shootingSuperstructure, pidController);
    }

    @Override
    public void initialize() {
        pidController.reset();

        shootingSuperstructure.activateShootingPipeline();

        if (ROTATE_AROUND_TARGET) shootingSuperstructure.rotateAroundTarget();
    }

    @Override
    public void execute() {
        swerveDriveSubsystem.drive(
                new ChassisSpeeds(
                        forward.getAsDouble(), strafe.getAsDouble(), aimStrategy.calculateRotationalVelocity()),
                true);

        if (aimStrategy.isAimed()) lightsSubsystem.solidGreen();
        else lightsSubsystem.showTeamColor();
    }

    @Override
    public void end(boolean interrupted) {
        if (ROTATE_AROUND_TARGET) shootingSuperstructure.stopRotatingAroundTarget();
    }
}
