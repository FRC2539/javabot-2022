package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.controller.Axis;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem.LimelightPipeline;

public class LimelightDriveCommand extends CommandBase {
    private SwerveDriveSubsystem drivetrainSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private LightsSubsystem lightsSubsystem;

    private DoubleSupplier forward;
    private DoubleSupplier strafe;

    private PIDController pidController = new PIDController(0.1, 0, 0, 0.02);

    public LimelightDriveCommand(SwerveDriveSubsystem drivetrain, DoubleSupplier forward, DoubleSupplier strafe, LimelightSubsystem limelightSubsystem, LightsSubsystem lightsSubsystem) {
        this.forward = forward;
        this.strafe = strafe;

        drivetrainSubsystem = drivetrain;
        this.limelightSubsystem = limelightSubsystem;
        this.lightsSubsystem = lightsSubsystem;

        addRequirements(drivetrain, lightsSubsystem);

        pidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        pidController.reset();

        limelightSubsystem.setPipeline(LimelightPipeline.SHOOT);
    }

    @Override
    public void execute() {
        double rotationOutput = 0;

        if (limelightSubsystem.hasTarget()) {
            double currentAngle = drivetrainSubsystem.getGyroRotation2d().getRadians();
            double targetAngle = MathUtil.angleModulus(currentAngle + Math.toRadians(limelightSubsystem.getHorizontalAngle()));

            pidController.setSetpoint(targetAngle);

            rotationOutput = pidController.calculate(currentAngle);
        }

        drivetrainSubsystem.drive(new ChassisSpeeds(forward.getAsDouble(), strafe.getAsDouble(), rotationOutput), true);

        if (limelightSubsystem.isAimed())
            lightsSubsystem.solidGreen();
        else
            lightsSubsystem.showTeamColor();
    }
}