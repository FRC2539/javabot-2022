package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.controller.Axis;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LimelightDriveCommand extends CommandBase {
    private SwerveDriveSubsystem drivetrainSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private LightsSubsystem lightsSubsystem;

    private Axis forward;
    private Axis strafe;

    private PIDController pidController = new PIDController(0.1, 0, 0, 0.02);

    public LimelightDriveCommand(SwerveDriveSubsystem drivetrain, Axis forward, Axis strafe, LimelightSubsystem limelightSubsystem, LightsSubsystem lightsSubsystem) {
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

        drivetrainSubsystem.drive(new ChassisSpeeds(forward.get(true), strafe.get(true), rotationOutput), true);

        if (limelightSubsystem.isAimed())
            lightsSubsystem.solidGreen();
        else
            lightsSubsystem.showTeamColor();
    }
}