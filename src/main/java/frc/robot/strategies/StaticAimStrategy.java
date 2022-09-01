package frc.robot.strategies;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.ShootingSuperstructure;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class StaticAimStrategy implements LimelightAimStrategy {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private PIDController pidController;

    public StaticAimStrategy(ShootingSuperstructure shootingSuperstructure, PIDController pidController) {
        this.pidController = pidController;
        this.limelightSubsystem = shootingSuperstructure.getLimelightSubsystem();
        this.swerveDriveSubsystem = shootingSuperstructure.getSwerveDriveSubsystem();
    }

    public double calculateRotationalVelocity() {
        double rotationOutput = 0;

        if (limelightSubsystem.hasTarget()) {
            double currentAngle = swerveDriveSubsystem.getGyroRotation2d().getRadians();
            double targetAngle =
                    MathUtil.angleModulus(currentAngle - Math.toRadians(limelightSubsystem.getHorizontalAngle()));

            pidController.setSetpoint(targetAngle);

            rotationOutput = pidController.calculate(currentAngle);
        }

        return -1 * rotationOutput * SwerveConstants.maxAngularVelocity;
    }

    public boolean isAimed() {
        return limelightSubsystem.isAimed();
    }
}
