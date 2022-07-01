package frc.robot.strategies;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.ShootingSuperstructure;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class MovingAimStrategy implements LimelightAimStrategy {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private PIDController pidController;

    public MovingAimStrategy(ShootingSuperstructure shootingSuperstructure, PIDController pidController) {
        this.pidController = pidController;
        this.limelightSubsystem = shootingSuperstructure.getLimelightSubsystem();
        this.swerveDriveSubsystem = shootingSuperstructure.getSwerveDriveSubsystem();
    }

    public double calculateRotationalVelocity() {
        double rotationOutput = 0;

        if (limelightSubsystem.hasTarget()) {
            // Fetch the current angle of the robot
            double currentAngle = swerveDriveSubsystem.getGyroRotation2d().getRadians();

            // Store the predicted angle (calculated by the limelight subsystem), and if that value is empty, fetch the
            // current horizontal angle instead.
            double predictedAngle = Math.toRadians(limelightSubsystem
                    .getPredictedHorizontalAngle()
                    .orElseGet(() -> limelightSubsystem.getHorizontalAngle()));

            double targetAngle = MathUtil.angleModulus(currentAngle + predictedAngle);

            pidController.setSetpoint(targetAngle);

            rotationOutput = pidController.calculate(currentAngle);
        }

        return rotationOutput * SwerveDriveSubsystem.MAX_ANGULAR_VELOCITY;
    }
}
