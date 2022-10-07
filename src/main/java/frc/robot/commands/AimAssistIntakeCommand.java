package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.controller.Axis;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.MachineLearningSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.LoggingCommand;

public class AimAssistIntakeCommand extends LoggingCommand {
    private MachineLearningSubsystem machineLearningSubsystem;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private BalltrackSubsystem balltrackSubsystem;
    private LightsSubsystem lightsSubsystem;

    private Axis forward;
    private Axis strafe;
    private Axis rotate;

    private double speedModifier;

    private static final double INTAKE_FACTOR = 0.85;

    private static final double INTAKE_DOWN_DISTANCE = 1.2;

    private boolean sawBallSoFar;

    private ProfiledPIDController rotationControl = new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(
                    SwerveConstants.maxAngularVelocity / 2, SwerveConstants.maxAngularVelocity / 4));

    public AimAssistIntakeCommand(
            MachineLearningSubsystem machineLearningSubsystem,
            SwerveDriveSubsystem swerveDriveSubsystem,
            BalltrackSubsystem balltrackSubsystem,
            LightsSubsystem lightsSubsystem,
            Axis forward,
            Axis strafe,
            Axis rotate) {
        this.machineLearningSubsystem = machineLearningSubsystem;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.balltrackSubsystem = balltrackSubsystem;
        this.lightsSubsystem = lightsSubsystem;

        addRequirements(machineLearningSubsystem, swerveDriveSubsystem, balltrackSubsystem);

        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;

        rotationControl.setGoal(0);
        rotationControl.setTolerance(Math.toRadians(5));
    }

    @Override
    public void initialize() {
        speedModifier = 1;
        balltrackSubsystem.stopIntakeMode();
        sawBallSoFar = false;
    }

    @Override
    public void execute() {
        if (machineLearningSubsystem.hasTarget()) {
            lightsSubsystem.solidPurple();
            sawBallSoFar = true;
        }
        double rotationAngle = machineLearningSubsystem.getHorizontalAngle();
        if (isDriverGoingForBall() && sawBallSoFar) {
            swerveDriveSubsystem.drive(
                    new ChassisSpeeds(
                            -getDriverValueTowardsBall() * Math.cos(rotationAngle) * speedModifier,
                            -getDriverValueTowardsBall() * Math.sin(rotationAngle) * speedModifier,
                            rotationControl.calculate(rotationAngle)),
                    false);
        } else {
            swerveDriveSubsystem.drive(new ChassisSpeeds(0, 0, rotate.get(true) * speedModifier), true);
            /*swerveDriveSubsystem.drive(
            new ChassisSpeeds(
                    forward.get(true) * speedModifier,
                    strafe.get(true) * speedModifier,
                    rotate.get(true) * speedModifier),
            true);*/
        }
        if (shouldIntake()) {
            balltrackSubsystem.intakeMode();
            speedModifier = INTAKE_FACTOR;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.drive(new ChassisSpeeds(), true);
        balltrackSubsystem.stopIntakeMode();
    }

    private double getVelocity() {
        ChassisSpeeds chassisSpeeds = swerveDriveSubsystem.getVelocity();
        return Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    private boolean isDriverGoingForBall() {
        // if (Math.abs(getDriverStrafeFromBall()) < 0.5) return false;

        if (Math.abs(machineLearningSubsystem.getHorizontalAngle()) < Math.toRadians(30)) {
            return true;
        } else {
            return false;
        }
    }

    private double getDriverValueTowardsBall() {
        double ballAngle = machineLearningSubsystem.getHorizontalAngle()
                + swerveDriveSubsystem.getGyroRotation2d().getRadians();
        double towardsBall = -forward.get(true) * Math.cos(ballAngle) + -strafe.get(true) * Math.sin(ballAngle);
        return towardsBall;
    }

    private double getDriverStrafeFromBall() {
        double ballAngle = machineLearningSubsystem.getHorizontalAngle()
                + swerveDriveSubsystem.getGyroRotation2d().getRadians();
        double strafeBall = forward.get(true) * Math.sin(ballAngle) + -strafe.get(true) * Math.cos(ballAngle);
        return strafeBall;
    }

    private boolean shouldIntake() {
        if (!machineLearningSubsystem.getBallDistance().isPresent()) return false;

        if (machineLearningSubsystem.getBallDistance().orElse(2) / getVelocity()
                < (INTAKE_DOWN_DISTANCE * INTAKE_FACTOR)) {
            return true;
        }

        if (machineLearningSubsystem.getBallDistance().orElse(2) < 0.6) return true;

        return false;
    }
}
