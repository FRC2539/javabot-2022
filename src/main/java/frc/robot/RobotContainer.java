package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.common.controller.Axis;
import frc.robot.common.controller.ThrustmasterJoystick;
import frc.robot.common.controller.LogitechController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.util.AutonomousManager;

public class RobotContainer {
    private final ThrustmasterJoystick leftDriveController = new ThrustmasterJoystick(Constants.LEFT_DRIVE_CONTROLLER);
    private final ThrustmasterJoystick rightDriveController = new ThrustmasterJoystick(Constants.RIGHT_DRIVE_CONTROLLER);
    private final LogitechController operatorController = new LogitechController(Constants.OPERATOR_CONTROLLER);
    
    private final SwerveDriveSubsystem drivetrainSubsystem = new SwerveDriveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    private AutonomousManager autonomousManager;

    public RobotContainer() {
        autonomousManager = new AutonomousManager();

        CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));

        CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);

        configureControllerLayout();
    }

    private void configureControllerLayout() {
        leftDriveController.getLeftTopLeft().whenPressed(() -> drivetrainSubsystem.resetGyroAngle());

        leftDriveController.getLeftThumb().whenPressed(() -> shooterSubsystem.setShooterAngle(ShooterAngle.CLOSE_SHOT), shooterSubsystem);
        leftDriveController.getRightThumb().whenPressed(() -> shooterSubsystem.setShooterAngle(ShooterAngle.FAR_SHOT), shooterSubsystem);
    }

    public Command getAutonomousCommand() {
        return autonomousManager.getAutonomousCommand(this);
    }

    private Axis getDriveForwardAxis() {
        return leftDriveController.getXAxis();
    }

    private Axis getDriveStrafeAxis() {
        return leftDriveController.getYAxis();
    }

    private Axis getDriveRotationAxis() {
        return rightDriveController.getXAxis();
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return drivetrainSubsystem;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }
}
