package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.common.controller.Axis;
import frc.robot.common.controller.ThrustmasterJoystick;
import frc.robot.common.controller.LogitechController;
import frc.robot.subsystems.*;
import frc.robot.util.AutonomousManager;

public class RobotContainer {
    private final ThrustmasterJoystick leftDriveController = new ThrustmasterJoystick(Constants.LEFT_DRIVE_CONTROLLER);
    private final ThrustmasterJoystick rightDriveController = new ThrustmasterJoystick(Constants.RIGHT_DRIVE_CONTROLLER);
    private final LogitechController operatorController = new LogitechController(Constants.OPERATOR_CONTROLLER);
    
    private final SwerveDriveSubsystem drivetrainSubsystem = new SwerveDriveSubsystem();

    private AutonomousManager autonomousManager;

    public RobotContainer() {
        autonomousManager = new AutonomousManager();

        CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));

        configureControllerLayout();
    }

    private void configureControllerLayout() {
        leftDriveController.getLeftTopLeft().whenPressed(() -> drivetrainSubsystem.resetGyroAngle());
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
}
