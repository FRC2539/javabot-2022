package frc.robot;

import frc.robot.controller.ThrustmasterJoystick;
import frc.robot.controller.Axis;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final ThrustmasterJoystick leftDriveController = new ThrustmasterJoystick(Constants.LEFT_DRIVE_CONTROLLER);
    private final ThrustmasterJoystick rightDriveController = new ThrustmasterJoystick(Constants.RIGHT_DRIVE_CONTROLLER);
    
    private final SwerveDriveSubsystem drivetrainSubsystem = new SwerveDriveSubsystem();

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);

        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));

        configureControllerLayout();
    }

    private void configureControllerLayout() {

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
}
