package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.controller.Axis;

public class DriveCommand extends CommandBase {
    private SwerveDriveSubsystem drivetrainSubsystem;
    private Axis forward;
    private Axis strafe;
    private Axis rotation;

    public DriveCommand(SwerveDriveSubsystem drivetrain, Axis forward, Axis strafe, Axis rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        drivetrainSubsystem = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrainSubsystem.drive(new ChassisSpeeds(forward.get(false), strafe.get(false), rotation.get(false)));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds());
    }

}