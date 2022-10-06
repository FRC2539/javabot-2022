package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.controller.Axis;
import frc.robot.subsystems.BalltrackSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveCommand extends CommandBase {
    private SwerveDriveSubsystem drivetrainSubsystem;
    private BalltrackSubsystem balltrackSubsystem;
    private Axis forward;
    private Axis strafe;
    private Axis rotation;

    private static final double INTAKE_FACTOR = 0.75; // 0.85

    public DriveCommand(
            SwerveDriveSubsystem drivetrain,
            Axis forward,
            Axis strafe,
            Axis rotation,
            BalltrackSubsystem balltrackSubsystem) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        drivetrainSubsystem = drivetrain;
        this.balltrackSubsystem = balltrackSubsystem;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (balltrackSubsystem.isIntaking()) {
            drivetrainSubsystem.drive(
                    new ChassisSpeeds(
                            forward.get(true) * INTAKE_FACTOR,
                            strafe.get(true) * INTAKE_FACTOR,
                            rotation.get(true) * INTAKE_FACTOR),
                    true);
        } else {
            drivetrainSubsystem.drive(new ChassisSpeeds(forward.get(true), strafe.get(true), rotation.get(true)), true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(), true);
    }
}
