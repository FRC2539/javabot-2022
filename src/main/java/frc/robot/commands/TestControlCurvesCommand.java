package frc.robot.commands;

import com.team2539.cougarlib.controller.Axis;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TestControlCurvesCommand extends CommandBase {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private Axis forward;
    private Axis strafe;
    private Axis rotation;
    private Axis twist;

    public TestControlCurvesCommand(
            SwerveDriveSubsystem swerveDriveSubsystem, Axis forward, Axis strafe, Axis rotation, Axis twist) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        this.twist = twist;

        this.swerveDriveSubsystem = swerveDriveSubsystem;

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void execute() {
        double exponent = -twist.get(false, true) + 1.0;

        double forwardValue = useExponent(forward.get(false, true), exponent) * SwerveDriveSubsystem.MAX_VELOCITY;
        double strafeValue = useExponent(strafe.get(false, true), exponent) * SwerveDriveSubsystem.MAX_VELOCITY;
        double rotationValue =
                useExponent(rotation.get(false, true), exponent) * SwerveDriveSubsystem.MAX_ANGULAR_VELOCITY;

        swerveDriveSubsystem.drive(new ChassisSpeeds(forwardValue, strafeValue, rotationValue), true);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.drive(new ChassisSpeeds(), true);
    }

    private double useExponent(double value, double exponent) {
        return Math.copySign(Math.pow(Math.abs(value), exponent), value);
    }
}
