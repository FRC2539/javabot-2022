package frc.robot.common.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;

public class ShooterState implements Interpolatable<ShooterState> {
    public double rearShooterRPM;
    public double frontShooterRPM;
    public ShooterAngle angle;

    public ShooterState(double rearShooterRPM, double frontShooterRPM, ShooterAngle angle) {
        this.rearShooterRPM = rearShooterRPM;
        this.frontShooterRPM = frontShooterRPM;
        this.angle = angle;
    }

    public ShooterState(double rearShooterRPM, double frontShooterRPM) {
        this(rearShooterRPM, frontShooterRPM, ShooterAngle.FAR_SHOT);
    }

    public ShooterState() {
        this(0, 0, ShooterAngle.FAR_SHOT);
    }

    public ShooterState interpolate(ShooterState otherState, double t) {
        // Linearly interpolate between the RPMs at t (0 - 1)
        return new ShooterState(
                MathUtil.interpolate(this.rearShooterRPM, otherState.rearShooterRPM, t),
                MathUtil.interpolate(this.frontShooterRPM, otherState.frontShooterRPM, t),
                angle);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }

        if (obj.getClass() != this.getClass()) {
            return false;
        }

        // Verify that the shooter rpms are equal
        final ShooterState other = (ShooterState) obj;
        if (this.rearShooterRPM != other.rearShooterRPM || this.frontShooterRPM != other.frontShooterRPM) {
            return false;
        }

        return true;
    }
}
