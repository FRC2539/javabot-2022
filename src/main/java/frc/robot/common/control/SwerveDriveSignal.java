package frc.robot.common.control;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDriveSignal extends ChassisSpeeds {
    private boolean isFieldOriented;

    public SwerveDriveSignal(
            double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond, boolean isFieldOriented) {
        super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

        this.isFieldOriented = isFieldOriented;
    }

    public SwerveDriveSignal(ChassisSpeeds velocity, boolean isFieldOriented) {
        this(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond, isFieldOriented);
    }

    public SwerveDriveSignal() {
        super();

        this.isFieldOriented = false;
    }

    public boolean isFieldOriented() {
        return this.isFieldOriented;
    }
}
