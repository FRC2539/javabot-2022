package com.swervedrivespecialties.swervelib;

public interface DriveController
 {
    Double getDriveMotor();
    void resetEncoder();
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
