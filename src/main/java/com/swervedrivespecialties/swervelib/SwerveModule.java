package com.swervedrivespecialties.swervelib;

public interface SwerveModule 
{
    Double getDriveMotor();

    Object getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    void resetEncoder();
    
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);
}
