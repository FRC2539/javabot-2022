package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public interface SwerveModule {
    Double getDriveMotor();

    Object getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    void resetEncoder();

    double getDriveVelocity();

    double getSteerAngle();

    Double getDriveTemperature();

    Double getSteerTemperature();

    WPI_TalonFX getRawDriveMotor();

    WPI_TalonFX getRawSteerMotor();

    void set(double driveVoltage, double steerAngle);
}
