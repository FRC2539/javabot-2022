package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public interface SteerController {
    Object getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    Double getMotorTemperature();

    WPI_TalonFX getRawSteerMotor();
}
