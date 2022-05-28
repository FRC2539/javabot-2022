package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;

public class ClimberSubsystem extends NetworkTablesSubsystem {
    private WPI_TalonFX climberMotor = new WPI_TalonFX(Constants.CLIMBER_MOTOR_PORT, Constants.CANIVORE_NAME);

    private final boolean USE_LIMITS = true;

    private final double UPPER_LIMIT = 224000;
    private final double LOWER_LIMIT = 8200;
    private final double LOWER_LIMIT_RAMP_END = 27500;

    private final double CLIMBER_MOTOR_SPEED = 1;

    public ClimberSubsystem() {
        super("Climber");

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.forwardSoftLimitEnable = USE_LIMITS;
        configuration.forwardSoftLimitThreshold = UPPER_LIMIT;
        configuration.reverseSoftLimitEnable = USE_LIMITS;
        configuration.reverseSoftLimitThreshold = LOWER_LIMIT;

        climberMotor.configAllSettings(configuration);
        climberMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void raiseClimber() {
        climberMotor.set(ControlMode.PercentOutput, calculateRampSpeed());
    }

    public void lowerClimber() {
        climberMotor.set(ControlMode.PercentOutput, CLIMBER_MOTOR_SPEED);
    }

    private double calculateRampSpeed() {
        double speed = Math.min(
            (getPosition() - LOWER_LIMIT)
            / (LOWER_LIMIT_RAMP_END - LOWER_LIMIT),
            CLIMBER_MOTOR_SPEED
        );

        return Math.max(0.5, speed);
    }

    public double getPosition() {
        return climberMotor.getSelectedSensorPosition();
    }

    public void enableLimits() {
        climberMotor.overrideSoftLimitsEnable(true);
    }

    public void disableLimits() {
        climberMotor.overrideSoftLimitsEnable(false);
    }
}
