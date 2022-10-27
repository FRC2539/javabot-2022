package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.swerve.CTREModuleState;
import frc.lib.swerve.Conversions;
import frc.lib.swerve.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;
    private boolean inverted;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        inverted = moduleConstants.inverted;

        // Angle Encoder Config
        angleEncoder = moduleConstants.canivoreName.isEmpty()
                ? new CANCoder(moduleConstants.cancoderID)
                : new CANCoder(moduleConstants.cancoderID, moduleConstants.canivoreName.get());
        configAngleEncoder();

        // Angle Motor Config
        angleMotor = moduleConstants.canivoreName.isEmpty()
                ? new TalonFX(moduleConstants.angleMotorID)
                : new TalonFX(moduleConstants.angleMotorID, moduleConstants.canivoreName.get());
        configAngleMotor();

        // Drive Motor Config
        driveMotor = moduleConstants.canivoreName.isEmpty()
                ? new TalonFX(moduleConstants.driveMotorID)
                : new TalonFX(moduleConstants.driveMotorID, moduleConstants.canivoreName.get());
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(
                    desiredState.speedMetersPerSecond,
                    Constants.SwerveConstants.wheelCircumference,
                    Constants.SwerveConstants.driveGearRatio);
            driveMotor.set(
                    ControlMode.Velocity,
                    velocity,
                    DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees();

        angleMotor.set(
                ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.SwerveConstants.angleGearRatio));
        lastAngle = angle;
    }

    /**
     * Offset the angle motor encoder with the measurement from the CANCoder.
     * (Orient the swerve modules)
     */
    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(
                getCanCoder().getDegrees() - angleOffset, Constants.SwerveConstants.angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        angleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        angleMotor.setNeutralMode(Constants.SwerveConstants.angleNeutralMode);
        angleMotor.enableVoltageCompensation(true);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setNeutralMode(Constants.SwerveConstants.driveNeutralMode);
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.setInverted(inverted);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(
                driveMotor.getSelectedSensorVelocity(),
                Constants.SwerveConstants.wheelCircumference,
                Constants.SwerveConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(
                angleMotor.getSelectedSensorPosition(), Constants.SwerveConstants.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    /**
     * @return The drive motor temperature in Celsius
     */
    public double getDriveTemperature() {
        return driveMotor.getTemperature();
    }

    /**
     * @return The steer motor temperature in Celsius
     */
    public double getSteerTemperature() {
        return angleMotor.getTemperature();
    }
}
