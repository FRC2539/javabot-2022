package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.swerve.CTREModuleState;
import frc.lib.swerve.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;
    private boolean inverted;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        inverted = moduleConstants.inverted;

        /* Angle Encoder Config */
        angleEncoder = moduleConstants.canivoreName.isEmpty()
                ? new CANCoder(moduleConstants.cancoderID)
                : new CANCoder(moduleConstants.cancoderID, moduleConstants.canivoreName.get());
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = moduleConstants.canivoreName.isEmpty()
                ? new TalonFX(moduleConstants.angleMotorID)
                : new TalonFX(moduleConstants.angleMotorID, moduleConstants.canivoreName.get());
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = moduleConstants.canivoreName.isEmpty()
                ? new TalonFX(moduleConstants.driveMotorID)
                : new TalonFX(moduleConstants.driveMotorID, moduleConstants.canivoreName.get());
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(
                desiredState,
                getState()
                        .angle); // Custom optimize command, since default WPILib optimize assumes continuous controller
        // which CTRE is not

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(
                    desiredState.speedMetersPerSecond,
                    Constants.SwerveConstants.wheelCircumference,
                    Constants.SwerveConstants.driveGearRatio);
            mDriveMotor.set(
                    ControlMode.Velocity,
                    velocity,
                    DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle
                        .getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.set(
                ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.SwerveConstants.angleGearRatio));
        lastAngle = angle;
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(
                getCanCoder().getDegrees() - angleOffset, Constants.SwerveConstants.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.SwerveConstants.angleNeutralMode);
        mAngleMotor.enableVoltageCompensation(true);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setNeutralMode(Constants.SwerveConstants.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
        mDriveMotor.enableVoltageCompensation(true);
        mDriveMotor.setInverted(inverted);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(
                mDriveMotor.getSelectedSensorVelocity(),
                Constants.SwerveConstants.wheelCircumference,
                Constants.SwerveConstants.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(
                mAngleMotor.getSelectedSensorPosition(), Constants.SwerveConstants.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    public double getDriveTemperature() {
        return mDriveMotor.getTemperature();
    }

    public double getSteerTemperature() {
        return mAngleMotor.getTemperature();
    }
}
