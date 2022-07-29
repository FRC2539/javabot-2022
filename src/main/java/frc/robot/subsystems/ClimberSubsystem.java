package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.GlobalConstants;

public class ClimberSubsystem extends NetworkTablesSubsystem {
    private DoubleSolenoid climberSolenoid = new DoubleSolenoid(
            GlobalConstants.PCM_ID,
            PneumaticsModuleType.REVPH,
            ClimberConstants.CLIMBER_SOLENOID_FORWARD_CHANNEL,
            ClimberConstants.CLIMBER_SOLENOID_REVERSE_CHANNEL);

    private WPI_TalonFX climberMotor =
            new WPI_TalonFX(ClimberConstants.CLIMBER_MOTOR_PORT, GlobalConstants.CANIVORE_NAME);

    private final boolean USE_LIMITS = true;

    private final double UPPER_LIMIT = 230000;
    private final double LOWER_LIMIT = 1000;

    private final double CLIMBER_MOTOR_SPEED = 1;

    private final double CLIMBER_RAMP_DURATION = 0.29;

    private final double PROXIMITY_SENSOR_THRESHOLD = 50;

    private AnalogInput proximitySensorRight = new AnalogInput(ClimberConstants.CLIMBER_SENSOR_RIGHT_PORT);

    private AnalogInput proximitySensorLeft = new AnalogInput(ClimberConstants.CLIMBER_SENSOR_LEFT_PORT);

    private NetworkTableEntry positionEntry;

    public ClimberSubsystem() {
        super("Climber");

        TalonFXConfiguration configuration = new TalonFXConfiguration();
        configuration.forwardSoftLimitEnable = USE_LIMITS;
        configuration.forwardSoftLimitThreshold = UPPER_LIMIT;
        configuration.reverseSoftLimitEnable = USE_LIMITS;
        configuration.reverseSoftLimitThreshold = LOWER_LIMIT;
        configuration.openloopRamp = CLIMBER_RAMP_DURATION;

        climberMotor.configAllSettings(configuration);
        climberMotor.setNeutralMode(NeutralMode.Brake);

        positionEntry = getEntry("Position");
    }

    public void raiseClimber() {
        climberMotor.set(ControlMode.PercentOutput, CLIMBER_MOTOR_SPEED);
    }

    public void lowerClimber() {
        climberMotor.set(ControlMode.PercentOutput, -CLIMBER_MOTOR_SPEED);
    }

    public void stopClimber() {
        climberMotor.stopMotor();
    }

    public void setClimberStraightUp() {
        climberSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setClimberAngled() {
        climberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void toggleClimberArm() {
        switch (climberSolenoid.get()) {
            case kOff:
                setClimberAngled();
                break;
            case kForward:
                setClimberAngled();
                break;
            case kReverse:
                setClimberStraightUp();
                break;
        }
    }

    @Override
    public void periodic() {
        positionEntry.setDouble(getPosition());
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

    public boolean isClimberOnBar() {
        return proximitySensorLeft.getValue() < PROXIMITY_SENSOR_THRESHOLD
                && proximitySensorRight.getValue() < PROXIMITY_SENSOR_THRESHOLD;
    }
}
