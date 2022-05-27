package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import frc.robot.util.Updatable;

public class ShooterSubsystem extends NetworkTablesSubsystem implements Updatable {
    private final DoubleSolenoid shooterAngleSolenoid = new DoubleSolenoid(Constants.PCM_ID, 
                                                                            PneumaticsModuleType.REVPH,
                                                                            Constants.SHOOTER_SOLENOID_FORWARD_CHANNEL,
                                                                            Constants.SHOOTER_SOLENOID_REVERSE_CHANNEL);

    private ShooterAngle targetShooterAngle = ShooterAngle.NOT_SET;

    private WPI_TalonFX rearShooterMotor = new WPI_TalonFX(Constants.SHOOTER_REAR_MOTOR_PORT);
    private WPI_TalonFX frontShooterMotor = new WPI_TalonFX(Constants.SHOOTER_FRONT_MOTOR_PORT);

    private double rearShooterRPM = 0;
    private double frontShooterRPM = 0;

    private final double SHOOTER_F = 0.05;
    private final double SHOOTER_P = 0.13;
    private final double SHOOTER_I = 0;
    private final double SHOOTER_D = 0.05;


    public ShooterSubsystem() {
        super("Shooter");

        TalonFXConfiguration shooterConfiguration = new TalonFXConfiguration();
        shooterConfiguration.slot0.kF = SHOOTER_F;
        shooterConfiguration.slot0.kP = SHOOTER_P;
        shooterConfiguration.slot0.kI = SHOOTER_I;
        shooterConfiguration.slot0.kD = SHOOTER_D;
        shooterConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

        rearShooterMotor.configAllSettings(shooterConfiguration);
        frontShooterMotor.configAllSettings(shooterConfiguration);
    }

    @Override
    public void update() {
        updateShooterAngle();
        updateShooterRPMs();
    }

    public void setShooterRPMs(double rearShooterRPM, double frontShooterRPM) {
        this.rearShooterRPM = rearShooterRPM;
        this.frontShooterRPM = frontShooterRPM;
    }

    public void setShooterAngle(ShooterAngle angle) {
        targetShooterAngle = angle;
    }

    private void updateShooterRPMs() {

    }

    private void updateShooterAngle() {
        switch (targetShooterAngle) {
            case NOT_SET:
                shooterAngleSolenoid.set(DoubleSolenoid.Value.kOff);
                break;
            case CLOSE_SHOT:
                shooterAngleSolenoid.set(DoubleSolenoid.Value.kForward);
                break;
            case FAR_SHOT:
                shooterAngleSolenoid.set(DoubleSolenoid.Value.kReverse);
                break;
        }
    }

    public enum ShooterAngle {
        NOT_SET,
        FAR_SHOT,
        CLOSE_SHOT
    }
}
