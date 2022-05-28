package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import frc.robot.common.MathUtils;
import frc.robot.common.control.ShooterState;
import frc.robot.util.Updatable;

public class ShooterSubsystem extends NetworkTablesSubsystem implements Updatable {
    private final DoubleSolenoid shooterAngleSolenoid = new DoubleSolenoid(Constants.PCM_ID, 
                                                                            PneumaticsModuleType.REVPH,
                                                                            Constants.SHOOTER_SOLENOID_FORWARD_CHANNEL,
                                                                            Constants.SHOOTER_SOLENOID_REVERSE_CHANNEL);

    private ShooterAngle targetShooterAngle = ShooterAngle.DISABLED;

    private WPI_TalonFX rearShooterMotor = new WPI_TalonFX(Constants.SHOOTER_REAR_MOTOR_PORT, "CANivore");
    private WPI_TalonFX frontShooterMotor = new WPI_TalonFX(Constants.SHOOTER_FRONT_MOTOR_PORT, "CANivore");

    private final double SHOOTER_F = 0.05;
    private final double SHOOTER_P = 0.13;
    private final double SHOOTER_I = 0;
    private final double SHOOTER_D = 0.05;

    private final double SHOOTER_MOTOR_GEAR_RATIO = 1.5;

    private final double SHOOTER_RPM_ERROR = 40;

    private final ShooterState rejectShooterState = new ShooterState(1000, 800, ShooterAngle.DISABLED);

    private final ShooterState fenderLowGoalShooterState = new ShooterState(1150, 900, ShooterAngle.FAR_SHOT);
    private final ShooterState fenderHighGoalShooterState = new ShooterState(980, 2480, ShooterAngle.CLOSE_SHOT);

    private final ShooterState farShotStartState = new ShooterState(2400, 1800, 0.8);
    private final ShooterState farShotEndState = new ShooterState(4160, 3120, 3);

    private NetworkTableEntry customRearShooterRPMEntry;
    private NetworkTableEntry customFrontShooterRPMEntry;
    private NetworkTableEntry customShooterAngleEntry;

    private NetworkTableEntry rearShooterRPMEntry;
    private NetworkTableEntry frontShooterRPMEntry;

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

        rearShooterMotor.setNeutralMode(NeutralMode.Coast);
        frontShooterMotor.setNeutralMode(NeutralMode.Coast);

        frontShooterMotor.setInverted(true);

        rearShooterRPMEntry = getEntry("Shooter RPM");
        frontShooterRPMEntry = getEntry("Shooter RPM2");

        customRearShooterRPMEntry = getEntry("Custom Rear RPM");
        customFrontShooterRPMEntry = getEntry("Custom Front RPM");
        customShooterAngleEntry = getEntry("Far Shot");

        customRearShooterRPMEntry.setDouble(0);
        customFrontShooterRPMEntry.setDouble(0);
        customShooterAngleEntry.setBoolean(true);
    }

    public void setShooter(ShooterState shooterState) {
        setShooterAngle(shooterState.angle);
        setShooterRPMs(shooterState.rearShooterRPM, shooterState.frontShooterRPM);
    }

    public void setShooterRPMs(double rearShooterRPM, double frontShooterRPM) {
        rearShooterMotor.set(ControlMode.Velocity, rpmToTalonUnits(rearShooterRPM));
        frontShooterMotor.set(ControlMode.Velocity, rpmToTalonUnits(frontShooterRPM));
    }

    public void setShooterAngle(ShooterAngle angle) {
        targetShooterAngle = angle;
    }

    public boolean isShooterAtVelocity() {
        return MathUtils.equalsWithinError(getMotorTargetRPM(rearShooterMotor), getMotorRPM(rearShooterMotor), SHOOTER_RPM_ERROR)
                && MathUtils.equalsWithinError(getMotorTargetRPM(frontShooterMotor), getMotorRPM(frontShooterMotor), SHOOTER_RPM_ERROR);
    }

    private double getMotorTargetRPM(WPI_TalonFX motor) {
        return talonUnitsToRPM(motor.getClosedLoopTarget());
    }

    private double getMotorRPM(WPI_TalonFX motor) {
        return talonUnitsToRPM(motor.getSensorCollection().getIntegratedSensorVelocity());
    }

    public void stopShooter() {
        rearShooterMotor.stopMotor();
        frontShooterMotor.stopMotor();
    }

    public void setRejectShot() {
        // No need to change the shooter angle to reject a ball
        setShooterRPMs(rejectShooterState.rearShooterRPM, rejectShooterState.frontShooterRPM);
    }

    public void setFenderLowGoalShot() {
        setShooter(fenderLowGoalShooterState);
    }

    public void setFenderHightGoalShot() {
        setShooter(fenderHighGoalShooterState);
    }

    public void setFarShot(double distance) {
        setShooter(farShotStartState.interpolateWithDistance(farShotEndState, distance));
    }

    public void setCustomShot() {
        setShooterRPMs(customRearShooterRPMEntry.getDouble(0), customFrontShooterRPMEntry.getDouble(0));
        setShooterAngle(customShooterAngleEntry.getBoolean(true) ? ShooterAngle.FAR_SHOT : ShooterAngle.CLOSE_SHOT);
    }

    @Override
    public void update() {
        switch (targetShooterAngle) {
            case DISABLED:
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

    @Override
    public void periodic() {
        rearShooterRPMEntry.setDouble(getMotorRPM(rearShooterMotor));
        frontShooterRPMEntry.setDouble(getMotorRPM(frontShooterMotor));
    }

    public enum ShooterAngle {
        DISABLED,
        FAR_SHOT,
        CLOSE_SHOT
    }

    private double rpmToTalonUnits(double rpm) {
        return (rpm * 2048 * SHOOTER_MOTOR_GEAR_RATIO) / 600;
    }

    private double talonUnitsToRPM(double units) {
        return (units * 600) / (2048 * SHOOTER_MOTOR_GEAR_RATIO);
    }
}
