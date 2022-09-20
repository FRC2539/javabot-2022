package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.team2539.cougarlib.MathUtils;
import com.team2539.cougarlib.control.InterpolatingMap;
import com.team2539.cougarlib.util.Updatable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Regressions;
import frc.robot.commands.ModifyShooterStateCommand;
import frc.robot.util.ShooterState;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends ShootingComponentSubsystem implements Updatable {
    private final DoubleSolenoid shooterAngleSolenoid = new DoubleSolenoid(
            GlobalConstants.PCM_ID,
            PneumaticsModuleType.REVPH,
            ShooterConstants.SHOOTER_SOLENOID_FORWARD_CHANNEL,
            ShooterConstants.SHOOTER_SOLENOID_REVERSE_CHANNEL);

    private ShooterAngle targetShooterAngle = ShooterAngle.DISABLED;

    private WPI_TalonFX rearShooterMotor =
            new WPI_TalonFX(ShooterConstants.SHOOTER_REAR_MOTOR_PORT, GlobalConstants.CANIVORE_NAME);
    private WPI_TalonFX frontShooterMotor =
            new WPI_TalonFX(ShooterConstants.SHOOTER_FRONT_MOTOR_PORT, GlobalConstants.CANIVORE_NAME);

    private final double SHOOTER_F = 0.05;
    private final double SHOOTER_P = 0.13;
    private final double SHOOTER_I = 0;
    private final double SHOOTER_D = 0.05;

    private final double SHOOTER_MOTOR_GEAR_RATIO = 1.5;

    private final double SHOOTER_RPM_ERROR = 40;

    private Optional<DoubleSupplier> distanceSupplier = Optional.empty();

    private final ShooterState rejectShooterState = new ShooterState(1000, 800, ShooterAngle.DISABLED);

    private final ShooterState fenderLowGoalShooterState = new ShooterState(1150, 900, ShooterAngle.FAR_SHOT);
    private final ShooterState fenderHighGoalShooterState = new ShooterState(980, 2480, ShooterAngle.CLOSE_SHOT);

    private final InterpolatingMap<ShooterState> farShotStateMap = Regressions.getPracticeShootingMap();

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
        shooterConfiguration.primaryPID.selectedFeedbackSensor =
                TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

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

        ModifyShooterStateCommand.sendShooterMapToNetworkTables(getShooterMap());
    }

    public InterpolatingMap<ShooterState> getShooterMap() {
        return farShotStateMap;
    }

    public void setShooter(ShooterState shooterState) {
        setShooterAngle(shooterState.angle);
        setShooterRPMs(shooterState.rearShooterRPM, shooterState.frontShooterRPM);
    }

    public void setShooterRPMs(double rearShooterRPM, double frontShooterRPM) {
        double rearFeedforward = (rearShooterRPM * SHOOTER_F) / RobotController.getBatteryVoltage();
        double frontFeedforward = (frontShooterRPM * SHOOTER_F) / RobotController.getBatteryVoltage();

        rearShooterMotor.set(ControlMode.Velocity, rpmToTalonUnits(rearShooterRPM) + rearFeedforward);
        frontShooterMotor.set(ControlMode.Velocity, rpmToTalonUnits(frontShooterRPM) + frontFeedforward);
    }

    public void setShooterPercents(double rearShooterPercent, double frontShooterPercent) {
        rearShooterMotor.set(ControlMode.PercentOutput, rearShooterPercent);
        frontShooterMotor.set(ControlMode.PercentOutput, frontShooterPercent);
    }

    public void setShooterAngle(ShooterAngle angle) {
        targetShooterAngle = angle;
    }

    public boolean isShooterAtVelocity() {
        return MathUtils.equalsWithinError(getMotorTargetRPM(rearShooterMotor), getRearMotorRPM(), SHOOTER_RPM_ERROR)
                && MathUtils.equalsWithinError(
                        getMotorTargetRPM(frontShooterMotor), getFrontMotorRPM(), SHOOTER_RPM_ERROR);
    }

    private double getMotorTargetRPM(WPI_TalonFX motor) {
        return talonUnitsToRPM(motor.getClosedLoopTarget());
    }

    private double getRearMotorRPM() {
        return getMotorRPM(rearShooterMotor);
    }

    private double getFrontMotorRPM() {
        return getMotorRPM(frontShooterMotor);
    }

    private double getMotorRPM(WPI_TalonFX motor) {
        return talonUnitsToRPM(motor.getSelectedSensorVelocity());
    }

    public void stopShooter() {
        rearShooterMotor.stopMotor();
        frontShooterMotor.stopMotor();

        distanceSupplier = Optional.empty();
    }

    public void setRejectShot() {
        // No need to change the shooter angle to reject a ball
        setShooterRPMs(rejectShooterState.rearShooterRPM, rejectShooterState.frontShooterRPM);
    }

    public void setFenderLowGoalShot() {
        setShooter(fenderLowGoalShooterState);
    }

    public void setFenderHighGoalShot() {
        setShooter(fenderHighGoalShooterState);
    }

    public ShooterState calculateShooterStateForDistance(double distance) {
        return farShotStateMap.getInterpolated(distance).orElse(new ShooterState());
    }

    public void modifyShooterStateForDistance(double distance, double rearModification, double frontModification) {
        ShooterState ceilingShooterState =
                farShotStateMap.ceilingEntry(distance).getValue();

        ceilingShooterState.rearShooterRPM += rearModification;
        ceilingShooterState.frontShooterRPM += frontModification;
    }

    public void setFarShot(double distance) {
        setShooter(calculateShooterStateForDistance(distance));
    }

    public void setFarShot(DoubleSupplier distanceSupplier) {
        setFarShot(distanceSupplier.getAsDouble());

        this.distanceSupplier = Optional.of(distanceSupplier);
    }

    public void setCustomShot() {
        setShooterRPMs(customRearShooterRPMEntry.getDouble(0), customFrontShooterRPMEntry.getDouble(0));
        setShooterAngle(customShooterAngleEntry.getBoolean(true) ? ShooterAngle.FAR_SHOT : ShooterAngle.CLOSE_SHOT);
    }

    @Override
    public void update() {
        if (distanceSupplier.isPresent()) {
            setFarShot(distanceSupplier.get().getAsDouble());
        }

        switch (targetShooterAngle) {
            case DISABLED:
                shooterAngleSolenoid.set(DoubleSolenoid.Value.kOff);
                break;
            case CLOSE_SHOT:
                shooterAngleSolenoid.set(DoubleSolenoid.Value.kReverse);
                break;
            case FAR_SHOT:
                shooterAngleSolenoid.set(DoubleSolenoid.Value.kForward);
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
