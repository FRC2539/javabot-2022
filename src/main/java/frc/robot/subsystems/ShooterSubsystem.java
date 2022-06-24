package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.common.MathUtils;
import frc.robot.common.control.InterpolatingMap;
import frc.robot.common.control.ShooterState;
import frc.robot.util.Updatable;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends NetworkTablesSubsystem implements Updatable {
    private final DoubleSolenoid shooterAngleSolenoid = new DoubleSolenoid(
            Constants.PCM_ID,
            PneumaticsModuleType.REVPH,
            Constants.SHOOTER_SOLENOID_FORWARD_CHANNEL,
            Constants.SHOOTER_SOLENOID_REVERSE_CHANNEL);

    private ShooterAngle targetShooterAngle = ShooterAngle.DISABLED;

    private WPI_TalonFX rearShooterMotor = new WPI_TalonFX(Constants.SHOOTER_REAR_MOTOR_PORT, Constants.CANIVORE_NAME);
    private WPI_TalonFX frontShooterMotor =
            new WPI_TalonFX(Constants.SHOOTER_FRONT_MOTOR_PORT, Constants.CANIVORE_NAME);

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

    private final InterpolatingMap<ShooterState> farShotStateMap = new InterpolatingMap<ShooterState>();

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
        
        /*
        note, some of these values have the radius added into them afterwards, when adding new points
        in the future do not do this as it means you can take the value straight from the
        limelight subsystem instead
        */
        farShotStateMap.put(2.07 + LimelightSubsystem.TARGET_RADIUS, new ShooterState(2300, 1600));
        farShotStateMap.put(2.74 + LimelightSubsystem.TARGET_RADIUS, new ShooterState(2650, 1550));
        farShotStateMap.put(3.62 + LimelightSubsystem.TARGET_RADIUS, new ShooterState(3150, 1550));
        farShotStateMap.put(4.57, new ShooterState(3700, 1800)); //do not re-add in radius
        farShotStateMap.put(5.35 + LimelightSubsystem.TARGET_RADIUS, new ShooterState(4200, 2000));
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
