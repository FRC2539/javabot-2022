package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;
import com.team2539.cougarlib.util.Updatable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class BalltrackSubsystem extends NetworkTablesSubsystem implements Updatable {
    private WPI_TalonSRX conveyorMotor = new WPI_TalonSRX(Constants.BALLTRACK_CONVEYOR_MOTOR_PORT);
    private WPI_TalonSRX chamberMotor = new WPI_TalonSRX(Constants.BALLTRACK_CHAMBER_MOTOR_PORT);
    private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.BALLTRACK_INTAKE_MOTOR_PORT);

    private AnalogInput conveyorProximitySensor = new AnalogInput(Constants.BALLTRACK_CONVEYOR_SENSOR_PORT);
    private AnalogInput chamberProximitySensor = new AnalogInput(Constants.BALLTRACK_CHAMBER_SENSOR_PORT);

    private ColorSensorV3 chamberColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(
            Constants.PCM_ID,
            PneumaticsModuleType.REVPH,
            Constants.BALLTRACK_INTAKE_SOLENOID_FORWARD_CHANNEL,
            Constants.BALLTRACK_INTAKE_SOLENOID_REVERSE_CHANNEL);

    private final double INTAKE_MOTOR_SPEED = 0.9;

    private final double SHOOTING_CONVEYOR_SPEED = 0.9;
    private final double SHOOTING_CHAMBER_SPEED = 1;

    private final double INTAKE_CONVEYOR_SPEED = 1;
    private final double INTAKE_CHAMBER_SPEED = 0.7;

    private final double PROXIMITY_SENSOR_THRESHOLD = 50;

    private NetworkTableEntry conveyorBallPresentEntry;
    private NetworkTableEntry chamberBallPresentEntry;

    private BalltrackMode balltrackMode = BalltrackMode.DISABLED;

    private boolean conveyorBallIsPresent = false;
    private boolean chamberBallIsPresent = false;

    private ShooterSubsystem shooterSubsystem;

    public BalltrackSubsystem(ShooterSubsystem shooterSubsystem) {
        super("BallSystem");

        this.shooterSubsystem = shooterSubsystem;

        conveyorMotor.setNeutralMode(NeutralMode.Brake);
        chamberMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        conveyorMotor.setInverted(true);
        chamberMotor.setInverted(true);
        intakeMotor.setInverted(true);

        chamberColorSensor.configureColorSensor(
                ColorSensorV3.ColorSensorResolution.kColorSensorRes18bit,
                ColorSensorV3.ColorSensorMeasurementRate.kColorRate25ms,
                ColorSensorV3.GainFactor.kGain1x);

        conveyorBallPresentEntry = getEntry("Conveyor Ball");
        chamberBallPresentEntry = getEntry("Chamber Ball");
    }

    public void intakeWithConveyorMotor() {
        conveyorMotor.set(ControlMode.PercentOutput, INTAKE_CONVEYOR_SPEED);
    }

    public void intakeWithChamberMotor() {
        chamberMotor.set(ControlMode.PercentOutput, INTAKE_CHAMBER_SPEED);
    }

    public void shootWithConveyorMotor() {
        conveyorMotor.set(ControlMode.PercentOutput, SHOOTING_CONVEYOR_SPEED);
    }

    public void shootWithChamberMotor() {
        chamberMotor.set(ControlMode.PercentOutput, SHOOTING_CHAMBER_SPEED);
    }

    public void reverseChamberAndConveyor() {
        conveyorMotor.set(ControlMode.PercentOutput, -SHOOTING_CONVEYOR_SPEED);
        chamberMotor.set(ControlMode.PercentOutput, -SHOOTING_CHAMBER_SPEED);
    }

    public void stopChamberMotor() {
        chamberMotor.stopMotor();
    }

    public void stopConveyorMotor() {
        conveyorMotor.stopMotor();
    }

    public void stopChamberAndConveyor() {
        stopConveyorMotor();
        stopChamberMotor();
    }

    public void runIntakeMotor() {
        intakeMotor.set(ControlMode.PercentOutput, INTAKE_MOTOR_SPEED);
    }

    public void reverseIntakeMotor() {
        intakeMotor.set(ControlMode.PercentOutput, -INTAKE_MOTOR_SPEED);
    }

    public void stopIntakeMotor() {
        intakeMotor.stopMotor();
    }

    public void extendIntake() {
        intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retractIntake() {
        intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isConveyorBallPresent() {
        return conveyorProximitySensor.getValue() < PROXIMITY_SENSOR_THRESHOLD;
    }

    public boolean isChamberBallPresent() {
        return chamberProximitySensor.getValue() < PROXIMITY_SENSOR_THRESHOLD;
    }

    public boolean hasOneBall() {
        return conveyorBallIsPresent || chamberBallIsPresent;
    }

    public boolean isBalltrackFull() {
        return conveyorBallIsPresent && chamberBallIsPresent;
    }

    public Alliance getChamberBallColor() {
        Color colorMeasurement = chamberColorSensor.getColor();

        if (!chamberBallIsPresent) return Alliance.Invalid;
        else if (colorMeasurement.blue > colorMeasurement.red) return Alliance.Blue;
        else return Alliance.Red;
    }

    public void setBalltrackMode(BalltrackMode balltrackMode) {
        this.balltrackMode = balltrackMode;
    }

    public void intakeMode() {
        if (balltrackMode == BalltrackMode.SHOOT || balltrackMode == BalltrackMode.INTAKE_AND_SHOOT)
            setBalltrackMode(BalltrackMode.INTAKE_AND_SHOOT);
        else setBalltrackMode(BalltrackMode.INTAKE);
    }

    public void stopIntakeMode() {
        if (balltrackMode == BalltrackMode.INTAKE_AND_SHOOT) setBalltrackMode(BalltrackMode.SHOOT);
        else setBalltrackMode(BalltrackMode.DISABLED);
    }

    public void shootMode() {
        if (balltrackMode == BalltrackMode.INTAKE || balltrackMode == BalltrackMode.INTAKE_AND_SHOOT)
            setBalltrackMode(BalltrackMode.INTAKE_AND_SHOOT);
        else setBalltrackMode(BalltrackMode.SHOOT);
    }

    public void stopShootMode() {
        if (balltrackMode == BalltrackMode.INTAKE_AND_SHOOT) setBalltrackMode(BalltrackMode.INTAKE);
        else setBalltrackMode(BalltrackMode.DISABLED);
    }

    public BalltrackMode getBalltrackMode() {
        return balltrackMode;
    }

    public boolean isIntaking() {
        return getBalltrackMode() == BalltrackMode.INTAKE || getBalltrackMode() == BalltrackMode.INTAKE_AND_SHOOT;
    }

    @Override
    public void update() {
        conveyorBallIsPresent = isConveyorBallPresent();
        chamberBallIsPresent = isChamberBallPresent();

        switch (balltrackMode) {
            case DISABLED:
                stopChamberAndConveyor();
                stopIntakeMotor();
                retractIntake();
                break;
            case PREPARE:
                prepareBalls();
                break;
            case INTAKE:
                extendIntake();
                intakeBalls();
                break;
            case SHOOT:
                retractIntake();
                stopIntakeMotor();

                if (shooterSubsystem.isShooterAtVelocity()) {
                    shootWithConveyorMotor();
                    shootWithChamberMotor();
                }

                break;
            case INTAKE_AND_SHOOT:
                extendIntake();

                if (shooterSubsystem.isShooterAtVelocity()) {
                    shootWithConveyorMotor();
                    shootWithChamberMotor();
                    runIntakeMotor();
                } else {
                    intakeBalls();
                }

                break;
            case REVERSE:
                reverseChamberAndConveyor();
                reverseIntakeMotor();
                break;
        }
    }

    private void prepareBalls() {
        if (isBalltrackFull()) {
            stopChamberAndConveyor();
        } else if (chamberBallIsPresent) {
            intakeWithConveyorMotor();
            stopChamberMotor();
        } else {
            intakeWithConveyorMotor();
            intakeWithChamberMotor();
        }
    }

    private void intakeBalls() {
        if (isBalltrackFull()) {
            stopChamberAndConveyor();
            stopIntakeMotor();
            // retractIntake();
        } else if (chamberBallIsPresent) {
            intakeWithConveyorMotor();
            stopChamberMotor();
            runIntakeMotor();
        } else {
            intakeWithConveyorMotor();
            intakeWithChamberMotor();
            runIntakeMotor();
        }
    }

    @Override
    public void periodic() {
        conveyorBallPresentEntry.setBoolean(conveyorBallIsPresent);
        chamberBallPresentEntry.setBoolean(chamberBallIsPresent);
    }

    public enum BalltrackMode {
        DISABLED,
        PREPARE,
        INTAKE,
        SHOOT,
        INTAKE_AND_SHOOT,
        REVERSE
    }
}
