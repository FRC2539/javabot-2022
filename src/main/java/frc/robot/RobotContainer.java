package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.controller.Axis;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.lib.loops.UpdateManager;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TimesliceConstants;
import frc.robot.commands.*;
import frc.robot.strategies.MovingAimStrategy;
import frc.robot.subsystems.*;
import frc.robot.util.AutonomousManager;
import frc.robot.util.TrajectoryLoader;

public class RobotContainer {
    private final ThrustmasterJoystick leftDriveController =
            new ThrustmasterJoystick(ControllerConstants.LEFT_DRIVE_CONTROLLER);
    private final ThrustmasterJoystick rightDriveController =
            new ThrustmasterJoystick(ControllerConstants.RIGHT_DRIVE_CONTROLLER);
    private final LogitechController operatorController =
            new LogitechController(ControllerConstants.OPERATOR_CONTROLLER);

    private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final LightsSubsystem lightsSubsystem = new LightsSubsystem();
    private final BalltrackSubsystem balltrackSubsystem = new BalltrackSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final MachineLearningSubsystem machineLearningSubsystem = new MachineLearningSubsystem();

    private AutonomousManager autonomousManager;
    private TrajectoryLoader trajectoryLoader;

    private ShootingSuperstructure shootingSuperstructure = new ShootingSuperstructure();

    public RobotContainer(TimesliceRobot robot) {
        trajectoryLoader = new TrajectoryLoader();

        UpdateManager updateManager = new UpdateManager(robot);

        autonomousManager = new AutonomousManager(trajectoryLoader, this);

        shootingSuperstructure.registerComponent(swerveDriveSubsystem);
        shootingSuperstructure.registerComponent(shooterSubsystem);
        shootingSuperstructure.registerComponent(limelightSubsystem);
        shootingSuperstructure.registerComponent(balltrackSubsystem);

        updateManager.schedule(swerveDriveSubsystem, TimesliceConstants.DRIVETRAIN_PERIOD);
        updateManager.schedule(shooterSubsystem, TimesliceConstants.SHOOTER_PERIOD);
        updateManager.schedule(limelightSubsystem, TimesliceConstants.LIMELIGHT_PERIOD);
        updateManager.schedule(balltrackSubsystem, TimesliceConstants.BALLTRACK_PERIOD);
        updateManager.schedule(lightsSubsystem);
        updateManager.schedule(climberSubsystem);
        updateManager.schedule(machineLearningSubsystem);

        CommandScheduler.getInstance()
                .setDefaultCommand(
                        swerveDriveSubsystem,
                        new DriveCommand(
                                swerveDriveSubsystem,
                                getDriveForwardAxis(),
                                getDriveStrafeAxis(),
                                getDriveRotationAxis(),
                                balltrackSubsystem));
        CommandScheduler.getInstance().setDefaultCommand(lightsSubsystem, new DefaultLightsCommand(lightsSubsystem));

        configureControllerLayout();
    }

    private void configureControllerLayout() {
        leftDriveController.nameXAxis("Strafe");
        leftDriveController.nameYAxis("Forward");
        rightDriveController.nameXAxis("Rotate");

        leftDriveController.getXAxis().setScale(Constants.SwerveConstants.maxSpeed);
        leftDriveController.getYAxis().setScale(Constants.SwerveConstants.maxSpeed);
        rightDriveController.getXAxis().setScale(Constants.SwerveConstants.maxAngularVelocity);

        leftDriveController.nameLeftTopLeft("Reset Gyro");
        leftDriveController.getLeftTopLeft().whenPressed(() -> swerveDriveSubsystem.resetGyroAngle());
        leftDriveController
                .getLeftTopRight()
                .whenPressed(() -> swerveDriveSubsystem.resetPose(new Pose2d(
                        new Translation2d(),
                        swerveDriveSubsystem.getGyroRotation2d().rotateBy(Rotation2d.fromDegrees(180)))));

        leftDriveController.nameLeftThumb("Lower Climber");
        leftDriveController.nameRightThumb("Raise Climber");
        leftDriveController.nameBottomThumb("Toggle Climber Arm");

        leftDriveController.getLeftThumb().whileHeld(new LowerClimberCommand(climberSubsystem));
        leftDriveController.getRightThumb().whileHeld(new RaiseClimberCommand(climberSubsystem));
        leftDriveController.getBottomThumb().whenPressed(() -> climberSubsystem.toggleClimberArm(), climberSubsystem);

        leftDriveController.nameLeftBottomMiddle("Seizure Mode");
        leftDriveController.getLeftBottomMiddle().whenPressed(new SeizureModeCommand(lightsSubsystem));

        LimelightShootCommand limelightShootCommand = new LimelightShootCommand(shootingSuperstructure);
        LimelightDriveCommand limelightDriveCommand = new LimelightDriveCommand(
                getDriveForwardAxis(), getDriveStrafeAxis(), shootingSuperstructure, lightsSubsystem);

        ParallelCommandGroup defaultShootCommand =
                new ParallelCommandGroup(limelightShootCommand, limelightDriveCommand);

        rightDriveController.nameTrigger("Default Shoot");
        rightDriveController.getTrigger().whileHeld(defaultShootCommand);

        rightDriveController.nameLeftThumb("Intake");
        rightDriveController.nameRightThumb("Aim Assist Intake");
        leftDriveController.nameTrigger("Moving Shooting");

        rightDriveController.getLeftThumb().whileHeld(new IntakeCommand(balltrackSubsystem, lightsSubsystem));
        rightDriveController
                .getRightThumb()
                .whileHeld(new AimAssistIntakeCommand(
                        machineLearningSubsystem,
                        swerveDriveSubsystem,
                        balltrackSubsystem,
                        lightsSubsystem,
                        getDriveForwardAxis(),
                        getDriveStrafeAxis(),
                        getDriveRotationAxis()));

        {
            PIDController pidController = new PIDController(1.7, 0, 0.1, 0.02);
            pidController.enableContinuousInput(-Math.PI, Math.PI);
            MovingAimStrategy movingAimStrategy = new MovingAimStrategy(shootingSuperstructure, pidController);

            ParallelCommandGroup movingShootingCommand = new ParallelCommandGroup(
                    new MovingShootDriveCommand(
                            getDriveForwardAxis(),
                            getDriveStrafeAxis(),
                            shootingSuperstructure,
                            lightsSubsystem,
                            movingAimStrategy),
                    new SequentialCommandGroup(
                            new WaitCommand(0.25), new MovingShootCommand(shootingSuperstructure, movingAimStrategy)));

            leftDriveController.getTrigger().whileHeld(movingShootingCommand);
        }

        operatorController.nameRightTrigger("Default Shoot");
        operatorController.nameRightBumper("Prepare To Shoot");

        operatorController.getRightTrigger().whileHeld(defaultShootCommand);
        operatorController.getRightBumper().whileHeld(new PrepareToShootCommand(shootingSuperstructure));

        operatorController.nameLeftTrigger("Custom Shoot");
        operatorController.nameLeftBumper("High Goal Shot");
        operatorController.nameY("Increase Rear Shot");
        operatorController.nameA("Decrease Rear Shot");
        operatorController.nameX("Increase Front Shot");
        operatorController.nameY("Decrease Rear Shot");

        operatorController.getLeftTrigger().whileHeld(new CustomShootCommand(shootingSuperstructure));
        operatorController
                .getLeftBumper()
                .whileHeld(new SimpleShootCommand(shootingSuperstructure, shooterSubsystem::setFenderHighGoalShot));

        operatorController
                .getY()
                .whenPressed(new ModifyShooterStateCommand(shooterSubsystem, shootingSuperstructure, 100, 0));
        operatorController
                .getA()
                .whenPressed(new ModifyShooterStateCommand(shooterSubsystem, shootingSuperstructure, -100, 0));
        operatorController
                .getX()
                .whenPressed(new ModifyShooterStateCommand(shooterSubsystem, shootingSuperstructure, 0, 100));
        operatorController
                .getB()
                .whenPressed(new ModifyShooterStateCommand(shooterSubsystem, shootingSuperstructure, 0, -100));

        operatorController.nameStart("Enable Temperature Logging");
        operatorController.nameBack("Reverse Balltrack");

        operatorController.getStart().whenPressed(new EnableTemperatureLogging(swerveDriveSubsystem));
        operatorController.getBack().whileHeld(new ReverseBalltrackCommand(balltrackSubsystem, shooterSubsystem));

        rightDriveController.sendButtonNamesToNT();
        leftDriveController.sendButtonNamesToNT();
        operatorController.sendButtonNamesToNT();
    }

    public Command getAutonomousCommand() {
        return autonomousManager.getAutonomousCommand();
    }

    public Axis getDriveForwardAxis() {
        return leftDriveController.getYAxis();
    }

    public Axis getDriveStrafeAxis() {
        return leftDriveController.getXAxis();
    }

    public Axis getDriveRotationAxis() {
        return rightDriveController.getXAxis();
    }

    public ShootingSuperstructure getShootingSuperstructure() {
        return shootingSuperstructure;
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return swerveDriveSubsystem;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    public LightsSubsystem getLightsSubsystem() {
        return lightsSubsystem;
    }

    public BalltrackSubsystem getBalltrackSubsystem() {
        return balltrackSubsystem;
    }

    public ClimberSubsystem getClimberSubsystem() {
        return climberSubsystem;
    }

    public LimelightSubsystem getLimelightSubsystem() {
        return limelightSubsystem;
    }

    public MachineLearningSubsystem getMachineLearningSubsystem() {
        return machineLearningSubsystem;
    }
}
