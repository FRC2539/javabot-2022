package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.controller.Axis;
import frc.lib.controller.LogitechController;
import frc.lib.controller.ThrustmasterJoystick;
import frc.robot.Constants.ControllerConstants;
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

    public RobotContainer() {
        trajectoryLoader = new TrajectoryLoader();

        autonomousManager = new AutonomousManager(trajectoryLoader, this);

        shootingSuperstructure.registerComponent(swerveDriveSubsystem);
        shootingSuperstructure.registerComponent(shooterSubsystem);
        shootingSuperstructure.registerComponent(limelightSubsystem);
        shootingSuperstructure.registerComponent(balltrackSubsystem);

        CommandScheduler.getInstance().registerSubsystem(swerveDriveSubsystem);
        CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);
        CommandScheduler.getInstance().registerSubsystem(lightsSubsystem);
        CommandScheduler.getInstance().registerSubsystem(balltrackSubsystem);
        CommandScheduler.getInstance().registerSubsystem(climberSubsystem);
        CommandScheduler.getInstance().registerSubsystem(limelightSubsystem);
        CommandScheduler.getInstance().registerSubsystem(machineLearningSubsystem);

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
        leftDriveController.getXAxis().setScale(Constants.SwerveConstants.maxSpeed);
        leftDriveController.getYAxis().setScale(Constants.SwerveConstants.maxSpeed);
        rightDriveController.getXAxis().setScale(Constants.SwerveConstants.maxAngularVelocity);

        leftDriveController.getLeftTopLeft().whenPressed(() -> swerveDriveSubsystem.resetGyroAngle());
        leftDriveController
                .getLeftTopRight()
                .whenPressed(() -> swerveDriveSubsystem.resetPose(new Pose2d(
                        new Translation2d(),
                        swerveDriveSubsystem.getGyroRotation2d().rotateBy(Rotation2d.fromDegrees(180)))));

        leftDriveController.getLeftThumb().whileHeld(new LowerClimberCommand(climberSubsystem));
        leftDriveController.getRightThumb().whileHeld(new RaiseClimberCommand(climberSubsystem));
        leftDriveController.getBottomThumb().whenPressed(() -> climberSubsystem.toggleClimberArm(), climberSubsystem);

        leftDriveController.getLeftBottomMiddle().whenPressed(new SeizureModeCommand(lightsSubsystem));

        LimelightShootCommand limelightShootCommand = new LimelightShootCommand(shootingSuperstructure);
        LimelightDriveCommand limelightDriveCommand = new LimelightDriveCommand(
                getDriveForwardAxis(), getDriveStrafeAxis(), shootingSuperstructure, lightsSubsystem);

        ParallelCommandGroup defaultShootCommand =
                new ParallelCommandGroup(limelightShootCommand, limelightDriveCommand);

        rightDriveController.getTrigger().whileHeld(defaultShootCommand);

        rightDriveController.getLeftThumb().whileHeld(new IntakeCommand(balltrackSubsystem, lightsSubsystem));
        rightDriveController
                .getRightThumb()
                .whileHeld(new BallCollectCommand(machineLearningSubsystem, swerveDriveSubsystem, balltrackSubsystem));

        {
            PIDController pidController = new PIDController(1.9, 0, 0.08, 0.02);
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

        operatorController.getRightTrigger().whileHeld(defaultShootCommand);

        operatorController.getLeftTrigger().whileHeld(new CustomShootCommand(shootingSuperstructure));
        operatorController.getRightBumper().whileHeld(new PrepareToShootCommand(shootingSuperstructure));

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

        operatorController.getStart().whenPressed(new EnableTemperatureLogging(swerveDriveSubsystem));
        operatorController.getBack().whileHeld(new ReverseBalltrackCommand(balltrackSubsystem, shooterSubsystem));
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
