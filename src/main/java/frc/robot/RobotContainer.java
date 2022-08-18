package frc.robot;

import com.team2539.cougarlib.controller.Axis;
import com.team2539.cougarlib.controller.LogitechController;
import com.team2539.cougarlib.controller.ThrustmasterJoystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.*;
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
        leftDriveController.getXAxis().setScale(SwerveDriveSubsystem.MAX_VELOCITY);
        leftDriveController.getYAxis().setScale(SwerveDriveSubsystem.MAX_VELOCITY);
        rightDriveController.getXAxis().setScale(SwerveDriveSubsystem.MAX_ANGULAR_VELOCITY);

        leftDriveController.getLeftTopLeft().whenPressed(() -> swerveDriveSubsystem.resetGyroAngle());

        leftDriveController.getLeftThumb().whileHeld(new LowerClimberCommand(climberSubsystem));
        leftDriveController.getRightThumb().whileHeld(new RaiseClimberCommand(climberSubsystem));
        leftDriveController.getBottomThumb().whenPressed(() -> climberSubsystem.toggleClimberArm(), climberSubsystem);

        leftDriveController.getLeftTopRight().whenPressed(new SeizureModeCommand(lightsSubsystem));

        LimelightShootCommand limelightShootCommand = new LimelightShootCommand(shootingSuperstructure);

        leftDriveController
                .getTrigger()
                .whileHeld(
                        limelightShootCommand);
        rightDriveController
                .getTrigger()
                .whileHeld(
                        new SimpleShootCommand(shootingSuperstructure, () -> shooterSubsystem.setFenderHighGoalShot()));

        rightDriveController.getLeftThumb().whileHeld(new IntakeCommand(balltrackSubsystem, lightsSubsystem));
        rightDriveController
                .getBottomThumb()
                .whileHeld(new LimelightDriveCommand(
                        getDriveForwardAxis(), getDriveStrafeAxis(), shootingSuperstructure, lightsSubsystem));
        rightDriveController
                .getRightThumb()
                .whileHeld(new BallCollectCommand(machineLearningSubsystem, swerveDriveSubsystem, balltrackSubsystem));

        operatorController.getRightTrigger().whileHeld(limelightShootCommand);
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

    private Axis getDriveForwardAxis() {
        return leftDriveController.getYAxis();
    }

    private Axis getDriveStrafeAxis() {
        return leftDriveController.getXAxis();
    }

    private Axis getDriveRotationAxis() {
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
