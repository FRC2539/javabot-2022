package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.loops.UpdateManager;
import frc.robot.Constants.TimesliceConstants;
import frc.robot.subsystems.LimelightSubsystem.LimelightPipeline;

public class Robot extends TimesliceRobot {
    public static CTREConfigs ctreConfigs = new CTREConfigs();

    private RobotContainer robotContainer = new RobotContainer();

    // Create an object to manage the timeslices for the subsystems
    private UpdateManager updateManager = new UpdateManager(this);

    public Robot() {
        super(TimesliceConstants.ROBOT_PERIODIC_ALLOCATION, TimesliceConstants.CONTROLLER_PERIOD);

        // Prevents the logging of many errors with our controllers
        DriverStation.silenceJoystickConnectionWarning(true);

        // Send as little data as possible with live window
        LiveWindow.disableAllTelemetry();
    }

    @Override
    public void robotInit() {
        scheduleUpdateFunctions();
    }

    private void scheduleUpdateFunctions() {
        updateManager.schedule(robotContainer.getSwerveDriveSubsystem(), TimesliceConstants.DRIVETRAIN_PERIOD);
        updateManager.schedule(robotContainer.getLimelightSubsystem(), TimesliceConstants.LIMELIGHT_PERIOD);
        updateManager.schedule(robotContainer.getShooterSubsystem(), TimesliceConstants.SHOOTER_PERIOD);
        updateManager.schedule(robotContainer.getBalltrackSubsystem(), TimesliceConstants.BALLTRACK_PERIOD);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        robotContainer.getAutonomousCommand().schedule();
        robotContainer.getLimelightSubsystem().stopUpdatingPoseUsingLimelight();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        robotContainer.getLimelightSubsystem().setPipeline(LimelightPipeline.SHOOT);
        robotContainer.getLimelightSubsystem().startUpdatingPoseUsingLimelight();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {
        robotContainer.getLimelightSubsystem().setPipeline(LimelightPipeline.SHOOT);
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
