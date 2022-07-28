package frc.robot;

import com.team2539.cougarlib.util.UpdateManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.TimesliceConstants;

public class Robot extends TimesliceRobot {
    private RobotContainer robotContainer = new RobotContainer();

    // Create an object to manage the timeslices for the subsystems
    private UpdateManager updateManager = new UpdateManager(this);

    public Robot() {
        super(TimesliceConstants.ROBOT_PERIODIC_ALLOCATION, TimesliceConstants.CONTROLLER_PERIOD);

        // Prevents the logging of many errors with our controllers
        DriverStation.silenceJoystickConnectionWarning(true);
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
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}
}
