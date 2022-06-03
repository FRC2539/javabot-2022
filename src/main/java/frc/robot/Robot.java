package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.UpdateManager;

public class Robot extends TimesliceRobot {
	private RobotContainer robotContainer = new RobotContainer();

	// Create an object to manage the timeslices for the subsystems
	private UpdateManager updateManager = new UpdateManager(this);

	public Robot() {
		super(Constants.ROBOT_PERIODIC_ALLOCATION, Constants.CONTROLLER_PERIOD);

		// Prevents the logging of many errors with our controllers
		DriverStation.silenceJoystickConnectionWarning(true);
	}

	@Override
	public void robotInit() {
		scheduleUpdateFunctions();
	}

	private void scheduleUpdateFunctions() {
		updateManager.schedule(robotContainer.getSwerveDriveSubsystem(), Constants.DRIVETRAIN_PERIOD);
		updateManager.schedule(robotContainer.getShooterSubsystem(), Constants.SHOOTER_PERIOD);
		updateManager.schedule(robotContainer.getBalltrackSubsystem(), Constants.BALLTRACK_PERIOD);
		updateManager.schedule(robotContainer.getLimelightSubsystem(), Constants.LIMELIGHT_PERIOD);
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
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {}

	@Override
	public void testPeriodic() {}
}
