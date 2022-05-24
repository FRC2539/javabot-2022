package frc.robot;

import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimesliceRobot {
	private static double robotPeriodicAllocation = 0.02;
	private static double controllerPeriod = 0.05;

	public Robot() {
		super(robotPeriodicAllocation, controllerPeriod);
	}

	private RobotContainer robotContainer = new RobotContainer();

	@Override
	public void robotInit() {
		this.schedule(() -> robotContainer.getSwerveDriveSubsystem().update(), 0.015);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
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
