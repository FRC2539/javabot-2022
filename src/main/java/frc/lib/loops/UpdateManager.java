package frc.lib.loops;

import edu.wpi.first.wpilibj.TimesliceRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class UpdateManager {
    private TimesliceRobot robot;

    public UpdateManager(TimesliceRobot robot) {
        this.robot = robot;
    }

    public void schedule(Subsystem subsystem) {
        CommandScheduler.getInstance().registerSubsystem(subsystem);
    }

    public void schedule(Subsystem subsystem, double updateTimeslice) {
        CommandScheduler.getInstance().registerSubsystem(subsystem);
        robot.schedule(() -> ((Updatable)subsystem).update(), updateTimeslice);
    }

}
