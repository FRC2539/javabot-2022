package frc.robot.util;

import edu.wpi.first.wpilibj.TimesliceRobot;

public class UpdateManager {
    private TimesliceRobot robot;

    public UpdateManager(TimesliceRobot robot) {
        this.robot = robot;
    }

    public void schedule(Updatable subsystem, double updateTimeslice) {
        robot.schedule(() -> subsystem.update(), updateTimeslice);
    }
}
