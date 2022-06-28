package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;

public class LoggingManager {
    private static Optional<DataLog> log = Optional.empty();

    static {
        if (RobotBase.isReal()) {
            // Set the logging directory
            DataLogManager.start("logs");

            // Disable network tables logging, as we use network tables intensively
            DataLogManager.logNetworkTables(false);

            log = Optional.of(DataLogManager.getLog());
        }
    }

    public static Optional<DataLog> getLog() {
        return log;
    }

    public static void logMessage(String message) {
        DataLogManager.log(message);
    }
}
