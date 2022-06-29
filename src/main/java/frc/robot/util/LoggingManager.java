package frc.robot.util;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

public class LoggingManager {
    private static boolean FORCE_LOGGING = false;

    private static Optional<DataLog> log = Optional.empty();

    static {
        if (RobotBase.isReal() || FORCE_LOGGING) {
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
