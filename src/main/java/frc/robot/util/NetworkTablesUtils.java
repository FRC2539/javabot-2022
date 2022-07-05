package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NetworkTablesUtils {
    public static void sendSendableToSmartDashboard(Sendable sendable) {
        SmartDashboard.putData(sendable);
    }

    public static void sendSendableToShuffleboard(Sendable sendable, String tabName) {
        Shuffleboard.getTab(tabName).add(sendable);
    }
}
