package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LoggingCommand extends CommandBase {
    protected static NetworkTable commandsTable =
            NetworkTableInstance.getDefault().getTable("commands");

    protected static NetworkTableEntry getEntry(String entryName) {
        return commandsTable.getEntry(entryName);
    }
}
