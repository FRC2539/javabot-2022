package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

abstract public class NetworkTablesSubsystem implements Subsystem {
    private NetworkTable table;

    public NetworkTablesSubsystem(String tableName) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        table = inst.getTable(tableName);
    }

    public NetworkTableEntry getEntry(String key) {
        return table.getEntry(key);
    }
}
