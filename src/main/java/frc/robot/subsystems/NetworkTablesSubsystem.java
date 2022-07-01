package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class NetworkTablesSubsystem implements Subsystem {
    private NetworkTable table;
    protected String name;

    public NetworkTablesSubsystem(String tableName) {
        this.name = tableName;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        table = inst.getTable(name);
    }

    public NetworkTableEntry getEntry(String key) {
        return table.getEntry(key);
    }
}
