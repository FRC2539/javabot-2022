package frc.robot.subsystems;

import frc.robot.ShootingSuperstructure;
import frc.robot.util.ShootingComponent;

public class ShootingComponentSubsystem extends NetworkTablesSubsystem implements ShootingComponent {
    protected ShootingSuperstructure shootingSuperstructure;

    public ShootingComponentSubsystem(String tableName) {
        super(tableName);
    }

    public void registerMediator(ShootingSuperstructure shootingSuperstructure) {
        this.shootingSuperstructure = shootingSuperstructure;
    }

    public String getName() {
        return name;
    }
}
