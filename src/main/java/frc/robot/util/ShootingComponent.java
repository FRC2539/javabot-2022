package frc.robot.util;

import frc.robot.ShootingSuperstructure;

public interface ShootingComponent {
    public void registerSuperstructure(ShootingSuperstructure shootingSuperstructure);

    public String getName();
}
