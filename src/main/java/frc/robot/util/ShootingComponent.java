package frc.robot.util;

import frc.robot.ShootingSuperstructure;

public interface ShootingComponent {
    public void registerMediator(ShootingSuperstructure shootingSuperstructure);
    public String getName();
}
