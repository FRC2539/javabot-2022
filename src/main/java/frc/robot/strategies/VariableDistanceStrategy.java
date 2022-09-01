package frc.robot.strategies;

import frc.robot.util.ShooterState;

public interface VariableDistanceStrategy extends LimelightAimStrategy {
    public ShooterState getShooterState();
}
