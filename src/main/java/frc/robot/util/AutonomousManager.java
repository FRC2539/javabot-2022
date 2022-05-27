package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutonomousManager {
    private NetworkTable autonomousTable;

    private NetworkTableEntry selectedAuto;

    private final String[] autoStrings = {"test"};

    public AutonomousManager() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        autonomousTable = inst.getTable("Autonomous");

        autonomousTable.getEntry("autos").setStringArray(autoStrings);

        selectedAuto = autonomousTable.getEntry("selectedAuto");

        // Choose the first auto as the default
        selectedAuto.setString(autoStrings[0]);
    }

    public Command getTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        SwerveDriveSubsystem driveSubsystem = container.getSwerveDriveSubsystem();
        
        ChassisSpeeds velocity = new ChassisSpeeds(1, 0, 0);

        command.addCommands(new WaitCommand(2)
                .alongWith(new InstantCommand(() -> driveSubsystem.drive(velocity, false), driveSubsystem)));

        return command;
    }

    public Command getAutonomousCommand(RobotContainer container) {
        switch (selectedAuto.getString(autoStrings[0])) {
            case "test":
                return getTestAutoCommand(container);        
        }

        // Return an empty command group if no auto is specified
        return new SequentialCommandGroup();
    }
}
