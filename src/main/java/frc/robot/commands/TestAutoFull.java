package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.DriveTrainSubsystemRick;
import frc.robot.commands.HelixAutoTools.TrajectoryFollower;
import frc.robot.commands.HelixAutoTools.Paths.TestAuto;

public class TestAutoFull extends SequentialCommandGroup {
    public TestAutoFull(DriveTrainSubsystemRick drive) {
        addCommands(
            new TrajectoryFollower(drive, new TestAuto())
        );
    }
}