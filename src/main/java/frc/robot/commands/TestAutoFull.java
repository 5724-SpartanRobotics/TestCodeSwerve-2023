package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.DriveTrainSubsystemRick;
import frc.robot.commands.HelixAutoTools.TrajectoriesManager;
import frc.robot.commands.HelixAutoTools.TrajectoryFollower;
import frc.robot.commands.HelixAutoTools.Paths.Path;

public class TestAutoFull extends SequentialCommandGroup {
    public TestAutoFull(DriveTrainSubsystemRick drive, TrajectoriesManager trajectoriesManager) {
        Path testAuto = trajectoriesManager.loadTrajectory("StraightLineTraj");
        addCommands(
            new TrajectoryFollower(drive, testAuto)
        );
    }
}