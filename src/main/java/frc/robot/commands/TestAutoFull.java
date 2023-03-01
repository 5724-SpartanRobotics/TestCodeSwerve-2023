package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.DriveTrainSubsystemRick;
import frc.robot.commands.HelixAutoTools.TrajectoriesManager;
import frc.robot.commands.HelixAutoTools.TrajectoryFollower;
import frc.robot.commands.HelixAutoTools.Paths.Path;

public class TestAutoFull extends SequentialCommandGroup {
    public TestAutoFull(DriveTrainSubsystemRick drive, TrajectoriesManager trajectoriesManager) {
        Path pt1 = trajectoriesManager.loadTrajectory("Basic0");
        Path pt2 = trajectoriesManager.loadTrajectory("Basic1");
        addCommands(
            new SequentialCommandGroup(
                new TrajectoryFollower(drive, pt1),
                new WaitCommand(3),
                new TrajectoryFollower(drive, pt2)    
            )
        );
    }
}