package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveTrainSubsystemRick;
import frc.robot.commands.HelixAutoTools.TrajectoriesManager;
import frc.robot.commands.HelixAutoTools.TrajectoryFollower;
import frc.robot.commands.HelixAutoTools.Paths.Path;

public class TestAutoFull extends SequentialCommandGroup {
    public TestAutoFull(DriveTrainSubsystemRick drive, TrajectoriesManager trajectoriesManager) {
        // addRequirements(drive, arm);
        Path pt1 = trajectoriesManager.loadTrajectory("Basic0");
        Path pt2 = trajectoriesManager.loadTrajectory("Basic1");
        // addCommands(
        //     new SequentialCommandGroup(
        //         new RunCommand(() -> {
        //             arm.humanPlayerCone();
        //         }, arm),
        //         new WaitCommand(3)
        //     )
        // );
    }
}