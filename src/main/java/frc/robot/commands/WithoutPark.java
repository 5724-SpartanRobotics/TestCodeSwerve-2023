package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveTrainSubsystemRick;
import frc.robot.Subsystems.Constant.ArmConstants;
import frc.robot.commands.HelixAutoTools.TrajectoriesManager;
import frc.robot.commands.HelixAutoTools.TrajectoryFollower;
import frc.robot.commands.HelixAutoTools.Paths.Path;

public class WithoutPark extends SequentialCommandGroup {
    public WithoutPark(DriveTrainSubsystemRick drive, ArmSubsystem arm, TrajectoriesManager trajectoriesManager) {
        Path pathing = trajectoriesManager.loadTrajectory("Basic0");
        addCommands(
            new SequentialCommandGroup(
                // new InstantCommand(() -> {
                //     arm.driveRotation(1);
                //     System.out.println("running arm");
                // }, arm),
                // new WaitCommand(2),
                // new InstantCommand(() -> {
                //     arm.driveExtension(1);
                //     System.out.println("extending arm");
                // }, arm),
                // new WaitCommand(4),
                // new InstantCommand(() -> {
                //     arm.zoop(1  * ArmConstants.ClawMaxPercent * 6000);
                //     System.out.println("running claw");
                // }, arm),
                // new WaitCommand(2),
                // new InstantCommand(() -> {
                //     arm.driveRotation(-1);
                //     arm.driveExtension(-1);
                // }),
                // new WaitCommand(2),
                new TrajectoryFollower(drive, pathing)
            )
        );
    }
}