package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveTrainSubsystemRick;
import frc.robot.Subsystems.Constant.ArmConstants;
import frc.robot.commands.HelixAutoTools.TrajectoriesManager;
import frc.robot.commands.HelixAutoTools.TrajectoryFollower;
import frc.robot.commands.HelixAutoTools.Paths.Path;

public class LeftAuto extends SequentialCommandGroup {
    public LeftAuto(DriveTrainSubsystemRick drive, ArmSubsystem arm, TrajectoriesManager trajectoriesManager) {
        Path pathing = trajectoriesManager.loadTrajectory("2PieceQuestionable");
        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    arm.zoop(-0.5 * ArmConstants.ClawMaxPercent * 6000);
                    arm.driveRotation(1);
                    System.out.println("running arm");
                }, arm),
                new WaitCommand(2.5),
                new InstantCommand(() -> {
                    arm.driveExtension(1);
                }),
                new WaitCommand(3),
                new InstantCommand(() -> {
                    arm.driveRotation(-1);
                }),
                new WaitCommand(1),
                new InstantCommand(() -> {
                    arm.driveRotation(0);
                    arm.driveExtension(-1);
                    arm.zoop(0.5 * ArmConstants.ClawMaxPercent * 6000);
                }),
                new ParallelDeadlineGroup(
                    new WaitCommand(20),
                    new TrajectoryFollower(drive, pathing, true),
                    new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        new InstantCommand(() -> {
                            System.out.println(":)");
                            arm.wormIntakePos();
                        }),
                        new WaitCommand(1.5),
                        new InstantCommand(() -> {
                            arm.extendIntakePos();
                            arm.zoop(-1 * ArmConstants.ClawMaxPercent * 6000);
                        }),
                        new WaitCommand(3),
                        new InstantCommand(() -> {
                            arm.driveExtension(-1);
                        }),
                        new InstantCommand(() -> {
                            arm.driveRotation(1);
                            arm.driveExtension(1);
                        }),
                        new WaitCommand(3.3),
                        new InstantCommand(() -> {
                            arm.zoop(0.5 * ArmConstants.ClawMaxPercent * 6000);
                        })
                    )
                )
            )
        );
    }
}