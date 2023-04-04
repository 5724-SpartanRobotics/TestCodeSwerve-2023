package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveTrainSubsystemRick;
import frc.robot.Subsystems.Constant.ArmConstants;
import frc.robot.commands.HelixAutoTools.TrajectoriesManager;
import frc.robot.commands.HelixAutoTools.TrajectoryFollower;
import frc.robot.commands.HelixAutoTools.Paths.Path;

public class NoMovement extends SequentialCommandGroup {
    public NoMovement(DriveTrainSubsystemRick drive, ArmSubsystem arm, TrajectoriesManager trajectoriesManager) {
        Path pathing = trajectoriesManager.loadTrajectory("Basic0");
        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    drive.setFlag();
                    arm.zoop(-0.5 * ArmConstants.ClawMaxPercent * 6000);
                    arm.wormFullUp();
                    System.out.println("running arm");
                }, arm),
                new WaitCommand(2.5),
                new InstantCommand(() -> {
                    arm.extendFullOut();
                    arm.wormIncremental(true, false);
                }),
                new WaitCommand(3),
                new InstantCommand(() -> {
                    arm.wormIncremental(false, true);
                    arm.wormIncremental(false, false);
                }),
                new WaitCommand(1),
                new InstantCommand(() -> {
                    arm.extendFullIn();
                    arm.zoop(0.5 * ArmConstants.ClawMaxPercent * 6000);
                })
            )
        );
    }
}