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

public class ParkYeet extends SequentialCommandGroup {
    public ParkYeet(DriveTrainSubsystemRick drive, ArmSubsystem arm, TrajectoriesManager trajectoriesManager, int inv) {
        Path pathing = trajectoriesManager.loadTrajectory("Basic0");
        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    drive.setFlag();
                    arm.zoop(-0.5 * ArmConstants.ClawMaxPercent * 6000);
                    arm.wormFullUp();
                    //System.out.println("running arm");
                }, arm),
                new WaitCommand(2),
                new InstantCommand(() -> {
                    arm.extendFullOut();
                    arm.wormIncremental(true, false);
                }),
                new WaitCommand(1.2),
                new InstantCommand(() -> {
                    //place the cone
                    arm.wormFullUp();
                    arm.wormIncremental(false, true);
                }),
                new WaitCommand(1.1),
                new InstantCommand(() -> {
                    arm.extendFullIn();
                }),
                new WaitCommand(0.2),
                new InstantCommand(() -> {
                    arm.zoop(0.3 * ArmConstants.ClawMaxPercent * 6000);
                    drive.drive(new Translation2d(2, 0), 0);
                }),
                new WaitCommand(0.3),
                new InstantCommand(() -> {
                    arm.wormFullDown();
                }),
                new ParallelDeadlineGroup(
                    new WaitCommand(2), 
                    new RunCommand(() -> {
                        drive.spiiiiiiin(0, 1);
                    }, drive, arm)
                ),
                new InstantCommand(() -> {
                    // funny drive
                    drive.drive(new Translation2d(2, 0), 0);
                }),
                new WaitCommand(1),
                new InstantCommand(() -> {
                    // funny drive
                    arm.wormIntakePos();
                }),
                new WaitCommand(1), 
                new InstantCommand(() -> {
                    // funny drive
                    arm.extendIntakePos();
                    arm.zoop(-0.5 * ArmConstants.ClawMaxPercent * 6000);
                    drive.setFlag();
                }),
                new WaitCommand(1),
                new ParallelDeadlineGroup(
                    new WaitCommand(4.2),
                    new RunCommand(() -> {
                        drive.heyAreWeUpYet();
                    }, drive, arm),
                    new SequentialCommandGroup(
                        new WaitCommand(1),
                        new InstantCommand(() -> {
                            // funny drive
                            arm.extendFullIn();
                            arm.wormFullUp();
                        })
                    ) 
                ),
                new ParallelDeadlineGroup(
                    new WaitCommand(2),
                    new RunCommand(() -> {
                        drive.spiiiiiiin(180, -2 * inv);
                        drive.yeet(1);
                    }, drive, arm)
                )
            )
        );
    }
}