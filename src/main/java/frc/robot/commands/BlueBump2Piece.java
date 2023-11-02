package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.math.geometry.Translation2d;
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

public class BlueBump2Piece extends SequentialCommandGroup {
    public BlueBump2Piece(DriveTrainSubsystemRick drive, ArmSubsystem arm, TrajectoriesManager trajectoriesManager) {
        Path pathing = trajectoriesManager.loadTrajectory("BRBBlueBump");
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
                    drive.drive(new Translation2d(-1, 0), 0);
                }),
                new WaitCommand(1.2),
                new InstantCommand(() -> {
                    drive.drive(new Translation2d(), 0);
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
                }),
                new ParallelDeadlineGroup(
                    new WaitCommand(20),
                    new TrajectoryFollower(drive, pathing, false),
                    new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        new InstantCommand(() -> {
                            //System.out.println(":)");
                            arm.wormIntakePos();
                        }),
                        new WaitCommand(1.5),
                        new InstantCommand(() -> {
                            arm.extendIntakePos();
                            arm.zoop(-1 * ArmConstants.ClawMaxPercent * 6000);
                        }),
                        new WaitCommand(3),
                        new InstantCommand(() -> {
                            arm.wormFullUp();
                            arm.extendFullIn();
                        }),
                        new WaitCommand(3.3),
                        new InstantCommand(() -> {
                            arm.extendFullOut();
                        })
                    )
                )
            )
        );
    }
}