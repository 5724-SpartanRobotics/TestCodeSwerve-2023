package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.HelixAutoTools.PIDController;
import frc.robot.commands.HelixAutoTools.SwerveTrajectory;
import frc.robot.commands.HelixAutoTools.SwerveTrajectory.State;
import frc.robot.commands.HelixAutoTools.Paths.Path;
import frc.robot.Subsystems.Constant.AutoConstants;
import frc.robot.Subsystems.DriveTrainSubsystemRick;

public class GoToAPlace extends CommandBase {
  private DriveTrainSubsystemRick drive;
  private PIDController xController, yController, thetaController;
  private double lastTime = 0;
  private Timer timer = new Timer();
  private Pose2d initPose;
  private Pose2d targetPose;

  public GoToAPlace(DriveTrainSubsystemRick drive, Pose2d target) {
    addRequirements(drive);
    this.drive = drive;
    this.targetPose = target;
  }

  @Override
  public void initialize() {
    initPose = drive.getPose();
    timer.reset();
    timer.start();
    
    xController = new PIDController(AutoConstants.kPTranslationController, 0, 0);
    yController = new PIDController(AutoConstants.kPTranslationController, 0, 0);
    thetaController = new PIDController(AutoConstants.kPThetaController * 1.5, 0.5, 0);
    thetaController.setContinous(true);
    thetaController.setInputRange(Math.PI * 2);

    lastTime = 0;
  }

  @Override
  public void execute() {
    double time = timer.get();
    double dt = time - lastTime;
    Pose2d currentPose = drive.getPose();

    // xController.setReference(-refState.pose.getX());
    // yController.setReference(-refState.pose.getY());
    thetaController.setReference(-targetPose.getRotation().getRadians());

    // double vx = xController.calculate(-currentPose.getX(), dt) - refState.velocity.x;
    // double vy = yController.calculate(-currentPose.getY(), dt) - refState.velocity.y;
    // double omega = -thetaController.calculate(-currentPose.getRotation().getRadians(), dt) + refState.velocity.z;

    // double vx = -xController.calculate(currentPose.getX(), dt);
    // double vy = -yController.calculate(currentPose.getY(), dt);
    double omega = -thetaController.calculate(-currentPose.getRotation().getRadians(), dt);

    // Very dumb fix, should be x,y
    drive.drive(new Translation2d(0, 0), omega);
    lastTime = time;
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    drive.brake();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2);
  }
}