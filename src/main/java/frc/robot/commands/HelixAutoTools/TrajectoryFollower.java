package frc.robot.commands.HelixAutoTools;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.HelixAutoTools.PIDController;
import frc.robot.commands.HelixAutoTools.SwerveTrajectory;
import frc.robot.commands.HelixAutoTools.SwerveTrajectory.State;
import frc.robot.commands.HelixAutoTools.Paths.Path;
import frc.robot.Subsystems.Constant.AutoConstants;
import frc.robot.Subsystems.DriveTrainSubsystemRick;

public class TrajectoryFollower extends CommandBase {
  private DriveTrainSubsystemRick drive;
  private SwerveTrajectory trajectory;
  private PIDController xController, yController, thetaController;
  private double lastTime = 0;
  private Timer timer = new Timer();

  public TrajectoryFollower(DriveTrainSubsystemRick drive, Path path) {
    addRequirements(drive);
    this.drive = drive;
    this.trajectory = path.getPath();
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    
    xController = new PIDController(AutoConstants.kPTranslationController, 0, 0);
    yController = new PIDController(AutoConstants.kPTranslationController, 0, 0);
    thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    thetaController.setContinous(true);
    thetaController.setInputRange(Math.PI * 2);

    lastTime = 0;
  }

  @Override
  public void execute() {
    double time = timer.get();
    double dt = time - lastTime;
    State refState = trajectory.sample(time);
    Pose2d currentPose = drive.getPose();

    xController.setReference(-refState.pose.getX());
    yController.setReference(-refState.pose.getY());
    thetaController.setReference(refState.pose.getRotation().getRadians());

    // double vx = xController.calculate(-currentPose.getX(), dt) - refState.velocity.x;
    // double vy = yController.calculate(-currentPose.getY(), dt) - refState.velocity.y;
    // double omega = -thetaController.calculate(-currentPose.getRotation().getRadians(), dt) + refState.velocity.z;

    double vx = xController.calculate(currentPose.getX(), dt)-refState.velocity.x;
    double vy = yController.calculate(currentPose.getY(), dt)-refState.velocity.y;
    double omega = -thetaController.calculate(currentPose.getRotation().getRadians(), dt) - refState.velocity.z;

    SmartDashboard.putNumber("AutoTime", dt);
    SmartDashboard.putNumber("vxauto", vx);
    SmartDashboard.putNumber("vyauto", vy);
    SmartDashboard.putNumber("omegaauto", omega);

    drive.drive(new Translation2d(vx, vy), omega);
    lastTime = time;
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    drive.brake();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTime());
  }
}