package frc.robot.commands.HelixAutoTools;

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

public class TrajectoryFollower extends CommandBase {
  private DriveTrainSubsystemRick drive;
  private SwerveTrajectory trajectory;
  private PIDController xController, yController, thetaController;
  private double lastTime = 0;
  private Timer timer = new Timer();
  private int inverted = 1;
  private Pose2d initPose;

  public TrajectoryFollower(DriveTrainSubsystemRick drive, Path path, boolean invertedState) {
    addRequirements(drive);
    this.drive = drive;
    this.trajectory = path.getPath();
    if(invertedState) {
      this.inverted = -1;
    } else {
      this.inverted = 1;
    }
  }

  @Override
  public void initialize() {
    initPose = trajectory.getInitialPose();
    //System.out.println("Init :)");
    drive.ZeroDriveSensors(new Pose2d(new Translation2d(initPose.getX() * inverted, initPose.getY() * inverted), initPose.getRotation()));
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

    xController.setReference(inverted * -refState.pose.getX());
    yController.setReference(inverted * -refState.pose.getY());
    thetaController.setReference(-refState.pose.getRotation().getRadians());

    // double vx = xController.calculate(-currentPose.getX(), dt) - refState.velocity.x;
    // double vy = yController.calculate(-currentPose.getY(), dt) - refState.velocity.y;
    // double omega = -thetaController.calculate(-currentPose.getRotation().getRadians(), dt) + refState.velocity.z;

    SmartDashboard.putNumber("Xpos", -currentPose.getX());
    SmartDashboard.putNumber("Ypos", -currentPose.getY());
    SmartDashboard.putNumber("Zpos", -currentPose.getRotation().getRadians());
    SmartDashboard.putNumber("Xref", refState.pose.getX());
    SmartDashboard.putNumber("Yref", refState.pose.getY());
    SmartDashboard.putNumber("Zref", refState.pose.getRotation().getRadians());

    double vx = -xController.calculate(currentPose.getX(), dt)+refState.velocity.x * inverted;
    double vy = -yController.calculate(currentPose.getY(), dt)+refState.velocity.y * inverted;
    double omega = -thetaController.calculate(-currentPose.getRotation().getRadians(), dt)+ refState.velocity.z;

    SmartDashboard.putNumber("AutoTime", dt);
    SmartDashboard.putNumber("vxauto", vx);
    SmartDashboard.putNumber("vyauto", vy);
    SmartDashboard.putNumber("omegaauto", omega);

    // Very dumb fix, should be x,y
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