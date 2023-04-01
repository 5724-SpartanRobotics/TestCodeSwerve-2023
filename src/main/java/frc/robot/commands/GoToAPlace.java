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
  private int inverted = 1;
  private Pose2d refState;

  public GoToAPlace(DriveTrainSubsystemRick drive, Pose2d refState, boolean invertedState) {
    addRequirements(drive);
    this.drive = drive;
    this.refState = refState;
    if(invertedState) {
      this.inverted = -1;
    } else {
      this.inverted = 1;
    }
  }

  @Override
  public void initialize() {
    
    xController = new PIDController(AutoConstants.kPTranslationController / 30, 0, 0);
    yController = new PIDController(AutoConstants.kPTranslationController / 30, 0, 0);
    thetaController = new PIDController(AutoConstants.kPThetaController / 30, 0, 0);
    thetaController.setContinous(true);
    thetaController.setInputRange(Math.PI * 2);

    lastTime = 0;
  }

  @Override
  public void execute() {
    double time = timer.get();
    double dt = time - lastTime;
    Pose2d currentPose = drive.getPose();

    xController.setReference(refState.getX());
    yController.setReference(inverted * refState.getY());
    thetaController.setReference(-refState.getRotation().getRadians() * inverted);

    // double vx = xController.calculate(-currentPose.getX(), dt) - refState.velocity.x;
    // double vy = yController.calculate(-currentPose.getY(), dt) - refState.velocity.y;
    // double omega = -thetaController.calculate(-currentPose.getRotation().getRadians(), dt) + refState.velocity.z;

    SmartDashboard.putNumber("Xpos", -currentPose.getX());
    SmartDashboard.putNumber("Ypos", -currentPose.getY());
    SmartDashboard.putNumber("Zpos", -currentPose.getRotation().getRadians());

    double vx = -xController.calculate(-currentPose.getX(), dt);
    double vy = -yController.calculate(-currentPose.getY(), dt);
    double omega = -thetaController.calculate(-currentPose.getRotation().getRadians(), dt);

    // SmartDashboard.putNumber("AutoTime", dt);
    // SmartDashboard.putNumber("vxauto", vx);
    // SmartDashboard.putNumber("vyauto", vy);
    // SmartDashboard.putNumber("omegaauto", omega);

    drive.drive(new Translation2d(-vx, -vy), omega);
    lastTime = time;
    if(isFinished()) {
      end(true);
    }
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