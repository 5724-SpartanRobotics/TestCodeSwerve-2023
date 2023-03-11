// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import java.io.File;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.SensorTimeBase;

import org.opencv.core.Mat;

import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveTrainInterface;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.DriveTrainSubsystemRick;
import frc.robot.Subsystems.Field;
import frc.robot.commands.ArmControl;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestAutoFull;
import frc.robot.commands.WithoutPark;
import frc.robot.commands.HelixAutoTools.TrajectoriesManager;
import frc.robot.commands.HelixAutoTools.Paths.Path;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.cscore.UsbCamera;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private boolean wasAutoFlag;
  private TrajectoriesManager trajectoriesManager = new TrajectoriesManager(new File(Filesystem.getDeployDirectory(), "trajectories/"));
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private Command m_autoSelected;
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  //private DriveTrainSubsystem drive;
  private DriveTrainSubsystemRick drive;
  private ArmSubsystem arm;

  private Field field = new Field(drive);

  private double dif = 0;
  // Constants for angle setting of swerve
  private double m = -5;
  private double b = 3;
  private double maxturn = 0.5;

  private Joystick drivestick = new Joystick(0);
  private XboxController operator = new XboxController(1);
  private double[] dummyArray = new double[1];

  private Command parkAuto;
  private Command noParkAuto;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    UsbCamera cam2 = CameraServer.startAutomaticCapture();
    System.out.println(Filesystem.getDeployDirectory());
    trajectoriesManager.loadAllTrajectories();
    //if using rick's subsystem uncoment these
    drive = new DriveTrainSubsystemRick();
    drive.setDefaultCommand(new TeleopSwerve(drive, drivestick));

    arm = new ArmSubsystem();
    arm.setDefaultCommand(new ArmControl(arm, operator));

    parkAuto = new TestAutoFull(drive, arm, trajectoriesManager);
    noParkAuto = new WithoutPark(drive, arm, trajectoriesManager);


    dummyArray[0] = -1;
    m_chooser.setDefaultOption("Default Auto", parkAuto);
    m_chooser.addOption("Parkless", noParkAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    SmartDashboard.putNumber("setpos", 0);
    drive.setGyroZero();
    //  drive = new DriveTrainSubsystem();
    //  drive.setDefaultCommand(new RunCommand(() -> {
      
    //  }, drive));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override 
  public void robotPeriodic() {
    // System.out.println("dist: " + SmartDashboard.getNumberArray("aprilTag5", dummyArray)[0]);
    // System.out.println("ang: " + SmartDashboard.getNumberArray("aprilTag5", dummyArray)[1]);
      // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    wasAutoFlag = true;
    SmartDashboard.putNumber("xSpeed", 0);
    SmartDashboard.putNumber("ySpeed", 0);
    SmartDashboard.putNumber("rotationAuto", 0);
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    m_chooser.getSelected().schedule();
    CommandScheduler.getInstance().run();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // field.setTarget(0, 0, 0);
    // field.update();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if(wasAutoFlag) {
      wasAutoFlag = false;
      drive.flipGyro();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    drive.simulationInit();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
// if push input A them move left 