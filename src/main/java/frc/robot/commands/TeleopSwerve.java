package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.DriveTrainInterface;
import frc.robot.Subsystems.DriveTrainSubsystemRick;
import frc.robot.Subsystems.Constant.ControllerConstants;
import frc.robot.Subsystems.Constant.DebugLevel;
import frc.robot.Subsystems.Constant.DebugSetting;
import frc.robot.Subsystems.Constant.DriveConstants;

public class TeleopSwerve extends CommandBase {
    private Joystick controller;
    private DriveTrainSubsystemRick swerveDrive;

    /**
     * Drive Controller
     * @param swerveDrive The drive train subsystem
     * @param controller A joystick
     */
    public TeleopSwerve(DriveTrainSubsystemRick swerveDrive, Joystick controller){
        this.controller = controller;
        this.swerveDrive = swerveDrive;
        addRequirements((SubsystemBase)swerveDrive);
        if (TimedRobot.isSimulation() && DebugSetting.TraceLevel == DebugLevel.All)
        {
            SmartDashboard.putNumber("JoyStickY", 0.0);
            SmartDashboard.putNumber("JoyStickX", 0.0);
            SmartDashboard.putNumber("JoyStickZ", 0.0);
        }
 }

    @Override
    public void execute(){
        if (DriverStation.isAutonomous()){
            return;
        }
        double xAxis;
        double yAxis;
        double zAxis;
        double speedMod = 1;
        if(controller.getRawButton(11)){
            speedMod = 0.25;
        }
        else if (controller.getRawButton(9)) {
            speedMod = 0.65;
        }
        // This chunk of code locks certain joystick directions if buttons are pressed
        if(controller.getRawButton(5)) {
            yAxis = 0;
        } else {
            yAxis = -controller.getY();
        } 

        if(controller.getRawButton(3)) {
            xAxis = 0;
        } else {
            xAxis = -controller.getX();
            
        }
        if(controller.getRawButton(2)) {
            zAxis = 0;
        } else {
            zAxis = -controller.getZ() / 2;
        }
        if(controller.getRawButton(7)) {
            swerveDrive.setGyroZero();
        }
        // Power Array Auto Align Code
        // Conditional is a check for having a combination of buttons pressed
        if((controller.getRawButton(7) || controller.getRawButton(9) || 
        controller.getRawButton(11)) && (controller.getRawButton(8) || 
        controller.getRawButton(10) || controller.getRawButton(12))) {
            
        }

        
        
        yAxis = (Math.abs(yAxis) < ControllerConstants.joystickDeadband) ? 0 : yAxis * speedMod;
        xAxis = (Math.abs(xAxis) < ControllerConstants.joystickDeadband) ? 0 : xAxis * speedMod;
        zAxis = (Math.abs(zAxis) < ControllerConstants.joystickDeadband) ? 0 : zAxis * speedMod;

        double rotation = zAxis * DriveConstants.maxAngularVelocityRps;
        if (DebugSetting.TraceLevel == DebugLevel.Swerve) {
            SmartDashboard.putNumber("ControllerRotation", rotation);
            SmartDashboard.putNumber("ControllerX", xAxis);
            SmartDashboard.putNumber("ControllerY", yAxis);
        }
        Translation2d translation = new Translation2d(yAxis, xAxis).times(DriveConstants.maxRobotSpeedmps);
        swerveDrive.drive(translation, rotation);
    }
}