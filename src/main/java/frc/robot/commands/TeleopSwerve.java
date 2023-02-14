package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.DriveTrainInterface;
import frc.robot.Subsystems.Constant.ControllerConstants;
import frc.robot.Subsystems.Constant.DriveConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TeleopSwerve extends CommandBase {
    private Joystick controller;
    private DriveTrainInterface swerveDrive;

    /**
     * Drive Controller
     * @param swerveDrive The drive train subsystem
     * @param controller A joystick
     */
    public TeleopSwerve(DriveTrainInterface swerveDrive, Joystick controller){
        this.controller = controller;
        this.swerveDrive = swerveDrive;
        addRequirements((SubsystemBase)swerveDrive);
        if (TimedRobot.isSimulation())
        {
            SmartDashboard.putNumber("JoyStickY", 0.0);
            SmartDashboard.putNumber("JoyStickX", 0.0);
            SmartDashboard.putNumber("JoyStickZ", 0.0);
        }
 }

    @Override
    public void execute(){
        double xAxis;
        double yAxis;
        // This chunk of code locks certain joystick directions if buttons are pressed
        if(controller.getRawButton(5)) {
            yAxis = 0;
        } else {
            yAxis = controller.getX();
        } 

        if(controller.getRawButton(3)) {
            xAxis = 0;
        } else {
            xAxis = -controller.getY();
        }
        double zAxis = -controller.getZ() / 2;

        // Power Array Auto Align Code
        // Conditional is a check for having a combination of buttons pressed
        if((controller.getRawButton(7) || controller.getRawButton(9) || 
        controller.getRawButton(11)) && (controller.getRawButton(8) || 
        controller.getRawButton(10) || controller.getRawButton(12))) {
            
        }

        
        
        // if (TimedRobot.isSimulation())
        // {
        //     yAxis = SmartDashboard.getNumber("JoyStickY", 0.0);
        //     xAxis = SmartDashboard.getNumber("JoyStickX", 0.0);
        //     zAxis = SmartDashboard.getNumber("JoyStickZ", 0.0);
        // }
        yAxis = (Math.abs(yAxis) < ControllerConstants.joystickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < ControllerConstants.joystickDeadband) ? 0 : xAxis;
        zAxis = (Math.abs(zAxis) < ControllerConstants.joystickDeadband) ? 0 : zAxis;

        double rotation = zAxis * DriveConstants.maxAngularVelocityRps;
        Translation2d translation = new Translation2d(yAxis, xAxis).times(DriveConstants.maxRobotSpeedmps);
        swerveDrive.drive(translation, rotation);
    }
}