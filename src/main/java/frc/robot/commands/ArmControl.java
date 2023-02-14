package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveTrainInterface;
import frc.robot.Subsystems.Constant.ControllerConstants;
import frc.robot.Subsystems.Constant.DriveConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmControl extends CommandBase {
    private XboxController controller;
    private ArmSubsystem arm;

    /**
     * Drive Controller
     * @param arm The whole arm system
     * @param controller Nicks controller
     */
    public ArmControl(ArmSubsystem arm, XboxController controller){
        this.controller = controller;
        this.arm = arm;
        addRequirements((SubsystemBase)arm);
 }

    @Override
    public void execute(){
        arm.zoop(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());
        if(controller.getAButtonPressed()) {
            arm.setArmPosition(0);
        } else if (controller.getBButtonPressed()) {
            arm.setArmPosition(1);
        } else if (controller.getYButtonPressed()) {
            arm.setArmPosition(2);
        }
    }
}
