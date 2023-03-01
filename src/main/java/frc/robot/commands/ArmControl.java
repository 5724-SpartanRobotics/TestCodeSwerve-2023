package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.Constant.ArmConstants;

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
        if (!RobotState.isTeleop())
           return;
        if(controller.getLeftBumper()) {
            //jog retract
            arm.driveExtension(-1);
        } else if(controller.getRightBumper()) {
            //jog extend
            arm.driveExtension(1);
        } else if (controller.getXButton()) {
            //Set extend position to front pole
            arm.frontExtendPos();
        } else if (controller.getBButton()) {
            // extend at floor height of cone
            arm.cubeFloorPos();
        } else if (controller.getAButton()) {
            arm.coneFloorPos();
        } else {
            //stop jog
            arm.driveExtension(0);
        }
        //getPOV returns an angle for the POV control. Upper button is 0 and the degrees are positive 
        // for the clockwise direction. It has a value of -1 for nothing pressed.
        if(controller.getPOV() == 180) {
            //jog worm down
            arm.driveRotation(-1);
        } else if(controller.getPOV() == 0) {
            //jog worm up
            arm.driveRotation(1);
        } else if (controller.getYButton()) {
            //Set hoist position to front pole height
            arm.frontHoistPos();
        } else {
            //stop jog. 
            arm.driveRotation(0);
        }
        //set the claw motor speed
        arm.zoop((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * ArmConstants.ClawMaxPercent * 6000);

        if (controller.getStartButtonPressed()) {
            arm.useConeCurrentLimit();
        }
        else if (controller.getBackButtonPressed()) {
            arm.useCubeCurrentLimit();
        }
    }
}
