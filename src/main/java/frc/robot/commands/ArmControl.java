package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.Constant.ArmConstants;

public class ArmControl extends CommandBase {
    private XboxController controller;
    private ArmSubsystem arm;
    boolean povUnpressed = true;
    boolean bumpersUnpressed = true;

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
        if(controller.getLeftBumper() && bumpersUnpressed) {
            //retract incremental
            bumpersUnpressed = false;
            arm.extendIncremental(true);
        } else if(controller.getRightBumper() && bumpersUnpressed) {
            //extend incremental
            bumpersUnpressed = false;
            arm.extendIncremental(false);
        } else if (controller.getXButton()) {
            //Set extend position to front pole
            arm.frontExtendPos();
        } else if (controller.getYButton()) {
            arm.extendIntakePos();
        } else if (-controller.getRightY() > 0.5){
            //go to full out position
            arm.extendFullOut();
        }
        else if (-controller.getRightY() < -0.5){
            arm.extendFullIn();
        }

        if (!controller.getLeftBumper() && !controller.getRightBumper())
            bumpersUnpressed = true;

        //getPOV returns an angle for the POV control. Upper button is 0 and the degrees are positive 
        // for the clockwise direction. It has a value of -1 for nothing pressed.
        int pov = controller.getPOV();
        if(pov > 140 && pov < 220 && povUnpressed) {
            //worm down incremental
            povUnpressed = false;
            arm.wormIncremental(false, false);
        } else if(((pov > 330 && pov <= 360) || (pov < 40 && pov >= 0)) && povUnpressed) {
            //worm up incremental
            povUnpressed = false;
            arm.wormIncremental(true, false);
        } else if (pov > 50 && pov < 130 && povUnpressed){
            povUnpressed = false;
            arm.wormIncremental(true, true);
        } else if (pov > 230 && pov < 310 && povUnpressed){
            povUnpressed = false;
            arm.wormIncremental(false, true);
        }
        if (pov == -1) {
            povUnpressed = true;
        }
        if (controller.getAButton()) {
            //Set hoist position to front pole height
            arm.frontHoistPos();
        } else if (controller.getBButton()) {
            arm.wormIntakePos();
        } else if (-controller.getLeftY() > 0.5) {
            arm.wormFullUp();
        }
        else if (-controller.getLeftY() < -0.5){
            arm.wormFullDown();
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
