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
            //retract incremental
            arm.extendIncremental(true);
        } else if(controller.getRightBumper()) {
            //extend incremental
            arm.extendIncremental(false);
        } else if (controller.getXButton()) {
            //Set extend position to front pole
            arm.frontExtendPos();
        } else if (controller.getYButton()) {
            arm.extendIntakePos();
        } else if (controller.getRightY() > 0.5){
            //go to full out position
            arm.extendFullOut();
        }
        else if (controller.getRightY() < -0.5){
            arm.extendFullIn();
        }
        //getPOV returns an angle for the POV control. Upper button is 0 and the degrees are positive 
        // for the clockwise direction. It has a value of -1 for nothing pressed.
        if(controller.getPOV() == 180) {
            //worm down incremental
            arm.wormIncremental(false, false);
        } else if(controller.getPOV() == 0) {
            //worm up incremental
            arm.wormIncremental(true, false);
        } else if (controller.getPOV() == 90){
            arm.wormIncremental(true, true);
        } else if (controller.getPOV() == 270){
            arm.wormIncremental(false, true);
        } else if (controller.getAButton()) {
            //Set hoist position to front pole height
            arm.frontHoistPos();
        } else if (controller.getBButton()) {
            arm.wormIntakePos();
        } else if (controller.getLeftY() > 0.5) {
            arm.wormFullUp();
        }
        else if (controller.getLeftY() < -0.5){
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
