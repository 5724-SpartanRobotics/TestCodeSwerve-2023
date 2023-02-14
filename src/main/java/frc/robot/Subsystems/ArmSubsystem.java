package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Constant.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax claw = new CANSparkMax(ArmConstants.ClawMotor, MotorType.kBrushless);
    private CANSparkMax worm = new CANSparkMax(ArmConstants.WormMotor, MotorType.kBrushless);
    private SparkMaxPIDController wormControl = worm.getPIDController();
    private CANSparkMax extend = new CANSparkMax(ArmConstants.ExtendMotor, MotorType.kBrushless);
    private SparkMaxPIDController extendControl = extend.getPIDController();

    public ArmSubsystem() {

    }

    public void zoop(double speed) {
        claw.set(speed * 0.25);
    }

    public void setArmPosition(int pos) {
        wormControl.setReference(ArmConstants.WormPositions[pos], ControlType.kSmartMotion);
        extendControl.setReference(ArmConstants.ExtendPositions[pos], ControlType.kSmartMotion);
    }
}
