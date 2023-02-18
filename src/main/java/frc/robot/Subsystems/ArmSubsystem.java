package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Constant.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax claw = new CANSparkMax(ArmConstants.ClawMotor, MotorType.kBrushless);
    private CANSparkMax worm = new CANSparkMax(ArmConstants.WormMotor, MotorType.kBrushless);
    private SparkMaxPIDController wormControl = worm.getPIDController();
    private CANSparkMax extend = new CANSparkMax(ArmConstants.ExtendMotor, MotorType.kBrushless);
    private SparkMaxPIDController extendControl = extend.getPIDController();
    private RelativeEncoder wormEncoder = worm.getEncoder();
    private RelativeEncoder extendEncoder = extend.getEncoder();
    int smartMotionSlot = 0;
    double maxVel = 1800; // rpm  
    double maxAcc = 3600; //rpm/sec

    public ArmSubsystem() {
        claw.restoreFactoryDefaults();
        worm.restoreFactoryDefaults();
        extend.restoreFactoryDefaults();
        wormControl.setP(0.00005);
        wormControl.setD(0);
        wormControl.setFF(0.00025);
        wormControl.setOutputRange(-1, 1);
        wormControl.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        wormControl.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        extendControl.setP(0.00005);
        extendControl.setD(0);
        extendControl.setFF(0.00025);
        extendControl.setOutputRange(-1, 1);
        extendControl.setSmartMotionMaxVelocity(maxVel * 2, smartMotionSlot);
        extendControl.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("WormPos", wormEncoder.getPosition());
        SmartDashboard.putNumber("Extend", extendEncoder.getPosition());
    }

    public void zoop(double speed) {
        claw.set(speed * 0.25);
    }

    public void driveRotation(double speed) {
        if((wormEncoder.getPosition() > ArmConstants.WormPositions[0] && speed > 0) || (wormEncoder.getPosition() < ArmConstants.WormPositions[1] && speed < 0)) {
            worm.set(0);
            System.out.println("AAAAAA");
        } else {
            worm.set(speed * 0.25);
            System.out.println("Moving");
        }
    }
    
    public void driveExtension(double speed) {
        if((extendEncoder.getPosition() > ArmConstants.ExtendPositions[1] && speed > 0) || (extendEncoder.getPosition() < ArmConstants.ExtendPositions[0] && speed < 0)) {
            extend.set(0);
        } else {
            extend.set(speed * 0.5);
        }        
    }
}
