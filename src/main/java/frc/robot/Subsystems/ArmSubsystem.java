package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Constant.ArmConstants;
import frc.robot.Subsystems.Constant.DebugLevel;
import frc.robot.Subsystems.Constant.DebugSetting;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax claw = new CANSparkMax(ArmConstants.ClawMotor, MotorType.kBrushless);
    private CANSparkMax worm = new CANSparkMax(ArmConstants.WormMotor, MotorType.kBrushless);
    private SparkMaxPIDController wormPidControl = worm.getPIDController();
    private CANSparkMax extend = new CANSparkMax(ArmConstants.ExtendMotor, MotorType.kBrushless);
    private SparkMaxPIDController extendPidControl = extend.getPIDController();
    private RelativeEncoder wormEncoder = worm.getEncoder();
    private RelativeEncoder extendEncoder = extend.getEncoder();
    int smartMotionSlot = 0;
    double maxVel = 1800; // rpm  
    double maxAcc = 3600; //rpm/sec
    double extendPosRef = 0;
    double wormPosRef = 0;
    double wormPosClampped;
    double extendPosClamped;
    Boolean wormFreezeSet = false;
    Boolean extendFreezeSet = false;
    double extendMaxLimit;
    double wormMinLimit;
    double extendFixedArmInEncoderCounts = 34 * ArmConstants.ExtendMotorRotationsPerInch;
    double extendMotorCountsFor42InchHeight = 42 * ArmConstants.ExtendMotorRotationsPerInch;

    public ArmSubsystem() {
        claw.restoreFactoryDefaults();
        worm.restoreFactoryDefaults();
        extend.restoreFactoryDefaults();
        extend.setIdleMode(IdleMode.kBrake);
        worm.setIdleMode(IdleMode.kBrake);
        claw.setIdleMode(IdleMode.kBrake);
        worm.setInverted(true);
        wormPidControl.setP(0.00005);
        wormPidControl.setD(0);
        wormPidControl.setI(0);
        wormPidControl.setFF(0);
        wormPidControl.setOutputRange(-1, 1);
        wormPidControl.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        wormPidControl.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);

        extendPidControl.setP(0.00005);
        extendPidControl.setD(0);
        extendPidControl.setI(0);
        extendPidControl.setFF(0.0);
        extendPidControl.setOutputRange(-1, 1);
        extendPidControl.setSmartMotionMaxVelocity(maxVel * 2, smartMotionSlot);
        extendPidControl.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);

        setExtendLimitToDefault();
        setWormLimitToDefault();
    }

    public void zoop(double speed) {
        claw.set(speed  * ArmConstants.ClawMaxPercent);
    }

    private void setExtendLimitToDefault()
    {
        extendMaxLimit = ArmConstants.ExtendPositionMax * ArmConstants.ExtendMotorRotationsPerInch;
    }

    private void setWormLimitToDefault()
    {
        wormMinLimit = ArmConstants.WormPositionMin * ArmConstants.WormMotorRotationsPerInch;
    }

    private void setExtendLimitToCurrent()
    {
        extendMaxLimit = extendEncoder.getPosition();
    }

    private void setWormLimitToCurrent()
    {
        wormMinLimit = wormEncoder.getPosition();
    }

    @Override
    public void periodic() {
        if (DebugSetting.TraceLevel == DebugLevel.Arm || DebugSetting.TraceLevel == DebugLevel.All) {
            SmartDashboard.putNumber("WormPosFbk", wormEncoder.getPosition());
            SmartDashboard.putNumber("ExtendPosFbk", extendEncoder.getPosition());
            SmartDashboard.putNumber("ExtendPosRef", extendPosRef);
            SmartDashboard.putNumber("WormPosRef", wormPosRef);
            SmartDashboard.putNumber("ExtendPosClamped", extendPosClamped);
            SmartDashboard.putNumber("WormPosClamped", wormPosClampped);
            SmartDashboard.putNumber("ExtendMaxLimit", extendMaxLimit);
            SmartDashboard.putNumber("WormMinLimit", wormMinLimit);
        }
        //set extend and rotate limits based on position feedbacks
        // if the total extend is greater than the current room to the floor, freeze the min worm limit
        // if (extendEncoder.getPosition() + extendFixedArmInEncoderCounts >
        //  extendMotorCountsFor42InchHeight + (wormEncoder.getPosition() * ArmConstants.ExtendMotorRotationsPerInch / ArmConstants.WormMotorRotationsPerInch)) {
        //     setWormLimitToCurrent();
        //     setExtendLimitToCurrent();
        // }
        // else {
        //     setExtendLimitToDefault();
        //     setWormLimitToDefault();
        // }
        //The teleop and autonomus commands set up the references
        // send the ref to the PID controllers.
        // The reference will be linited in the send methods.
        sendExtendPosToPidController();
        sendWormRefToPidController();
    }

     /**
     * -1 = Jog worm reverse, 1 = forward
     * 0 = freeze the worm the first time after a forward or reverse
     * @param speed
     */
    public void driveRotation(double speed) {
        if (speed > 0)
        {
            wormPosRef = ArmConstants.WormPositionMax * ArmConstants.WormMotorRotationsPerInch;
            wormFreezeSet = false;
        }
        else if (speed < 0)
        {
            wormPosRef = ArmConstants.WormPositionMin * ArmConstants.WormMotorRotationsPerInch;
            wormFreezeSet = false;
        }
        else if (!wormFreezeSet)//hold the last position
        {
            wormPosRef = wormEncoder.getPosition();
            wormFreezeSet = true;
        }
    }

    private void sendExtendPosToPidController() {
        if (extendPosRef > extendMaxLimit)
            extendPosClamped = extendMaxLimit;
        else
            extendPosClamped = extendPosRef;
        extendPidControl.setReference(extendPosClamped, ControlType.kSmartMotion);
    }
    /**
     * -1 = Jog telescoping reverse, 1 = forward
     * 0 = freeze the telescoping the first time after a forward or reverse
     * @param speed
     */
    public void driveExtension(double speed) {
        if (speed > 0)
        {
            extendPosRef = ArmConstants.ExtendPositionMax * ArmConstants.ExtendMotorRotationsPerInch;
            extendFreezeSet = false;
        }
        else if (speed < 0)
        {
            extendPosRef = ArmConstants.ExtendPositionMin * ArmConstants.ExtendMotorRotationsPerInch;
            extendFreezeSet = false;
        }
        else if (!wormFreezeSet)//hold the last position
        {
            extendPosRef = extendEncoder.getPosition();
            extendFreezeSet = true;
        }
    }

    private void sendWormRefToPidController() {
        if (wormPosRef < wormMinLimit)
            wormPosClampped = wormMinLimit;
        else
            wormPosClampped = wormPosRef;
        wormPidControl.setReference(wormPosClampped, ControlType.kSmartMotion);
    }

    /**
     * Set the hoist positions to reach the front cone post
     */
    public void frontHoistPos() {
        wormPosRef = ArmConstants.WormPositionFrontCone * ArmConstants.WormMotorRotationsPerInch;
    }

    /**
     * Set the position to reach the front cone post
     */
    public void frontExtendPos() {
        extendPosRef = ArmConstants.ExtendPositionFrontCone * ArmConstants.ExtendMotorRotationsPerInch;
    }

    /**
     * Set the arm inside the robot with the claw at cube pickup height
     */
    public void cubeFloorPos() {
        extendPosRef = ArmConstants.ExtendPositionFloorCube * ArmConstants.ExtendMotorRotationsPerInch;
    }

    /**
     * Set the arm inside the robot with the claw at cone pickup height.
     */
    public void tuckInExtendPos() {
        extendPosRef = ArmConstants.ExtendPositionFloorCone * ArmConstants.ExtendMotorRotationsPerInch;
    }
}
