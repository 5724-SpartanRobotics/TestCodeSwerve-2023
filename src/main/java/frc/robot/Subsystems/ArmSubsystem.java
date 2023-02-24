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
    private SparkMaxPIDController clawPidController = claw.getPIDController();
    private CANSparkMax worm = new CANSparkMax(ArmConstants.WormMotor, MotorType.kBrushless);
    private SparkMaxPIDController wormPidControl = worm.getPIDController();
    private CANSparkMax extend = new CANSparkMax(ArmConstants.ExtendMotor, MotorType.kBrushless);
    private SparkMaxPIDController extendPidControl = extend.getPIDController();
    private RelativeEncoder wormEncoder = worm.getEncoder();
    private RelativeEncoder extendEncoder = extend.getEncoder();
    double extendPosRef = 0;
    double wormPosRef = 0;
    Boolean wormFreezeSet = false;
    Boolean extendFreezeSet = false;
    double extendFixedArmInEncoderCounts = 34 * ArmConstants.ExtendMotorRotationsPerInch;
    double extendMotorCountsFor42InchHeight = 42 * ArmConstants.ExtendMotorRotationsPerInch;

    Boolean tunePidMode = false;//set to true to have position refs and gains set from smart dashboard.
    //The reference to the PID is in motor rotations, but all the gains and feed forward are normalized
    // to 1 = max, -1 = min
    double worm_kP = 0.00005;
    double worm_kI = 0.0;
    double worm_kD = 0.0;
    double worm_kFF = 0.001;

    double extend_kP = 0.00005;
    double extend_kI = 0.0;
    double extend_kD = 0.0;
    double extend_kFF = -0.001;

    double wormMaxVel = 1800; // rpm  
    double wormMaxAcc = 3600; //rpm/sec
    double extendMaxVel = 1800; // rpm  
    double extendMaxAcc = 3600; //rpm/sec



    public ArmSubsystem() {
        claw.restoreFactoryDefaults();
        worm.restoreFactoryDefaults();
        extend.restoreFactoryDefaults();
        extend.setIdleMode(IdleMode.kBrake);
        worm.setIdleMode(IdleMode.kBrake);
        claw.setIdleMode(IdleMode.kBrake);
        worm.setInverted(true);

        SetPidGainsForWormAndExtend();

        //The claw motor uses smart motion in velocity mode to control speed
        clawPidController.setP(0.0005);
        clawPidController.setSmartMotionMaxVelocity(5000, 0);
        clawPidController.setSmartMotionMaxAccel(5000, 0);
        claw.setSmartCurrentLimit(10, 30);

        if (tunePidMode)
            PutTuneValuesToSmartDashboard();
    }

    private void SetPidGainsForWormAndExtend() {
        wormPidControl.setP(worm_kP);
        wormPidControl.setD(worm_kD);
        wormPidControl.setI(worm_kI);
        wormPidControl.setFF(worm_kFF);
        wormPidControl.setSmartMotionMaxVelocity(wormMaxVel, 0);
        wormPidControl.setSmartMotionMaxAccel(wormMaxAcc, 0);

        extendPidControl.setP(extend_kP);
        extendPidControl.setD(extend_kD);
        extendPidControl.setI(extend_kI);
        extendPidControl.setFF(extend_kFF);
        extendPidControl.setSmartMotionMaxVelocity(extendMaxVel, 0);
        extendPidControl.setSmartMotionMaxAccel(extendMaxAcc, 0);
    }

    /**
     * speed reference to the claw
     * @param speed
     */
    public void zoop(double speed) {
        clawPidController.setReference(speed  * ArmConstants.ClawMaxPercent, ControlType.kVelocity);
    }


    @Override
    public void periodic() {
        if (DebugSetting.TraceLevel == DebugLevel.Arm || DebugSetting.TraceLevel == DebugLevel.All) {
            SmartDashboard.putNumber("WormPosFbk", wormEncoder.getPosition());
            SmartDashboard.putNumber("ExtendPosFbk", extendEncoder.getPosition());
            SmartDashboard.putNumber("ExtendPosRef", extendPosRef);
            SmartDashboard.putNumber("WormPosRef", wormPosRef);
        }

        if (tunePidMode)
        {
            ReadTuningRefsAndGainsFromSmartDashboard();
        }
        //The teleop and autonomus commands set up the references
        // send the ref to the PID controllers.
        // The reference will be linited in the send methods.
        sendExtendPosToPidController();
        sendWormRefToPidController();
    }

    private void PutTuneValuesToSmartDashboard()
    {
        SmartDashboard.putNumber("wormKP", worm_kP);
        SmartDashboard.putNumber("wormKI", worm_kI);
        SmartDashboard.putNumber("wormFF", worm_kFF);
        SmartDashboard.putNumber("wormMaxVel", wormMaxVel);
        SmartDashboard.putNumber("wormMaxAcc", wormMaxAcc);
        SmartDashboard.putNumber("extendKP", extend_kP);
        SmartDashboard.putNumber("extendKI", extend_kI);
        SmartDashboard.putNumber("extendFF", extend_kFF);
        SmartDashboard.putNumber("extendMaxVel", extendMaxVel);
        SmartDashboard.putNumber("extendMaxAcc", extendMaxAcc);
        SmartDashboard.putNumber("WormTunePosRef", wormPosRef);
        SmartDashboard.putNumber("ExtendTunePosRef", extendPosRef);
    }
    private void ReadTuningRefsAndGainsFromSmartDashboard() {
        double wormkP = SmartDashboard.getNumber("wormKP", worm_kP);
        double wormkI = SmartDashboard.getNumber("wormKI", worm_kI);
        double wormkFF = SmartDashboard.getNumber("wormFF", worm_kFF);
        double wormMVel = SmartDashboard.getNumber("wormMaxVel", wormMaxVel);
        double wormMAcc = SmartDashboard.getNumber("wormMaxAcc", wormMaxAcc);
        double extendkP = SmartDashboard.getNumber("extendKP", extend_kP);
        double extendkI = SmartDashboard.getNumber("extendKI", extend_kI);
        double extendkFF = SmartDashboard.getNumber("extendFF", extend_kFF);
        double extendMVel = SmartDashboard.getNumber("extendMaxVel", extendMaxVel);
        double extendMAcc = SmartDashboard.getNumber("extendMaxAcc", extendMaxAcc);

        SmartDashboard.putNumber("Worm IA", worm.getOutputCurrent());
        SmartDashboard.putNumber("Worm Spd", wormEncoder.getVelocity());
        SmartDashboard.putNumber("Extend IA", extend.getOutputCurrent());
        SmartDashboard.putNumber("Extend Spd", extendEncoder.getVelocity());

        //values in motor turns
        wormPosRef = SmartDashboard.getNumber("WormTunePosRef", wormPosRef);
        extendPosRef = SmartDashboard.getNumber("ExtendTunePosRef", extendPosRef);

        //set the values if they are different
        Boolean diff = false;
        if (wormkP != worm_kP){
            worm_kP = wormkP;
            diff = true;
        }
        if (wormkI != worm_kI){
            worm_kI = wormkI;
            diff = true;
        }
        if (wormkFF != worm_kFF){
            worm_kFF = wormkFF;
            diff = true;
        }
        if (wormMVel != wormMaxVel){
            wormMaxVel = wormMVel;
            diff = true;
        }
        if (wormMAcc != wormMaxAcc){
            wormMaxAcc = wormMAcc;
            diff = true;
        }
        if (extendkP != extend_kP){
            extend_kP = extendkP;
            diff = true;
        }
        if (extendkI != extend_kI){
            worm_kI = extendkI;
            diff = true;
        }
        if (extendkFF != extend_kFF){
            extend_kFF = extendkFF;
            diff = true;
        }
        if (extendMVel != extendMaxVel){
            extendMaxVel = extendMVel;
            diff = true;
        }
        if (extendMAcc != extendMaxAcc){
            extendMaxAcc = extendMAcc;
            diff = true;
        }
        if (diff)
            SetPidGainsForWormAndExtend();
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
        extendPidControl.setReference(extendPosRef, ControlType.kSmartMotion);
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
        wormPidControl.setReference(wormPosRef, ControlType.kSmartMotion);
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
