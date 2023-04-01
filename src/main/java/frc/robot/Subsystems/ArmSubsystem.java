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
    private RelativeEncoder clawEncoder = claw.getEncoder();
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
    double worm_kP = 0.0003;
    double worm_kI = 0.000000007;
    double worm_kD = 0.0;
    double worm_kFF = 0.00;
    double wormMaxVel = 5200; // rpm  
    double wormMaxAcc = 4000; //rpm/sec
    //max motor speed is 5676 RPM. This number gets multiplied by current speed
    // and added to the current position to have a softer stop. The first number
    // in the equation is a distance in inches that will be added / subtracted at max speed.
    double wormDistMult = 3 * ArmConstants.WormMotorRotationsPerInch / 5676;

    double extend_kP = 0.0004;
    double extend_kI = 0.00000007;
    double extend_kD = 0.0;
    double extend_kFF = 0.00;
    double extendMaxVel = 4000; // rpm  
    double extendMaxAcc = 4000; //rpm/sec
    double extendDistMult = 6 * ArmConstants.ExtendMotorRotationsPerInch / 5676;

    double claw_kP = 0.0003;
    double claw_kI = 0.000000;
    double claw_kFF = 0.000000;
    double clawMaxVel = 4000; //rpm
    double clawMaxAcc = 2000; //rpm/sec
    double clawSpeedRef = 0;
    double clawSpdRefTune = 0;
    int claw_StallCurLimitAmps = ArmConstants.claw_ConeStallCurLimitAmps;


    
    public ArmSubsystem() {
        claw.restoreFactoryDefaults();
        worm.restoreFactoryDefaults();
        extend.restoreFactoryDefaults();
        extend.setIdleMode(IdleMode.kBrake);
        worm.setIdleMode(IdleMode.kBrake);
        claw.setIdleMode(IdleMode.kCoast);
        worm.setInverted(true);

        //set the extend and worm freeze booleans so that drift after powerup and before teleop do not cause the current position to be the reference.
        // In other words keep the zero reference.
        // The FreezeSet bits are used to freeze the position reference to match the current feedback when a jog request is removed.
        extendFreezeSet = true;
        wormFreezeSet = true;

        SmartDashboard.putBoolean("ClawCubeForce", claw_StallCurLimitAmps == ArmConstants.claw_ConeStallCurLimitAmps);

        SetPidGainsForWormExtendClaw();

        claw.setSmartCurrentLimit(claw_StallCurLimitAmps, 50);

        if (tunePidMode)
            PutTuneValuesToSmartDashboard(false, false, true);
    }

    private void SetPidGainsForWormExtendClaw() {
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

        //The claw motor uses velocity mode to control speed
        clawPidController.setP(claw_kP);
        clawPidController.setD(0);
        clawPidController.setI(claw_kI);
        clawPidController.setFF(claw_kFF);
        clawPidController.setIZone(0);
        clawPidController.setOutputRange(-1, 1);
 //       clawPidController.setSmartMotionMaxVelocity(clawMaxVel, 0);
 //       clawPidController.setSmartMotionMaxAccel(clawMaxAcc, 0);
    }

    /**
     * speed reference to the claw
     * @param speed
     */
    public void zoop(double speed) {
        // System.out.println(speed);
        if (!tunePidMode)
            clawPidController.setReference(speed, ControlType.kVelocity);  //* ArmConstants.ClawMaxPercent, ControlType.kVelocity);
    }

    public void useConeCurrentLimit(){
        claw_StallCurLimitAmps = ArmConstants.claw_ConeStallCurLimitAmps;
        claw.setSmartCurrentLimit(claw_StallCurLimitAmps, 50);
        SmartDashboard.putBoolean("ClawCubeForce", claw_StallCurLimitAmps == ArmConstants.claw_ConeStallCurLimitAmps);
    }

    public void useCubeCurrentLimit(){
        claw_StallCurLimitAmps = ArmConstants.claw_CubeStallCurLimitAmps;
        claw.setSmartCurrentLimit(claw_StallCurLimitAmps, 50);
        SmartDashboard.putBoolean("ClawCubeForce", claw_StallCurLimitAmps == ArmConstants.claw_ConeStallCurLimitAmps);
    }

    public boolean usingLotsOfCurrent() {
        return claw.getOutputCurrent() > 30;
    }

    @Override
    public void periodic() {
        if (DebugSetting.TraceLevel == DebugLevel.Arm || DebugSetting.TraceLevel == DebugLevel.All) {
            SmartDashboard.putNumber("arm speed",extendEncoder.getVelocity());
            SmartDashboard.putNumber("WormPosFbk", wormEncoder.getPosition());
            SmartDashboard.putNumber("ExtendPosFbk", extendEncoder.getPosition());
            SmartDashboard.putNumber("ExtendPosRef", extendPosRef);
            SmartDashboard.putNumber("WormPosRef", wormPosRef);
        }

        if (tunePidMode)
        {
            ReadTuningRefsAndGainsFromSmartDashboard();
            clawSpeedRef = clawSpdRefTune;
            //claw.set(clawSpdRefTune/6000);
            clawPidController.setReference(clawSpdRefTune, ControlType.kVelocity);  //* ArmConstants.ClawMaxPercent, ControlType.kVelocity);
        }
        //The teleop and autonomus commands set up the references
        // send the ref to the PID controllers.
        // The reference will be linited in the send methods.
        sendExtendPosToPidController();
        sendWormRefToPidController();
    }

    /**
     * Change the worm position by an incremental amount. Once the maximum extend is reached
     * the increment will only have a single increment allowed, to allow them to raise just a bit
     * more.
     * @param raise True to raise an incremental amount, false to lower an incremental amount
     */
    public void wormIncremental(boolean raise, boolean larger){
        double increment = larger ? ArmConstants.WormPositionPlaceConeIncrement : ArmConstants.WormPositionIncrement;
        if (raise)
            wormPosRef += (increment * ArmConstants.WormMotorRotationsPerInch);
        else
            wormPosRef -= (increment * ArmConstants.WormMotorRotationsPerInch);
        double raisedAndThenSome = (ArmConstants.WormPositionMax + ArmConstants.WormPositionIncrement) * ArmConstants.WormMotorRotationsPerInch;
        if (wormPosRef > raisedAndThenSome)
            wormPosRef = raisedAndThenSome;
        else if (wormPosRef < (ArmConstants.WormPositionMin * ArmConstants.WormMotorRotationsPerInch))
            wormPosRef = ArmConstants.ExtendPositionMin;
    }

    /**
     * Sets the worm position reference to maximum up
     */
    public void wormFullUp(){
        wormPosRef = ArmConstants.WormPositionMax * ArmConstants.WormMotorRotationsPerInch;
    }

    /**
     * Sets the worm position reference to minimum down.
     */
    public void wormFullDown(){
        wormPosRef = ArmConstants.WormPositionMin * ArmConstants.WormMotorRotationsPerInch;
    }


    private void sendExtendPosToPidController() {
        extendPidControl.setReference(extendPosRef, ControlType.kSmartMotion);
    }

    /**
     * Change the Extend position by an incremental amount. The position is limited to maximum extend and
     * minimum extend.
     * @param extend True to extend an incremental amount, false to retract an incremental amount
     */
    public void extendIncremental(boolean extend){
        if (extend)
            extendPosRef += (ArmConstants.ExtendPositionIncrement * ArmConstants.ExtendMotorRotationsPerInch);
        else
            extendPosRef -= (ArmConstants.ExtendPositionIncrement * ArmConstants.ExtendMotorRotationsPerInch);
        if (extendPosRef > ArmConstants.ExtendPositionMax * ArmConstants.ExtendMotorRotationsPerInch)
            extendPosRef = ArmConstants.ExtendPositionMax;
        else if (extendPosRef < ArmConstants.ExtendPositionMin * ArmConstants.ExtendMotorRotationsPerInch)
            extendPosRef = ArmConstants.ExtendPositionMin;
    }

    /**
     * Sets the extend position reference to maximum out.
     */
    public void extendFullOut(){
        extendPosRef = ArmConstants.ExtendPositionMax * ArmConstants.ExtendMotorRotationsPerInch;
    }

    /**
     * Sets the extend position reference to minimum in.
     */
    public void extendFullIn(){
        extendPosRef = ArmConstants.ExtendPositionMin * ArmConstants.ExtendMotorRotationsPerInch;
    }

    private void sendWormRefToPidController() {
        wormPidControl.setReference(wormPosRef, ControlType.kSmartMotion);
    }

    /**
     * Set the hoist positions to reach the front cone post
     */
    public void frontHoistPos() {
        wormPosRef = ArmConstants.WormMidRung * ArmConstants.WormMotorRotationsPerInch;
    }

    

    /**
     * Set the position to reach the front cone post
     */
    public void frontExtendPos() {
        extendPosRef = ArmConstants.ExtendMidRung * ArmConstants.ExtendMotorRotationsPerInch;
    }

    public void wormIntakePos() {
        wormPosRef = ArmConstants.WormPositionIntake * ArmConstants.WormMotorRotationsPerInch;
    }

    // Set extension to intake pos
    public void extendIntakePos() {
        extendPosRef = ArmConstants.ExtendPositionIntake * ArmConstants.ExtendMotorRotationsPerInch;
    }

    public void weirdConeDrop() {
        extendPosRef = ArmConstants.ExtendFunkyConeDrop * ArmConstants.ExtendMotorRotationsPerInch;
        wormPosRef = ArmConstants.WormFunkyConeDrop * ArmConstants.WormMotorRotationsPerInch;
    }

    //tuning stuff below
    private void PutTuneValuesToSmartDashboard(Boolean includeWorm, Boolean includeExtend, Boolean includeClaw)
    {
        if (includeWorm){
            SmartDashboard.putNumber("wormKP", worm_kP);
            SmartDashboard.putNumber("wormKI", worm_kI);
            SmartDashboard.putNumber("wormFF", worm_kFF);
            SmartDashboard.putNumber("wormMaxVel", wormMaxVel);
            SmartDashboard.putNumber("wormMaxAcc", wormMaxAcc);
            SmartDashboard.putNumber("WormTunePosRef", wormPosRef);
        }
        if (includeExtend){
            SmartDashboard.putNumber("extendKP", extend_kP);
            SmartDashboard.putNumber("extendKI", extend_kI);
            SmartDashboard.putNumber("extendFF", extend_kFF);
            SmartDashboard.putNumber("extendMaxVel", extendMaxVel);
            SmartDashboard.putNumber("extendMaxAcc", extendMaxAcc);
            SmartDashboard.putNumber("ExtendTunePosRef", extendPosRef);
        }
        if (includeClaw){
            SmartDashboard.putNumber("clawKP", claw_kP);
            SmartDashboard.putNumber("clawKI", claw_kI);
            SmartDashboard.putNumber("clawFF", claw_kFF);
            SmartDashboard.putNumber("clawMaxVel", clawMaxVel);
            SmartDashboard.putNumber("clawMaxAcc", clawMaxAcc);
            SmartDashboard.putNumber("clawSpdRef", clawSpeedRef);
            }
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
        double clawkP = SmartDashboard.getNumber("clawKP", claw_kP);
        double clawkI = SmartDashboard.getNumber("clawKI", claw_kI);
        double clawkFF = SmartDashboard.getNumber("clawFF", claw_kFF);
        double clawMVel = SmartDashboard.getNumber("clawMaxVel", clawMaxVel);
        double clawMAcc = SmartDashboard.getNumber("clawMaxAcc", clawMaxAcc);


        SmartDashboard.putNumber("Worm IA", worm.getOutputCurrent());
        SmartDashboard.putNumber("Worm Spd", wormEncoder.getVelocity());
        SmartDashboard.putNumber("Extend IA", extend.getOutputCurrent());
        SmartDashboard.putNumber("Extend Spd", extendEncoder.getVelocity());
        SmartDashboard.putNumber("Claw IA", claw.getOutputCurrent());
        SmartDashboard.putNumber("Claw Spd", clawEncoder.getVelocity());

        //values in motor turns
        wormPosRef = SmartDashboard.getNumber("WormTunePosRef", wormPosRef);
        extendPosRef = SmartDashboard.getNumber("ExtendTunePosRef", extendPosRef);
        clawSpdRefTune = SmartDashboard.getNumber("clawSpdRef", clawSpeedRef);

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
            extend_kI = extendkI;
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
        if (clawkP != claw_kP){
            claw_kP = clawkP;
            diff = true;
        }
        if (clawkI != claw_kI) {
            claw_kI = clawkI;
            diff = true;
        }
        if (clawkFF != claw_kFF) {
            claw_kFF = clawkFF;
            diff = true;
        }
        if (clawMVel != clawMaxVel){
            clawMaxVel = clawMVel;
            diff = true;
        }
        if (clawMAcc != clawMaxAcc){
            clawMaxAcc = clawMAcc;
            diff = true;
        }

        if (diff)
            SetPidGainsForWormExtendClaw();
    }
}
