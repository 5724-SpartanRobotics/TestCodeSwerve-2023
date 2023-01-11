package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.SwerveModule;
import frc.robot.Subsystems.Constant.DriveConstants;
import java.util.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;

public class DriveTrainSubsystem extends SubsystemBase implements DriveTrainInterface {
    
    // Swerve modules
    private SwerveModule LF;
    private SwerveModule RF;
    private SwerveModule LB;
    private SwerveModule RB;

    // Gyro for now
    private ADIS16448_IMU gyro;
    private ADIS16448_IMUSim gyroSim;

    // Commands for swerve motion
    private double xvel = 0;
    private double yvel = 0;
    private double turnvel = 0;

    // for calculations, r is center to wheel, L is wheel to wheel
    private double r = 0.51;
    private double L = 0.71;

    public DriveTrainSubsystem() {
        LF = new SwerveModule(DriveConstants.LFTurnMotor, DriveConstants.LFDriveMotor, DriveConstants.LFCanID, DriveConstants.LFOff, "LF", this);
        RF = new SwerveModule(DriveConstants.RFTurnMotor, DriveConstants.RFDriveMotor, DriveConstants.RFCanID, DriveConstants.RFOff, "RF", this);
        LB = new SwerveModule(DriveConstants.LBTurnMotor, DriveConstants.LBDriveMotor, DriveConstants.LBCanID, DriveConstants.LBOff, "LB", this);
        RB = new SwerveModule(DriveConstants.RBTurnMotor, DriveConstants.RBDriveMotor, DriveConstants.RBCanID, DriveConstants.RBOff, "RB", this);
        gyro = new ADIS16448_IMU();
}

    @Override
    public void periodic() {
        // x and y turn components, temporary for now.
        double xturn = turnvel * 1.4 / 2;
        double yturn = turnvel * 1.4 / 2;

        // TODO Auto-generated method stub
        RF.setCartesian(xvel + xturn, yvel + yturn);
        RB.setCartesian(xvel - yturn, yvel + xturn);
        LF.setCartesian(xvel + yturn, yvel - xturn);
        LB.setCartesian(xvel - xturn, yvel - yturn);
    
        LF.update();
        RF.update();
        LB.update();
        RB.update();
    }

    public void drive(Translation2d translation, double rotation){
        
    }

    public void simulationInit()
    {
        gyroSim = new ADIS16448_IMUSim(gyro);
        LF.simulateInit();
        RF.simulateInit();
        LB.simulateInit();
        RB.simulateInit();
        gyroSim.setGyroAngleY(10);
    }
    @Override
    public void simulationPeriodic()
    {
        if (gyroSim == null)
            simulationInit();
        LF.simulatePeriodic();
        RF.simulatePeriodic();
        LB.simulatePeriodic();
        RB.simulatePeriodic();
    }

    public void driveCommand(double x, double y, double turn) {
        // Convert to polar
        double ang = Math.atan2(y, x);
        double speed = Math.sqrt(y * y + x * x);
        
        // Increase ang by gyro
        ang += gyro.getAngle();

        // Go back to cartesian
        xvel = Math.cos(ang) * speed;
        yvel = Math.sin(ang) * speed;
        turnvel = turn * 0.5;
    }

    public Rotation2d getGyroHeading(){
        return new Rotation2d();
        //the gyro getGyroAngleY returns positive values as the robot turns clockwise. We want negative for clockwise
//        return Rotation2d.fromDegrees(-gyro.getGyroAngleY());
    }

}
