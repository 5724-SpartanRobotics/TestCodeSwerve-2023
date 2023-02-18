package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Constant.DebugLevel;
import frc.robot.Subsystems.Constant.DebugSetting;
import frc.robot.Subsystems.Constant.DriveConstants;

public class DriveTrainSubsystemRick extends SubsystemBase implements DriveTrainInterface {
        // Swerve modules
        private SwerveModule LF;
        private SwerveModule RF;
        private SwerveModule LB;
        private SwerveModule RB;
        private Rotation2d lastUpdatedGyroHeading;
    
        // Used by helix, should eventually be used to zero gyro with a button
        private double offset = 0;

        // Gyro for now
        // private ADIS16448_IMU gyro;
        // private ADIS16448_IMUSim gyroSim;
        private ADXRS450_Gyro gyroFake;
        private ADXRS450_GyroSim gyroSim;
        private Pigeon2 gyro;

        // TODO make pigeon sim actually like, work

        private SwerveDriveKinematics swerveDriveKinematics;
        private SwerveDriveOdometry swerveDriveOdometry;
        private Pose2d robotPose;
        private SwerveModule[] modules;
    
        public DriveTrainSubsystemRick() {
//            gyro = new ADIS16448_IMU();
            gyro = new Pigeon2(DriveConstants.PigeonID);
            UpdateGyro();
            LF = new SwerveModule(DriveConstants.LFTurnMotor, DriveConstants.LFDriveMotor, DriveConstants.LFCanID, DriveConstants.LFOff, "LF", this);
            RF = new SwerveModule(DriveConstants.RFTurnMotor, DriveConstants.RFDriveMotor, DriveConstants.RFCanID, DriveConstants.RFOff, "RF", this);
            LB = new SwerveModule(DriveConstants.LBTurnMotor, DriveConstants.LBDriveMotor, DriveConstants.LBCanID, DriveConstants.LBOff, "LB", this);
            RB = new SwerveModule(DriveConstants.RBTurnMotor, DriveConstants.RBDriveMotor, DriveConstants.RBCanID, DriveConstants.RBOff, "RB", this);
            swerveDriveKinematics = new SwerveDriveKinematics(DriveConstants.LFLocation, DriveConstants.RFLocation, DriveConstants.LBLocation, DriveConstants.RBLocation);
            robotPose = new Pose2d(new Translation2d(4.0, 5.0), new Rotation2d());//starting pose doesn't really matter. We will call reset based on robot initial field position.
            SwerveModulePosition[] swerveInititialPositions = {LF.getPosition(), RF.getPosition(), LB.getPosition(), RB.getPosition()};
            swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics, getGyroHeading(), swerveInititialPositions, robotPose);
            modules = new SwerveModule[] {LF, RF, LB, RB};
        }

        public Rotation2d getGyroHeading(){
            return lastUpdatedGyroHeading;
        }

        // Used by helixnavigator
        public Rotation2d getHeading(){
            double raw_yaw = gyro.getYaw() - offset; // Returns yaw as -180 to +180.
            // float raw_yaw = m_ahrs.getYaw(); // Returns yaw as -180 to +180.
            double calc_yaw = raw_yaw;

            if (0.0 > raw_yaw ) { // yaw is negative
                calc_yaw += 360.0;
            }
            return Rotation2d.fromDegrees(-calc_yaw);
        }

        @Override
        public void periodic(){
            //the gyro getGyroAngleY returns positive values as the robot turns clockwise. We want negative for clockwise
            LF.periodic();
            LB.periodic();
            RF.periodic();
            RB.periodic();
            if (DebugSetting.TraceLevel == DebugLevel.Verbose){
                SmartDashboard.putNumber("Gyro Heading Deg", getGyroHeading().getDegrees());
            }
            UpdateGyro();
            robotPose = swerveDriveOdometry.update(getGyroHeading(), new SwerveModulePosition[] {LF.getPosition(), RF.getPosition(), LB.getPosition(), RB.getPosition()});
        }

        private void UpdateGyro() {
            lastUpdatedGyroHeading = Rotation2d.fromDegrees(gyro.getYaw());
        }

        public void drive(Translation2d translation, double rotation){
            SwerveModuleState[] swerveModStates = swerveDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroHeading()));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModStates, DriveConstants.maxRobotSpeedmps);
            LF.setDesiredState(swerveModStates[0]);
            RF.setDesiredState(swerveModStates[1]);
            LB.setDesiredState(swerveModStates[2]);
            RB.setDesiredState(swerveModStates[3]);
        }

        public void simulationInit()
        {
//            gyroSim = new ADIS16448_IMUSim(gyro);
            gyroSim = new ADXRS450_GyroSim(gyroFake);
            LF.simulateInit();
            RF.simulateInit();
            LB.simulateInit();
            RB.simulateInit();
//          gyroSim.setGyroAngleY(0);
            gyroSim.setAngle(0);
        }
        @Override
        public void simulationPeriodic(){
            if (gyroSim == null)
                simulationInit();
//            gyroSim.setGyroAngleY(Units.radiansToDegrees(0.0));
            gyroSim.setAngle(0.0);
            LF.simulatePeriodic();
            RF.simulatePeriodic();
            LB.simulatePeriodic();
            RB.simulatePeriodic();
            }

            // used by helix
        public void brake() {
            for (SwerveModule module : modules) {
                module.setDesiredState(new SwerveModuleState(0, module.getState().angle));
            }
        }
        public Pose2d getPose() {
            return swerveDriveOdometry.getPoseMeters();
        }
}
