package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constant {
    public static final class DriveConstants {
        public static final double LFOff = 1.52;//CANCoder offset in radians
        public static final int LFTurnMotor = 7;
        public static final int LFDriveMotor = 18;
        public static final int LFCanID = 61;
        public static final double RFOff = 1.71;
        public static final int RFTurnMotor = 21;
        public static final int RFDriveMotor = 5;
        public static final int RFCanID = 62;
        public static final double LBOff = 5.04;
        public static final int LBTurnMotor = 6;
        public static final int LBDriveMotor = 4;
        public static final int LBCanID = 60;
        public static final double RBOff = 2.83;
        public static final int RBTurnMotor = 3;
        public static final int RBDriveMotor = 2;
        public static final int RBCanID = 59;
        public static final double trackWidth = Units.inchesToMeters(30.0);//wheel center to center width
        public static final double wheelBase = Units.inchesToMeters(30.0);//wheel center to center front / back distance
        public static final double wheelDiameter = Units.inchesToMeters(4.125);//guessing there is about 1/8" added for the tread. The wheel diameter is 4"
        public static final double wheelCircumfrence = wheelDiameter * Math.PI;//meters
        public static final double driveGearRatio = 6.75;
        public static final double maxMotorRpm = 6380;
        public static final double maxWheelRpm = maxMotorRpm / driveGearRatio;
        public static final double maxRobotSpeedmps = maxWheelRpm / 60 * wheelCircumfrence;//should be 5.1853 mps
        //Swerve locations relative to the center of the robot. Positive x values represent moving toward the front of the robot whereas positive y values
        // represent moving toward the left of the robot. Distances are in meters.
        public static Translation2d LFLocation = new Translation2d(wheelBase/2, trackWidth/2);
        public static Translation2d RFLocation = new Translation2d(wheelBase/2, -trackWidth/2);
        public static Translation2d LBLocation = new Translation2d(-wheelBase/2, trackWidth/2);
        public static Translation2d RBLocation = new Translation2d(-wheelBase/2, -trackWidth/2);
        public static double turnGearRatio = 150.0 / 7.0;
        /**Maximum angular velocity in degrees per second */
        public static double maxAngularVelocityRps = 10.0;
    }
    public static final class ControllerConstants{
        public static double joystickDeadband = 0.1;//a deadband that you must overcome for the joystick input, otherwise we send 0
    }
    public static final class DebugSetting{
        public static final DebugLevel TraceLevel = DebugLevel.Verbose;//set this to verbose to get more values to smart dashboard.
    }
    public static enum DebugLevel{
        Info,
        Verbose
    }
}
