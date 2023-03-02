package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
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

        public static final int PigeonID = 49;
    }
    public static final class ArmConstants{
        public static final int ClawMotor = 12;
        public static final int WormMotor = 17;
        public static final double ClawMaxPercent = 0.35;
        public static final int claw_ConeStallCurLimitAmps = 65;
        public static final int claw_CubeStallCurLimitAmps = 50;

        //gear ratio is 4 and the worm gear motion isn't linear. one turn at the bottom has more arm angle
        //than a turn at the top. The desired max position determined imperically was 110 motor rotations
        // so we made this ration 7.33, 
        public static final double WormMotorRotationsPerInch = 7.33;
        public static final double WormPositionMax = 17;//inches of rod
        public static final double WormPositionFrontCone = 14;
        public static final double WormPositionMin = 0;//inches of rod
        public static final double WormPositionOutsideRobot = 5; //inches of rod
        public static final double ExtendMotorRotationsPerInch = 3.8197;//Spool size is 1" circumfrence is pi, therefore 1/pi spool rotations = 1 inch. Gear box is 12:1.
        public static final double WormPositionHPCone = 14.2;
        public static final double WormPositionHPCube = 14.7;
        public static final int ExtendMotor = 13;
        public static final double ExtendPositionMax = 20; //inches
        public static final double ExtendPositionFrontCone = 5;
        public static final double ExtendPositionFloorCone = 1.4;
        public static final double ExtendPositionFloorCube = 0;
        public static final double ExtendPositionMin = 0; //inches
    }
    public static final class ControllerConstants{
        public static double joystickDeadband = 0.1;//a deadband that you must overcome for the joystick input, otherwise we send 0
    }
    public static final class DebugSetting{
        public static final DebugLevel TraceLevel = DebugLevel.Arm;//set this to get more values to smart dashboard.
    }
    public static enum DebugLevel{
        Off,
        Swerve,
        Arm,
        All
    }
    public static final class AutoConstants {
        public static final double kPTranslationController = 0.1;
        public static final double kPThetaController = 0.1;
    }
}
