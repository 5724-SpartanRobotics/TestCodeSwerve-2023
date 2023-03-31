package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    
    private int tagOfInterest;
    private DriveTrainSubsystemRick drive;
    private double[] defaultTag = new double[1];

    public VisionSubsystem(DriveTrainSubsystemRick drive) {
        defaultTag[0] = -1;
        this.drive = drive;
    }

    @Override
    public void periodic(){
        if(tagOfInterest != -1) {
            checkFor(tagOfInterest);
        }
    }

    public void flagForVision(int aprilTag) {
        tagOfInterest = aprilTag;
    }

    private void checkFor(int aprilTag) {
        double[] tagPos = SmartDashboard.getNumberArray("aprilTag" + aprilTag, defaultTag);
        
    }
}
