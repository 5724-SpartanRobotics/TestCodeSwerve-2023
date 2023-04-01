package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    
    private int tagOfInterest;
    private DriveTrainSubsystemRick drive;
    private double[] defaultTag = new double[1];
    private double[] tagValues;

    public VisionSubsystem(DriveTrainSubsystemRick drive) {
        defaultTag[0] = -1;
        this.drive = drive;
        for(int i = 0; i < 8; i++) {
            SmartDashboard.putNumberArray("aprilTag" + i, new double[]{0, 0, 0, 0});
        }
    }

    @Override
    public void periodic() {

    }

    // Set the tag of interest
    public void setTag(int tag) {
        tagOfInterest = tag;
        SmartDashboard.putNumber("wantedTag", tagOfInterest);
    }

    // Check for the tag, return true if found
    public boolean foundTag() {
        double[] temp = SmartDashboard.getNumberArray("aprilTag" + tagOfInterest, defaultTag);
        if(temp[3] == 0){
            return false;
        } else {
            temp[3] = 0;
            SmartDashboard.putNumberArray("aprilTag" + tagOfInterest, temp);
            tagValues = temp;
            return true;
        }
    }
    
    public Pose2d getSetPose() {
        double camPoseX = Math.cos(Math.toRadians(tagValues[1] + drive.getGyroHeading().getDegrees())) * tagValues[0];
        double camPoseY = Math.sin(Math.toRadians(tagValues[1] + drive.getGyroHeading().getDegrees())) * tagValues[0];
        SmartDashboard.putNumber("camToTagX", camPoseX);
        SmartDashboard.putNumber("camToTagY", camPoseY);
        return new Pose2d(new Translation2d(camPoseX, camPoseY), new Rotation2d());
    }

    public void cancel() {
        tagOfInterest = -1;
    }
}
