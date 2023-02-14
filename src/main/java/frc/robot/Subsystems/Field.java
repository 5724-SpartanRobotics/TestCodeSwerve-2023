package frc.robot.Subsystems;

import java.util.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Field {

    private double[] tagX;
    private double[] tagY;
    private double targetX;
    private double targetY;
    private double targetRot;
    private DriveTrainSubsystemRick drive;
    private double desiredZ;
    private double zDif = 0;
    private double turnAccuracy = 10;
    private double autoTurnSpeed = 0.5;
    private double autoDriveSpeed = 0.5;
    private List<Double> xGuesses;
    private List<Double> yGuesses;
    private double[] dummyArray = new double[]{-1, -1, -1};
    private double gyro = 0;
    private double lastTime = 0;
    private double moveX = 0;
    private double moveY = 0;
    private double[] retArray = new double[3];

    public Field(DriveTrainSubsystemRick d) {
        drive = d;
        tagX = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0};
        tagY = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0};
    }
    
    public double[] update() {
        gyro = drive.getGyroHeading().getDegrees();
        zDif = gyro % 360 - targetRot;
        if(zDif > 180) {
            zDif -= 360;
        }
        if(zDif > turnAccuracy) {
            desiredZ = -autoTurnSpeed;
        } else if(zDif < -turnAccuracy) {
            desiredZ = -autoTurnSpeed;
        } else {
            desiredZ = 0;
        }
        // double[] temp = SmartDashboard.getNumberArray("aprilTag" + 5, dummyArray);
        // double tempAng = 0;
        // if(temp[2] == 0) {
        //     temp[2] = 1;
        //     SmartDashboard.putNumberArray("aprilTag" + 5, temp);
        //     tempAng = gyro + temp[1] + 180;
        //     yGuesses.add(Math.sin(Math.toRadians(tempAng)) * temp[0] + tagY[4]);
        //     xGuesses.add(Math.cos(Math.toRadians(tempAng)) * temp[0] + tagX[4]);            
        // }
        // if(xGuesses.size() > 1) {
        //     xGuesses.remove(0);
        //     yGuesses.remove(0);
        // }
        // moveX = targetX - xGuesses.get(0) / 5;
        // moveY = targetY - yGuesses.get(0) / 5;
        // if(moveX > autoDriveSpeed) {
        //     moveX = autoDriveSpeed;
        // }
        // if(moveY > autoDriveSpeed) {
        //     moveY = autoDriveSpeed;
        // }
        retArray = new double[]{moveX, moveY, desiredZ};
        return retArray;
    }

    // Note tZ, turn, should be in degrees
    public void setTarget(double tX, double tY, double tZ) {
        targetX = tX;
        targetY = tY;
        targetRot = tZ;
    }
}
