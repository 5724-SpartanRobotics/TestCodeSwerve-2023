
package frc.robot.commands.HelixAutoTools;

import frc.robot.commands.HelixAutoTools.SwerveTrajectory;

import frc.robot.commands.HelixAutoTools.Paths.Path;

public class ImportedTrajectory extends Path {

    private SwerveTrajectory trajectory;

    public ImportedTrajectory(SwerveTrajectory trajectory) {
        this.trajectory = trajectory;
    }

    @Override
    public SwerveTrajectory getPath() {
        return trajectory;
    }
    
}