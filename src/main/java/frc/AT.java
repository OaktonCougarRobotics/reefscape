package frc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class AT{
    private Pose2d aprilTagPose;
    private Pose2d offsetPose;
    private int id;

    public AT(double xIn, double yIn, double thetaDeg, int id)
    {
        double x = xIn / 39.3701;
        double y = yIn / 39.3701;
        double theta = thetaDeg * Math.PI / 180;
        this.aprilTagPose = new Pose2d(x, y, new Rotation2d(theta));
        this.offsetPose = new Pose2d(x + Constants.ANATOLI_CHASSIS_WIDTH/2 * Math.cos(theta), 
        y + Constants.ANATOLI_CHASSIS_WIDTH/2 * Math.sin(theta), 
        new Rotation2d(theta + Math.PI));
        this.id = id;
    } 

    public Pose2d getPose() {
        return aprilTagPose;
    }

    public Pose2d getOffestPose()
    {
        return offsetPose;
    }

    public int getId() {
        return id;
    }
}