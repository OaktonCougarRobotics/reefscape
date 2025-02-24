package frc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class AT{
    private Pose2d aprilTagPose;
    private Pose2d offsetPose;
    private int id;

    public AT(double x, double y, double theta, int id)
    {
        this.aprilTagPose = new Pose2d(x, y, new Rotation2d(theta * 2 * (Math.PI)/360));
        this.offsetPose = new Pose2d(x + Math.cos(Constants.ANATOLI_CHASSIS_WIDTH/2), 
        y + Math.cos(Constants.ANATOLI_CHASSIS_WIDTH/2), 
        new Rotation2d(theta * 2 * (Math.PI)/360 + Math.PI));
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